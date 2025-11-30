import math
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist


class WallFollower(Node):
    def __init__(self):
        super().__init__('wall_follower_node')

        # Parameters (pots ajustar-los via ROS params)
        self.declare_parameter('distance_limit', 0.5)    # desired distance to walls
        self.declare_parameter('forward_speed', 0.20)    # base linear speed (m/s)
        self.declare_parameter('time_to_stop', 30.0)     # auto-stop
        self.declare_parameter('tolerance', 0.05)        # tolerance band

        self.base_distance = float(self.get_parameter('distance_limit').value)
        self.v_lin = float(self.get_parameter('forward_speed').value)
        self.time_to_stop = float(self.get_parameter('time_to_stop').value)
        self.tol = float(self.get_parameter('tolerance').value)

        # Gains for correction (simple P-type behaviour)
        self.k_corr = 1.0  # multiplica l'error per obtenir la correcció (scale)

        # Last commanded twist (published periodically)
        self.cmd = Twist()

        # ROS 2: subs i pub
        self.subscription = self.create_subscription(
            LaserScan, '/scan', self.laser_callback, qos_profile_sensor_data
        )
        self.publisher = self.create_publisher(Twist, '/cmd_vel', 10)

        # Timers
        self.info_timer = self.create_timer(1.0, self.log_info)
        self.stop_timer = self.create_timer(0.05, self.stop_watchdog)
        self.cmd_timer = self.create_timer(0.1, self.cmd_publish_timer_cb)  # 10 Hz

        self._state_action = "Idle"
        self._last_action_logged = None
        self._shutting_down = False

        self.start_time_s = self.get_clock().now().nanoseconds * 1e-9

        # Estat de moviment actual (inicial: moure's endavant en X)
        # valors: 'vx_pos' (cap amunt/endavant), 'vy_pos' (cap esquerra), etc.
        self.current_motion = 'vx_pos'

        self.get_logger().info("WallFollower (holonomic, no rotations) - iniciat.")

    # ---------------------------
    def stop_watchdog(self):
        if self._shutting_down:
            return
        now = self.get_clock().now().nanoseconds * 1e-9
        if now - self.start_time_s >= self.time_to_stop:
            self.get_logger().info("Stopping due to timeout.")
            self.stop()

    def stop(self):
        self._shutting_down = True
        self.cmd = Twist()
        try:
            self.publisher.publish(self.cmd)
        except Exception:
            pass
        for t in [self.info_timer, self.stop_timer, self.cmd_timer]:
            try:
                t.cancel()
            except Exception:
                pass

    def cmd_publish_timer_cb(self):
        if self._shutting_down:
            return
        try:
            self.publisher.publish(self.cmd)
        except Exception:
            pass

    # ---------------------------
    def get_sector_angles(self):
        # Defineix límits angulars (graus) per sector respecte front = 0, augmentant cap a l'esquerra (+)
        return {
            'front': (-22.5, 22.5),
            'front_right': (-67.5, -22.5),
            'right': (-112.5, -67.5),
            'back_right': (-157.5, -112.5),
            'back': (157.5, -157.5),  # atravessa -180/180
            'back_left': (112.5, 157.5),
            'left': (67.5, 112.5),
            'front_left': (22.5, 67.5),
        }

    def normalize_angle_deg(self, a):
        a = ((a + 180.0) % 360.0) - 180.0
        return a

    def angle_in_sector(self, ang_deg, sector):
        limits = self.get_sector_angles()[sector]
        lo, hi = limits
        ang = self.normalize_angle_deg(ang_deg)
        if lo <= hi:
            return lo <= ang <= hi
        else:
            return ang >= lo or ang <= hi

    # ---------------------------
    def laser_callback(self, scan: LaserScan):
        if self._shutting_down:
            return

        angle_min = math.degrees(scan.angle_min)
        angle_inc = math.degrees(scan.angle_increment)

        sectors = {
            'front': [], 'front_right': [], 'right': [], 'back_right': [],
            'back': [], 'back_left': [], 'left': [], 'front_left': []
        }

        for i, d in enumerate(scan.ranges):
            if not math.isfinite(d):
                continue
            if d < scan.range_min or d > scan.range_max:
                continue
            ang = angle_min + i * angle_inc
            ang = self.normalize_angle_deg(ang)
            for s in sectors:
                if self.angle_in_sector(ang, s):
                    sectors[s].append(d)
                    break

        mins = {s: (min(v) if v else float('inf')) for s, v in sectors.items()}
        closest_sector = min(mins, key=lambda k: mins[k])
        tol_presence = self.base_distance + self.tol + 0.01
        present = {s: (mins[s] < tol_presence) for s in mins}

        twist = Twist()
        action = ""

        def clamp(x, a, b):
            return max(a, min(b, x))

        def correction_for(error):
            corr = self.k_corr * error
            corr = clamp(corr, -self.v_lin, self.v_lin)
            return corr

        # ---------- Regles combinades (dos murs) - corregides segons la teva convenció d'eixos
        # front + left -> moure's cap abaix (vx_neg)
        # left + back  -> moure's cap dreta (vy_neg)
        # back + right -> moure's cap amunt (vx_pos)   <-- CORRECCIÓ AQUI
        # right + front -> moure's cap esquerra (vy_pos) <-- CORRECCIÓ AQUI
        if (present['front'] and present['front_left']) or (present['front'] and present['left']):
            twist.linear.x = -self.v_lin
            twist.linear.y = 0.0
            self.current_motion = 'vx_neg'
            action = f"FRONT+LEFT present -> MOVE DOWN (vx_neg)"
        elif (present['left'] and present['back_left']) or (present['left'] and present['back']):
            twist.linear.x = 0.0
            twist.linear.y = -self.v_lin
            self.current_motion = 'vy_neg'
            action = f"LEFT+BACK present -> MOVE RIGHT (vy_neg)"
        elif (present['back'] and present['back_right']) or (present['back'] and present['right']):
            # back + right -> mover cap amunt (vx_pos)
            twist.linear.x = self.v_lin
            twist.linear.y = 0.0
            self.current_motion = 'vx_pos'
            action = f"BACK+RIGHT present -> MOVE UP (vx_pos)"
        elif (present['right'] and present['front_right']) or (present['right'] and present['front']):
            # right + front -> moure's cap esquerra (vy_pos)
            twist.linear.x = 0.0
            twist.linear.y = self.v_lin
            self.current_motion = 'vy_pos'
            action = f"RIGHT+FRONT present -> MOVE LEFT (vy_pos)"
        else:
            # Reglas senzilles per una sola paret present
            if present['front']:
                target = self.base_distance
                error = mins['front'] - target
                vx = 0.0 if abs(error) <= self.tol else correction_for(error)
                twist.linear.x = vx
                twist.linear.y = self.v_lin  # moure's cap esquerra (vy_pos)
                self.current_motion = 'vy_pos'
                action = f"FRONT present -> MOVE LEFT (vy_pos), corr Vx={vx:.2f}"

            elif present['left']:
                target = self.base_distance
                error = mins['left'] - target
                vy = 0.0 if abs(error) <= self.tol else correction_for(error)
                twist.linear.x = self.v_lin  # moure's cap amunt/endavant (vx_pos)
                twist.linear.y = vy
                self.current_motion = 'vx_pos'
                action = f"LEFT present -> MOVE UP (vx_pos), corr Vy={vy:.2f}"

            elif present['back']:
                target = self.base_distance
                error = mins['back'] - target
                vx_corr = 0.0 if abs(error) <= self.tol else correction_for(error)
                # Mourem cap abaix (vx_neg) i ajustem segons error
                twist.linear.x = -self.v_lin + vx_corr
                twist.linear.y = 0.0
                self.current_motion = 'vx_neg'
                action = f"BACK present -> MOVE DOWN (vx_neg), corr Vx adj {vx_corr:.2f}"

            elif present['right']:
                # Quan topa amb la paret a la dreta, segons la correcció feta: moure's cap amunt (vx_pos)
                target = self.base_distance
                error = mins['right'] - target
                vx_adj = 0.0 if abs(error) <= self.tol else correction_for(error)
                twist.linear.x = self.v_lin + vx_adj  # mover cap amunt (vx_pos) i aplicar correcció en vx
                twist.linear.y = 0.0
                # Nota: si prefereixes que la correcció per pared lateral s'apliqui en Vy, ho podem canviar.
                self.current_motion = 'vx_pos'
                action = f"RIGHT present -> MOVE UP (vx_pos), corr Vx adj {vx_adj:.2f}"

            else:
                # Recovery segons sector més proper (diagonals)
                if closest_sector == 'back_left':
                    twist.linear.x = -0.5 * self.v_lin
                    twist.linear.y = -0.5 * self.v_lin
                    self.current_motion = 'vx_neg'
                    action = "RECOVERY: closest back_left -> diagonal down-right"
                elif closest_sector == 'back_right':
                    twist.linear.x = -0.5 * self.v_lin
                    twist.linear.y = 0.5 * self.v_lin
                    self.current_motion = 'vx_neg'
                    action = "RECOVERY: closest back_right -> diagonal down-left"
                elif closest_sector == 'front_left':
                    twist.linear.x = 0.5 * self.v_lin
                    twist.linear.y = 0.5 * self.v_lin
                    self.current_motion = 'vx_pos'
                    action = "RECOVERY: closest front_left -> diagonal up-left"
                elif closest_sector == 'front_right':
                    twist.linear.x = 0.5 * self.v_lin
                    twist.linear.y = -0.5 * self.v_lin
                    self.current_motion = 'vx_pos'
                    action = "RECOVERY: closest front_right -> diagonal up-right"
                elif closest_sector == 'front':
                    twist.linear.x = self.v_lin
                    twist.linear.y = 0.0
                    self.current_motion = 'vx_pos'
                    action = "No present walls, but closest FRONT -> MOVE UP"
                elif closest_sector == 'left':
                    twist.linear.x = 0.0
                    twist.linear.y = self.v_lin
                    self.current_motion = 'vy_pos'
                    action = "No present walls, but closest LEFT -> MOVE LEFT"
                elif closest_sector == 'right':
                    twist.linear.x = 0.0
                    twist.linear.y = -self.v_lin
                    self.current_motion = 'vy_neg'
                    action = "No present walls, but closest RIGHT -> MOVE RIGHT"
                elif closest_sector == 'back':
                    twist.linear.x = -self.v_lin
                    twist.linear.y = 0.0
                    self.current_motion = 'vx_neg'
                    action = "No present walls, but closest BACK -> MOVE DOWN"
                else:
                    twist = Twist()
                    action = "No walls visible -> STOP"

        # Limits
        twist.linear.x = clamp(twist.linear.x, -self.v_lin, self.v_lin)
        twist.linear.y = clamp(twist.linear.y, -self.v_lin, self.v_lin)
        twist.angular.z = 0.0

        self.cmd = twist
        if action != self._last_action_logged:
            self.get_logger().info(action)
            self._last_action_logged = action
        self._state_action = action

    # ---------------------------
    def log_info(self):
        if not self._shutting_down:
            self.get_logger().info(self._state_action)


def main(args=None):
    rclpy.init(args=args)
    node = WallFollower()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.stop()
    finally:
        try:
            node.destroy_node()
        except Exception:
            pass
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
