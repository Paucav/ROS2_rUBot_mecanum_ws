import math
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist


class WallFollower(Node):
    """
    Wall follower per robot holonomic seguint les regles CCW.
    Ara amb diagonals de recuperació reduïdes (recover_speed_factor).
    """

    def __init__(self):
        super().__init__('wall_follower_node')

        # paràmetres
        self.declare_parameter('distance_limit', 0.5)        # umbral per considerar que hi ha paret
        self.declare_parameter('forward_speed', 0.20)       # velocitat base (m/s)
        self.declare_parameter('time_to_stop', 300.0)       # auto-stop (llarg per testing)
        self.declare_parameter('recover_speed_factor', 0.4) # factor per velocitat diagonal de recover (0..1)

        self.base_distance = float(self.get_parameter('distance_limit').value)
        self.v_lin = float(self.get_parameter('forward_speed').value)
        self.time_to_stop = float(self.get_parameter('time_to_stop').value)
        self.recover_speed_factor = float(self.get_parameter('recover_speed_factor').value)

        # publisher / subscriber
        self.subscription = self.create_subscription(
            LaserScan, '/scan', self.laser_callback, qos_profile_sensor_data
        )
        self.publisher = self.create_publisher(Twist, '/cmd_vel', 10)

        # timers
        self.cmd_timer = self.create_timer(0.1, self.cmd_publish_timer_cb)
        self.info_timer = self.create_timer(1.0, self.log_info)
        self.stop_timer = self.create_timer(0.05, self.stop_watchdog)

        self.cmd = Twist()
        self._shutting_down = False

        # estat: FOLLOW o RECOVER
        self.state = 'FOLLOW'
        # sector actual que seguim: 'front' / 'left' / 'back' / 'right'
        self.current_sector = None

        # en RECOVER guardem el target sector i la diagonal a aplicar (vx, vy)
        self.recover_target = None
        self.recover_diag = (0.0, 0.0)

        self.start_time_s = self.get_clock().now().nanoseconds * 1e-9

        # definició sectors (graus)
        self.sector_angles = {
            'front': (-22.5, 22.5),
            'front_right': (-67.5, -22.5),
            'right': (-112.5, -67.5),
            'back_right': (-157.5, -112.5),
            'back': (157.5, -157.5),
            'back_left': (112.5, 157.5),
            'left': (67.5, 112.5),
            'front_left': (22.5, 67.5),
        }

        # ordre CCW de sectors principals (només els 4 principals)
        self.ccw_order = ['front', 'left', 'back', 'right']

        # mapping sector -> moviment principal (vx, vy)
        self.follow_velocity = {
            'front': (0.0, +self.v_lin),  # vy+
            'left': (-self.v_lin, 0.0),   # vx-
            'back': (0.0, -self.v_lin),   # vy-
            'right': (+self.v_lin, 0.0),  # vx+
        }

        # recovery_table amb signes; diag es calcula com sign * recover_speed
        self.recovery_table = {
            'back': {
                'loss_indicator': 'back_left',
                'diag_signs': (-1, -1),  # vx-, vy-
                'target': 'left'
            },
            'left': {
                'loss_indicator': 'front_left',
                'diag_signs': (-1, +1),  # vx-, vy+
                'target': 'front'
            },
            'front': {
                'loss_indicator': 'front_right',
                'diag_signs': (+1, +1),  # vx+, vy+
                'target': 'right'
            },
            'right': {
                'loss_indicator': 'back_right',
                'diag_signs': (+1, -1),  # vx+, vy-
                'target': 'back'
            },
        }

        self.get_logger().info("WallFollower (holonomic, CCW) iniciat. recover_speed_factor=%.2f" % self.recover_speed_factor)

    # --------------------
    def stop_watchdog(self):
        if self._shutting_down:
            return
        now = self.get_clock().now().nanoseconds * 1e-9
        if now - self.start_time_s >= self.time_to_stop:
            self.get_logger().info("Timeout reached -> stopping")
            self.stop()

    def stop(self):
        self._shutting_down = True
        self.cmd = Twist()
        try:
            self.publisher.publish(self.cmd)
        except Exception:
            pass
        try:
            self.cmd_timer.cancel()
            self.info_timer.cancel()
            self.stop_timer.cancel()
        except Exception:
            pass

    def cmd_publish_timer_cb(self):
        if self._shutting_down:
            return
        try:
            self.publisher.publish(self.cmd)
        except Exception:
            pass

    def normalize_angle_deg(self, a):
        return ((a + 180.0) % 360.0) - 180.0

    def angle_in_sector(self, ang_deg, sector):
        lo, hi = self.sector_angles[sector]
        ang = self.normalize_angle_deg(ang_deg)
        if lo <= hi:
            return lo <= ang <= hi
        else:
            return ang >= lo or ang <= hi

    # --------------------
    def laser_callback(self, scan: LaserScan):
        if self._shutting_down:
            return

        angle_min = math.degrees(scan.angle_min)
        angle_inc = math.degrees(scan.angle_increment)

        # acumular ranges per sector
        sectors = {s: [] for s in self.sector_angles.keys()}

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

        # mínims per sector
        mins = {s: (min(vals) if vals else float('inf')) for s, vals in sectors.items()}

        # sector més proper (closest)
        closest_sector = min(mins, key=lambda k: mins[k])

        # quins sectors considerem "present" (hi ha paret) segons base_distance
        present = {s: (mins[s] <= self.base_distance) for s in mins}

        # si current_sector no inicialitzat, intentem determinar-lo
        if self.current_sector is None:
            primary = ['front', 'left', 'back', 'right']
            candidates = [s for s in primary if present[s]]
            if candidates:
                self.current_sector = min(candidates, key=lambda k: mins[k])
                self.get_logger().info(f"Initial current_sector := {self.current_sector}")
            else:
                self.current_sector = min(primary, key=lambda k: mins[k])
                self.get_logger().info(f"Initial (no present) current_sector := {self.current_sector}")

        twist = Twist()
        action = ""

        # velocitat de recover: escalada de v_lin
        recover_speed = self.v_lin * max(0.0, min(1.0, self.recover_speed_factor))

        # ---------- FOLLOW state ----------
        if self.state == 'FOLLOW':
            if present.get(self.current_sector, False):
                idx = self.ccw_order.index(self.current_sector)
                next_idx = (idx + 1) % len(self.ccw_order)
                next_sector = self.ccw_order[next_idx]

                if present.get(next_sector, False):
                    # cantonada: transició al següent sector CCW
                    self.current_sector = next_sector
                    vx, vy = self.follow_velocity[self.current_sector]
                    twist.linear.x = vx
                    twist.linear.y = vy
                    action = f"CORNER -> switch to {self.current_sector}"
                else:
                    vx, vy = self.follow_velocity[self.current_sector]
                    twist.linear.x = vx
                    twist.linear.y = vy
                    action = f"FOLLOW {self.current_sector}"
            else:
                info = self.recovery_table.get(self.current_sector)
                if info and closest_sector == info['loss_indicator']:
                    # iniciem recover usant signes i recover_speed
                    sx, sy = info['diag_signs']
                    self.recover_diag = (sx * recover_speed, sy * recover_speed)
                    self.recover_target = info['target']
                    self.state = 'RECOVER'
                    twist.linear.x = self.recover_diag[0]
                    twist.linear.y = self.recover_diag[1]
                    action = f"START RECOVER (loss {closest_sector}) diag={self.recover_diag} target={self.recover_target}"
                else:
                    # fallback: diagonal heurística (també escalada)
                    diag_signs_map = {
                        'back_left': (-1, -1),
                        'back_right': (+1, -1),
                        'front_left': (-1, +1),
                        'front_right': (+1, +1),
                        'front': (0, +1),
                        'left': (-1, 0),
                        'back': (0, -1),
                        'right': (+1, 0),
                    }
                    signs = diag_signs_map.get(closest_sector, (0, 0))
                    self.recover_diag = (signs[0] * recover_speed, signs[1] * recover_speed)
                    fallback_target = {
                        'back_left': 'left', 'front_left': 'front',
                        'front_right': 'right', 'back_right': 'back',
                        'front': 'front', 'left': 'left', 'back': 'back', 'right': 'right'
                    }
                    self.recover_target = fallback_target.get(closest_sector, 'front')
                    self.state = 'RECOVER'
                    twist.linear.x = self.recover_diag[0]
                    twist.linear.y = self.recover_diag[1]
                    action = f"START RECOVER fallback (closest {closest_sector}) diag={self.recover_diag} target={self.recover_target}"

        # ---------- RECOVER state ----------
        elif self.state == 'RECOVER':
            twist.linear.x = self.recover_diag[0]
            twist.linear.y = self.recover_diag[1]
            action = f"RECOVERING diag vx={twist.linear.x:.2f} vy={twist.linear.y:.2f} target={self.recover_target}"
            if present.get(self.recover_target, False):
                # finalitzat recover: passem a FOLLOW del target
                self.current_sector = self.recover_target
                self.state = 'FOLLOW'
                vx, vy = self.follow_velocity[self.current_sector]
                twist.linear.x = vx
                twist.linear.y = vy
                action = f"RECOVER DONE -> FOLLOW {self.current_sector}"
                self.recover_target = None
                self.recover_diag = (0.0, 0.0)

        # assegura límits
        def clamp(x, a, b):
            return max(a, min(b, x))

        twist.linear.x = clamp(twist.linear.x, -self.v_lin, self.v_lin)
        twist.linear.y = clamp(twist.linear.y, -self.v_lin, self.v_lin)
        twist.angular.z = 0.0

        self.cmd = twist
        if action != getattr(self, '_last_log', None):
            self.get_logger().info(action)
            self._last_log = action
        self._state_action = action

    # --------------------
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
