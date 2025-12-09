import math
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist


class WallFollower(Node):
    def __init__(self):
        super().__init__('wall_follower_node')

        # Parameters
        self.declare_parameter('distance_limit', 0.5)    # desired distance to right wall
        self.declare_parameter('forward_speed', 0.20)    # linear speed
        self.declare_parameter('turn_speed', 0.40)       # angular speed
        self.declare_parameter('time_to_stop', 30.0)     # auto-stop
        self.declare_parameter('tolerance', 0.05)        # band around base_distance (RIGHT)

        self.base_distance = float(self.get_parameter('distance_limit').value)
        self.v_lin = float(self.get_parameter('forward_speed').value)
        self.v_ang = float(self.get_parameter('turn_speed').value)
        self.time_to_stop = float(self.get_parameter('time_to_stop').value)
        self.tol = float(self.get_parameter('tolerance').value)

        # Last commanded twist (will be published periodically)
        self.cmd = Twist()

        self.prev_vx = 0.0
        self.prev_vy = 0.0
        self.front_wall_type = None
        
        # ROS 2 entities
        self.subscription = self.create_subscription(
            LaserScan, '/scan', self.laser_callback, qos_profile_sensor_data
        )
        self.publisher = self.create_publisher(Twist, '/cmd_vel', 10)

        # Timers
        self.info_timer = self.create_timer(1.0, self.log_info)
        self.stop_timer = self.create_timer(0.05, self.stop_watchdog)

        # Periodic cmd_vel publisher at 10 Hz (0.1 s)
        self.cmd_timer = self.create_timer(0.1, self.cmd_publish_timer_cb)

        self._state_action = "Idle"
        self._last_action_logged = None
        self._shutting_down = False

        # store start time as seconds
        self.start_time_s = self.get_clock().now().nanoseconds * 1e-9

        self.get_logger().info(
            "WallFollower (dominant-sector logic: all sectors act when dominant)."
        )

    #--------------------------------------------------------------------
    def stop_watchdog(self):
        """Stop the robot after time_to_stop seconds."""
        if self._shutting_down:
            return
        now = self.get_clock().now().nanoseconds * 1e-9
        if now - self.start_time_s >= self.time_to_stop:
            self.get_logger().info("Stopping due to timeout.")
            self.stop()

    #--------------------------------------------------------------------
    def stop(self):
        """Safe stop: set cmd to zero Twist, try to publish once, stop timers."""
        self._shutting_down = True

        # Set last command to zero
        self.cmd = Twist()

        # Try a final publish (publisher may still be valid even if shutdown started)
        try:
            self.publisher.publish(self.cmd)
        except Exception:
            # Context/publisher may already be invalid -> ignore
            pass

        # Cancel timers safely
        for t in [self.info_timer, self.stop_timer, self.cmd_timer]:
            try:
                t.cancel()
            except Exception:
                pass

    #--------------------------------------------------------------------
    def cmd_publish_timer_cb(self):
        """Periodic publisher: send the latest cmd_vel at 10 Hz."""
        if self._shutting_down:
            return

        try:
            self.publisher.publish(self.cmd)
        except Exception:
            # If the context or publisher is invalid, ignore
            pass

    #--------------------------------------------------------------------
    def laser_callback(self, scan):
        """Compute control action from LIDAR and update self.cmd using dominant-sector logic."""
        if self._shutting_down:
            return

        angle_min = math.degrees(scan.angle_min)
        angle_inc = math.degrees(scan.angle_increment)

        # Sectors: right side (negative angles), symmetric left (positive)
        FRONT       = []
        FR_RIGHT    = []
        RIGHT       = []
        BACK_RIGHT  = []
        BACK        = []

        FR_LEFT     = []
        LEFT        = []
        BACK_LEFT   = []

        for i, d in enumerate(scan.ranges):
            if not math.isfinite(d):
                continue
            if d < scan.range_min or d > scan.range_max:
                continue

            ang = angle_min + i * angle_inc  # degrees

            # right-side sectors (negative angles)
            if -20 <= ang <= 20:
                FRONT.append(d)
            elif -70 <= ang < -20:
                FR_RIGHT.append(d)
            elif -110 <= ang < -70:
                RIGHT.append(d)
            elif -160 <= ang < -110:
                BACK_RIGHT.append(d)
            elif ang <= -160 or ang >= 160:
                BACK.append(d)
            # left-side sectors (positive angles, symmetric)
            elif 20 < ang <= 70:
                FR_LEFT.append(d)
            elif 70 < ang <= 110:
                LEFT.append(d)
            elif 110 < ang < 160:
                BACK_LEFT.append(d)
            # any beam that doesn't fall in the above ranges is ignored

        # Compute minimal distances (inf if no reading)
        mins = {
            'front': min(FRONT) if FRONT else float('inf'),
            'fr_right': min(FR_RIGHT) if FR_RIGHT else float('inf'),
            'right': min(RIGHT) if RIGHT else float('inf'),
            'back_right': min(BACK_RIGHT) if BACK_RIGHT else float('inf'),
            'back': min(BACK) if BACK else float('inf'),
            'fr_left': min(FR_LEFT) if FR_LEFT else float('inf'),
            'left': min(LEFT) if LEFT else float('inf'),
            'back_left': min(BACK_LEFT) if BACK_LEFT else float('inf'),
        }

        # Debug log (uncomment for troubleshooting)
        # self.get_logger().debug(f"mins: {mins}")

        # Find sector with global minimum distance
        closest_sector, closest_dist = min(mins.items(), key=lambda kv: kv[1])

        twist = Twist()
        action = ""

        # Reset front_wall_type if front is no longer close
        if mins['front'] >= self.base_distance and self.front_wall_type is not None:
            self.front_wall_type = None

        # If nothing is visible at all -> stop
        if closest_dist == float('inf'):
            # no valid beams
            twist = Twist()
            action = "No beams -> stop"
        else:
            # Decision based on dominant (closest) sector
            # Now ALL sectors act when dominant and finite (math.isfinite(closest_dist))

            # FRONT: strafe left (vy+)
            if closest_sector == 'front' and math.isfinite(closest_dist):
                twist.linear.x = 0.0
                twist.linear.y = self.v_lin   # strafe left (positive vy)
                twist.angular.z = 0.0
                action = f"FRONT {closest_dist:.2f} m (dominant) → STRAFE LEFT"

            # FRONT-RIGHT: diagonal forward-left (vx +, vy +)
            elif closest_sector == 'fr_right' and math.isfinite(closest_dist):
                twist.linear.x = self.v_lin
                twist.linear.y = self.v_lin
                twist.angular.z = 0.0
                action = f"FRONT-RIGHT {closest_dist:.2f} m (dominant) → DIAGONAL FRONT-LEFT"

            # RIGHT: band control (dominant)
            elif closest_sector == 'right' and math.isfinite(closest_dist):
                error = closest_dist - self.base_distance
                if abs(error) <= self.tol:
                    twist.linear.x = self.v_lin
                    twist.linear.y = 0.0
                    twist.angular.z = 0.0
                    action = f"RIGHT ~OK ({closest_dist:.2f} m) → STRAIGHT"
                elif error < 0:
                    # too close
                    twist.linear.x = self.v_lin
                    twist.linear.y = self.v_lin
                    twist.angular.z = 0.0
                    action = f"RIGHT too CLOSE ({closest_dist:.2f} m) → forward + left"
                else:
                    # too far
                    twist.linear.x = self.v_lin
                    twist.linear.y = - self.v_lin
                    twist.angular.z = 0.0
                    action = f"RIGHT too FAR ({closest_dist:.2f} m) → forward + right strafe"

            # BACK-RIGHT: diagonal forward-right (vx +, vy -)
            elif closest_sector == 'back_right' and math.isfinite(closest_dist):
                twist.linear.x = self.v_lin
                twist.linear.y = -self.v_lin
                twist.angular.z = 0.0
                action = f"BACK-RIGHT {closest_dist:.2f} m (dominant) → DIAGONAL FRONT-RIGHT"

            # BACK: strafe right (vy -)
            elif closest_sector == 'back' and math.isfinite(closest_dist):
                twist.linear.x = 0.0
                twist.linear.y = - self.v_lin   # strafe right only
                twist.angular.z = 0.0
                action = f"BACK {closest_dist:.2f} m (dominant) → STRAFE RIGHT (recover)"

            # FRONT-LEFT: diagonal backward-right (vx - , vy +)
            elif closest_sector == 'fr_left' and math.isfinite(closest_dist):
                twist.linear.x = - self.v_lin
                twist.linear.y = self.v_lin
                twist.angular.z = 0.0
                action = f"FRONT-LEFT {closest_dist:.2f} m (dominant) → DIAGONAL BACK-RIGHT (vx - , vy +)"

            # LEFT: move backward (vx -)
            elif closest_sector == 'left' and math.isfinite(closest_dist):
                twist.linear.x = - self.v_lin
                twist.linear.y = 0.0
                twist.angular.z = 0.0
                action = f"LEFT {closest_dist:.2f} m (dominant) → MOVE BACKWARD (vx -)"

            # BACK-LEFT: diagonal back-left
            elif closest_sector == 'back_left' and math.isfinite(closest_dist):
                twist.linear.x = - self.v_lin
                twist.linear.y = - self.v_lin
                twist.angular.z = 0.0
                action = f"BACK-LEFT {closest_dist:.2f} m (dominant) → DIAGONAL BACK-LEFT"

            else:
                # should not happen: fallback stop
                twist = Twist()
                action = f"{closest_sector.upper()} {closest_dist:.2f} m dominant but not handled -> STOP"

        # Update last commanded twist (periodic timer will publish it)
        self.cmd = twist

        # Logging (only on change)
        if action != self._last_action_logged:
            self.get_logger().info(action)
            self._last_action_logged = action

        self._state_action = action

        self.prev_vx = twist.linear.x
        self.prev_vy = twist.linear.y

    #--------------------------------------------------------------------
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
