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
        self.declare_parameter('tolerance', 0.03)        # band around base_distance (RIGHT)

        # New parameters for lateral correction (existing change you already have)
        self.declare_parameter('lat_gain', 0.9)          # gain for linear.y correction (Vy = -lat_gain * error)
        self.declare_parameter('max_lateral', 0.15)     # max absolute lateral speed (m/s)

        # NEW (minimal) parameter to help Regla 4 stick to the wall:
        self.declare_parameter('back_lateral', 0.08)    # small lateral speed (m/s) applied in Regla 4 (negative y)

        self.base_distance = float(self.get_parameter('distance_limit').value)
        self.v_lin = float(self.get_parameter('forward_speed').value)
        self.v_ang = float(self.get_parameter('turn_speed').value)
        self.time_to_stop = float(self.get_parameter('time_to_stop').value)
        self.tol = float(self.get_parameter('tolerance').value)

        self.lat_gain = float(self.get_parameter('lat_gain').value)
        self.max_lateral = float(self.get_parameter('max_lateral').value)
        self.back_lateral = float(self.get_parameter('back_lateral').value)

        # Last commanded twist (will be published periodically)
        self.cmd = Twist()

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

        self.start_time_s = self.get_clock().now().nanoseconds * 1e-9

        self.get_logger().info(
            "WallFollower (RIGHT tol, BACK_RIGHT when closest) - differential drive."
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
        """Compute control action from LIDAR and update self.cmd."""
        if self._shutting_down:
            return

        angle_min = math.degrees(scan.angle_min)
        angle_inc = math.degrees(scan.angle_increment)

        FRONT       = []
        FR_RIGHT    = []
        RIGHT       = []
        BACK_RIGHT  = []

        for i, d in enumerate(scan.ranges):
            if not math.isfinite(d):
                continue
            if d < scan.range_min or d > scan.range_max:
                continue

            ang = angle_min + i * angle_inc

            if -20 <= ang <= 20:
                FRONT.append(d)
            elif -70 <= ang < -20:
                FR_RIGHT.append(d)
            elif -110 <= ang < -70:
                RIGHT.append(d)
            elif -160 <= ang < -110:
                BACK_RIGHT.append(d)

        # Minimal distances
        min_front      = min(FRONT)      if FRONT      else float('inf')
        min_fr_right   = min(FR_RIGHT)   if FR_RIGHT   else float('inf')
        min_right      = min(RIGHT)      if RIGHT      else float('inf')
        min_back_right = min(BACK_RIGHT) if BACK_RIGHT else float('inf')

        twist = Twist()
        action = ""

        #----------------------------------------------------------
        # RULE 1: FRONT obstacle → turn left
        #----------------------------------------------------------
        if min_front < self.base_distance:
            twist.linear.x = 0.0
            twist.linear.y = 0.0
            twist.angular.z = self.v_ang * 2.0
            action = f"FRONT {min_front:.2f} m → turn LEFT"

        #----------------------------------------------------------
        # RULE 2: FRONT-RIGHT obstacle → slow + left
        #----------------------------------------------------------
        elif min_fr_right < self.base_distance:
            twist.linear.x = 0.0
            twist.linear.y = 0.0
            twist.angular.z = self.v_ang * 2.0
            action = f"FRONT-RIGHT {min_fr_right:.2f} m → turn LEFT"

        #----------------------------------------------------------
        # RULE 3: RIGHT visible → control with tolerance band (use linear.y)
        #----------------------------------------------------------
        elif math.isfinite(min_right):
            # error > 0 → too far; error < 0 → too close
            error = min_right - self.base_distance

            # Compute lateral correction vy = -lat_gain * error (ROS convention: +y = left)
            vy = - self.lat_gain * error

            # Clamp lateral speed
            if vy > self.max_lateral:
                vy = self.max_lateral
            elif vy < -self.max_lateral:
                vy = -self.max_lateral

            if abs(error) <= self.tol:
                # Inside band: go straight but apply a small lateral correction to stay parallel
                twist.linear.x = self.v_lin
                twist.linear.y = vy
                twist.angular.z = 0.0
                action = (
                    f"RIGHT ~OK ({min_right:.2f} m) → STRAIGHT + Vy={twist.linear.y:.3f}"
                )

            elif error < 0:
                # Too close to right wall → slow forward + left lateral move to increase distance
                twist.linear.x = self.v_lin * 0.5
                twist.linear.y = vy  # vy will be positive (move left) because error<0
                twist.angular.z = self.v_ang * 2.0
                action = (
                    f"RIGHT too CLOSE ({min_right:.2f} m) → forward + LEFT lateral ({twist.linear.y:.3f}) + strong LEFT turn"
                )

            else:
                # Too far from right wall → slow forward + right lateral move to decrease distance
                twist.linear.x = self.v_lin * 0.5
                twist.linear.y = vy  # vy will be negative (move right) because error>0
                twist.angular.z = -self.v_ang * 2.0
                action = (
                    f"RIGHT too FAR ({min_right:.2f} m) → forward + RIGHT lateral ({twist.linear.y:.3f}) + strong RIGHT turn"
                )

        #----------------------------------------------------------
        # RULE 4: BACK-RIGHT → only if it is the most relevant wall
        #          Added small negative Vy to stick closer to the wall while turning.
        #----------------------------------------------------------
        elif math.isfinite(min_back_right) and (
            not math.isfinite(min_right) or min_back_right <= min_right
        ):
            twist.linear.x = self.v_lin * 0.1
            # Apply a small lateral movement to the RIGHT (negative y) so the robot stays closer to the wall
            twist.linear.y = -abs(self.back_lateral)
            twist.angular.z = -2.0 * self.v_ang
            action = (
                f"BACK-RIGHT {min_back_right:.2f} m → "
                f"very slow + STRONG RIGHT turn (2*w) + Vy={twist.linear.y:.3f}"
            )

        # if nothing is visible, twist remains zero -> robot stops

        # Update last commanded twist (periodic timer will publish it)
        self.cmd = twist

        # Logging (only on change)
        if action != self._last_action_logged:
            self.get_logger().info(action if action else "No action (stopped).")
            self._last_action_logged = action

        self._state_action = action if action else "Stopped (no wall detected)"

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
