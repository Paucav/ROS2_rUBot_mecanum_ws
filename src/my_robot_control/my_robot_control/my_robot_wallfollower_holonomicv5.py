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

        # Front-recovery state: when we detect a frontal obstacle we first
        # take one small lateral step away from the right wall, then rotate
        # left until we detect the right wall again.
        self._front_recovery_active = False
        self._front_recovery_step_done = False

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
            "WallFollower (RIGHT tol, BACK_RIGHT when closest) - differential drive. Modified to stay parallel to right wall."
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
    def _normalize_angle_deg(self, ang_deg):
        """Normalize angle to [-180, 180]."""
        a = ang_deg
        while a > 180:
            a -= 360
        while a <= -180:
            a += 360
        return a

    #--------------------------------------------------------------------
    def laser_callback(self, scan):
        """Compute control action from LIDAR and update self.cmd.

        Behavior changes made:
        - Robot should always travel parallel to the right wall (linear.x > 0 when right wall present).
        - When a front obstacle appears: perform a small lateral move away from the right wall (linear.y > 0) ONCE,
          then stop forward motion and rotate left until the right wall is seen again; then continue straight.
        - When the right wall "ends": detect via BACK_RIGHT and perform a diagonal forward+right (vx+, vy-) until
          the back-right reading indicates the wall is now the relevant measurement.
        """
        if self._shutting_down:
            return

        angle_min = math.degrees(scan.angle_min)
        angle_inc = math.degrees(scan.angle_increment)

        FRONT = []
        FR_RIGHT = []
        RIGHT = []
        BACK_RIGHT = []
        BACK = []

        for i, d in enumerate(scan.ranges):
            if not math.isfinite(d):
                continue
            if d < scan.range_min or d > scan.range_max:
                continue

            ang = self._normalize_angle_deg(angle_min + i * angle_inc)

            # Sectors (degrees): FRONT [-20,20], FRONT-RIGHT (-70,-20], RIGHT (-110,-70],
            # BACK-RIGHT (-160,-110], BACK (>160 or <=-160)
            if -20 <= ang <= 20:
                FRONT.append(d)
            elif -70 <= ang < -20:
                FR_RIGHT.append(d)
            elif -110 <= ang < -70:
                RIGHT.append(d)
            elif -160 <= ang < -110:
                BACK_RIGHT.append(d)
            elif ang >= 160 or ang <= -160:
                BACK.append(d)

        # Minimal distances
        min_front = min(FRONT) if FRONT else float('inf')
        min_fr_right = min(FR_RIGHT) if FR_RIGHT else float('inf')
        min_right = min(RIGHT) if RIGHT else float('inf')
        min_back_right = min(BACK_RIGHT) if BACK_RIGHT else float('inf')
        min_back = min(BACK) if BACK else float('inf')

        twist = Twist()
        action = ""

        # If front obstacle disappears, reset front recovery state
        if min_front >= self.base_distance and self._front_recovery_active:
            self._front_recovery_active = False
            self._front_recovery_step_done = False
            self.front_wall_type = None

        #----------------------------------------------------------
        # RULE 1: FRONT obstacle → perform recovery: small lateral step, then rotate left until right wall appears
        #----------------------------------------------------------
        if min_front < self.base_distance:
            # Start recovery if not already
            if not self._front_recovery_active:
                self._front_recovery_active = True
                self._front_recovery_step_done = False
                # classify wall orientation once
                if self.front_wall_type is None:
                    if abs(self.prev_vx) > abs(self.prev_vy):
                        self.front_wall_type = "horizontal"
                    else:
                        self.front_wall_type = "vertical"

            # First: a single small lateral move away from the right wall to create space
            if not self._front_recovery_step_done:
                # Move laterally + in y (to the left) a bit but do NOT advance in x
                twist.linear.x = 0.0
                twist.linear.y = self.v_lin * 0.3
                twist.angular.z = 0.0
                self._front_recovery_step_done = True
                action = (
                    f"FRONT {min_front:.2f} m → small LATERAL step left to separate from right wall"
                )
            else:
                # Then stop forward motion and rotate left until the right wall becomes visible again
                twist.linear.x = 0.0
                twist.linear.y = 0.0
                # rotate left in place
                twist.angular.z = self.v_ang
                action = (
                    f"FRONT {min_front:.2f} m → ROTATING LEFT until right wall detected"
                )

                # If during rotation we detect the right wall, finish recovery and go straight
                if math.isfinite(min_right):
                    # Finish recovery: move forward parallel to right wall
                    twist.linear.x = self.v_lin
                    twist.linear.y = 0.0
                    twist.angular.z = 0.0
                    self._front_recovery_active = False
                    self._front_recovery_step_done = False
                    action = (
                        f"Recovered from FRONT: right wall at {min_right:.2f} m → STRAIGHT"
                    )

        #----------------------------------------------------------
        # RULE 2: FRONT-RIGHT obstacle (close diagonal front-right) -> gentle diagonal away
        # This is kept but tuned for the case where robot normally moves forward (vx>0)
        #----------------------------------------------------------
        elif min_fr_right < self.base_distance:
            # gentle diagonal forward-left to avoid the corner
            twist.linear.x = self.v_lin * 0.6
            twist.linear.y = self.v_lin * 0.3
            twist.angular.z = 0.0
            action = f"FRONT-RIGHT {min_fr_right:.2f} m → gentle DIAGONAL forward-left"

        #----------------------------------------------------------
        # RULE 3: RIGHT visible → go straight (we want to stay parallel to right wall)
        #----------------------------------------------------------
        elif math.isfinite(min_right):
            error = min_right - self.base_distance

            if abs(error) <= self.tol:
                # Inside band: go straight forward (parallel)
                twist.linear.x = self.v_lin
                twist.linear.y = 0.0
                twist.angular.z = 0.0
                action = (
                    f"RIGHT ~OK ({min_right:.2f} m) → STRAIGHT parallel to wall"
                )

            elif error < 0:
                # Too close to right wall → back off slightly from the wall while continuing forward
                twist.linear.x = self.v_lin * 0.5
                twist.linear.y = self.v_lin * 0.2  # small left component
                twist.angular.z = 0.0
                action = (
                    f"RIGHT too CLOSE ({min_right:.2f} m) → forward + slight LEFT to increase gap"
                )

            else:
                # Too far from right wall → move slightly right while moving forward to get closer
                twist.linear.x = self.v_lin * 0.5
                twist.linear.y = -self.v_lin * 0.2
                twist.angular.z = 0.0
                action = (
                    f"RIGHT too FAR ({min_right:.2f} m) → forward + slight RIGHT to reduce gap"
                )

        #----------------------------------------------------------
        # RULE 4: RIGHT not visible but BACK-RIGHT visible -> this can indicate the right wall ends ahead;
        # do a diagonal forward-right (vx+, vy-) to search for the continuation of the wall.
        # Continue this until a RIGHT reading becomes available or back-right stops being the most relevant.
        #----------------------------------------------------------
        elif math.isfinite(min_back_right) and (
            not math.isfinite(min_right) or min_back_right <= min_right
        ):
            # Move forward and slightly to the right to follow a wall that continues behind/on the right
            twist.linear.x = self.v_lin
            twist.linear.y = -self.v_lin * 0.5
            twist.angular.z = 0.0
            action = (
                f"BACK-RIGHT {min_back_right:.2f} m (no RIGHT) → DIAGONAL forward-right searching for wall"
            )

            # If while doing this we detect RIGHT, then normal following will resume next cycle

        # otherwise: nothing visible -> stop

        # Update last commanded twist (periodic timer will publish it)
        self.cmd = twist

        # Logging (only on change)
        if action != self._last_action_logged:
            self.get_logger().info(action if action else "No action (stopped).")
            self._last_action_logged = action

        self._state_action = action if action else "Stopped (no wall detected)"

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
