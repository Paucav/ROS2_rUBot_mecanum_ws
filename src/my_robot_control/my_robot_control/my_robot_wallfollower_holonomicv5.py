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
            "WallFollower (RIGHT tol). BACK now triggers when min_back < min_right."
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
        BACK        = []

        FR_LEFT     = []
        LEFT        = []
        BACK_LEFT   = []

        for i, d in enumerate(scan.ranges):
            if not math.isfinite(d):
                continue
            if d < scan.range_min or d > scan.range_max:
                continue

            ang = angle_min + i * angle_inc

            # Define angular sectors (degrees)
            # FRONT: -20 .. 20
            # FR_RIGHT: -70 .. -20
            # RIGHT: -110 .. -70
            # BACK_RIGHT: -160 .. -110
            # BACK: ang <= -160 or ang >= 160  (covers the rear)
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
            
            # LEFT sectors (simètrics)
            elif 20 < ang <= 70:
                FR_LEFT.append(d)
            elif 70 < ang <= 110:
                LEFT.append(d)
            elif 110 < ang < 160:
                BACK_LEFT.append(d)

        # Minimal distances
        min_front      = min(FRONT)      if FRONT      else float('inf')
        min_fr_right   = min(FR_RIGHT)   if FR_RIGHT   else float('inf')
        min_right      = min(RIGHT)      if RIGHT      else float('inf')
        min_back_right = min(BACK_RIGHT) if BACK_RIGHT else float('inf')
        min_back       = min(BACK) if BACK else float('inf')

        min_fr_left    = min(FR_LEFT)    if FR_LEFT    else float('inf')
        min_left       = min(LEFT)       if LEFT       else float('inf')
        min_back_left  = min(BACK_LEFT)  if BACK_LEFT  else float('inf')

        twist = Twist()
        action = ""
        # Si ja no hi ha obstáculo davant, resetejem el tipus de paret
        if min_front >= self.base_distance and self.front_wall_type is not None:
            self.front_wall_type = None

        #----------------------------------------------------------
        # RULE 1: FRONT obstacle → strafe left (recover)
        #----------------------------------------------------------
        if min_front < self.base_distance and min_front < min_left:
            twist.linear.x = 0.0
            twist.linear.y = self.v_lin   # strafe left (positive vy)
            twist.angular.z = 0.0
            action = f"FRONT {min_front:.2f} m → STRAFE LEFT (obstacle)"

        #----------------------------------------------------------
        # RULE 2: FRONT-RIGHT obstacle → slow + left diagonal forward-left
        #----------------------------------------------------------
        elif min_fr_right < self.base_distance and min_fr_right < min_left:
            twist.linear.x = self.v_lin
            twist.linear.y = self.v_lin
            twist.angular.z = 0.0
            action = f"FRONT-RIGHT {min_fr_right:.2f} m → DIAGONAL FRONT-LEFT"

        #----------------------------------------------------------
        # RULE 3: RIGHT visible → control with tolerance band (no vy) BUT
        # only if RIGHT is more relevant than BACK_RIGHT
        #----------------------------------------------------------
        elif (
            math.isfinite(min_right)
            and (not math.isfinite(min_back_right) or min_right < min_back_right)
            and (not math.isfinite(min_back) or min_right < min_back)
            and min_right < min_left
        ):

            # error > 0 → too far; error < 0 → too close
            error = min_right - self.base_distance

            if abs(error) <= self.tol:
                # Inside band: go straight
                twist.linear.x = self.v_lin
                twist.linear.y = 0.0
                twist.angular.z = 0.0
                action = (
                    f"RIGHT ~OK ({min_right:.2f} m, target "
                    f"{self.base_distance:.2f}±{self.tol:.2f}) → STRAIGHT"
                )

            elif error < 0:
                # Too close to right wall → slow forward + stronger left turn
                twist.linear.x = self.v_lin 
                twist.linear.y = self.v_lin 
                twist.angular.z = 0.0
                action = (
                    f"RIGHT too CLOSE ({min_right:.2f} m < "
                    f"{self.base_distance:.2f}-{self.tol:.2f}) → "
                    f"forward + strong LEFT turn"
                )

            else:
                # Too far from right wall → slow forward + stronger right strafe
                twist.linear.x = self.v_lin 
                twist.linear.y = - self.v_lin 
                twist.angular.z = 0.0
                action = (
                    f"RIGHT too FAR ({min_right:.2f} m > "
                    f"{self.base_distance:.2f}+{self.tol:.2f}) → "
                    f"forward + strong RIGHT strafe"
                )

        #----------------------------------------------------------
        # RULE 4: BACK-RIGHT → diagonal forward-right (recover)
        # Use when BACK_RIGHT is the most relevant right-side reading
        #----------------------------------------------------------
        elif (math.isfinite(min_back_right) 
            and (not math.isfinite(min_right) or min_back_right <= min_right)
            and min_back_right < min_left
        ):
            twist.linear.x = self.v_lin
            twist.linear.y = -self.v_lin
            twist.angular.z = 0.0
            action = (
                f"BACK-RIGHT {min_back_right:.2f} m → DIAGONAL FRONT-RIGHT (recover)"
            )

        #----------------------------------------------------------
        # RULE 5: BACK visible → move only vy- (strafe right) to recover wall
        # Now triggers only when rear reading is closer than right reading:
        # min_back < min_right
        #----------------------------------------------------------
        elif math.isfinite(min_back) and (min_back < min_right):
            twist.linear.x = 0.0
            twist.linear.y = - self.v_lin   # strafe right only
            twist.angular.z = 0.0
            action = (
                f"BACK {min_back:.2f} m (< min_right {min_right if math.isfinite(min_right) else 'inf'}) → STRAFE RIGHT (recover from back)"
            )
        
        #----------------------------------------------------------
        # RULE L1: LEFT → move backward (vx -)
        # Only if RIGHT is not relevant
        #----------------------------------------------------------
        elif math.isfinite(min_left) and min_left < self.base_distance and min_left < min_front:
            twist.linear.x = -self.v_lin
            twist.linear.y = 0.0
            twist.angular.z = 0.0
            action = f"LEFT {min_left:.2f} m → MOVE BACKWARD (vx -)"
        
        #----------------------------------------------------------
        # RULE L2: FR_LEFT → diagonal backward-right (vx - and vy +)
        # Triggered when LEFT is gone but FR_LEFT detects wall
        # Only if RIGHT is not relevant
        #----------------------------------------------------------
        elif (math.isfinite(min_fr_left) 
            #and (min_fr_left < self.base_distance) 
            and (not math.isfinite(min_right) or min_fr_left < min_right)
            
        ):
            twist.linear.x = -self.v_lin
            twist.linear.y = self.v_lin
            twist.angular.z = 0.0
            action = f"FRONT-LEFT {min_fr_left:.2f} m → DIAGONAL BACK-RIGHT (vx - , vy +)"

        # if nothing is visible, twist remains zero -> robot stops

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
