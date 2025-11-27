import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
import math


class RobotSelfControl(Node):

    def __init__(self):
        super().__init__('robot_selfcontrol_node')

        # Configurable parameters
        self.declare_parameter('distance_limit', 0.3)
        self.declare_parameter('speed_factor', 1.0)
        self.declare_parameter('forward_speed', 0.2)
        self.declare_parameter('rotation_speed', 0.3)
        self.declare_parameter('time_to_stop', 5.0)

        self._distanceLimit = self.get_parameter('distance_limit').value
        self._speedFactor = self.get_parameter('speed_factor').value
        self._forwardSpeed = self.get_parameter('forward_speed').value
        self._rotationSpeed = self.get_parameter('rotation_speed').value
        self._time_to_stop = self.get_parameter('time_to_stop').value

        self._msg = Twist()
        # Moviment inicial: diagonal cap a la dreta -> x positiu, y negatiu
        self._msg.linear.x = self._forwardSpeed * self._speedFactor
        self._msg.linear.y = -self._forwardSpeed * self._speedFactor  
        self._msg.angular.z = 0.0

        self._cmdVel = self.create_publisher(Twist, '/cmd_vel', 10)
        self.timer = self.create_timer(0.05, self.timer_callback)

        self.subscription = self.create_subscription(
            LaserScan,
            '/scan',
            self.laser_callback,
            10  # Default QoS depth
        )
        self.start_time = self.get_clock().now().nanoseconds * 1e-9
        self._shutting_down = False
        self._last_info_time = self.start_time
        self._last_speed_time = self.start_time

    def timer_callback(self):
        if self._shutting_down:
            return
        now_sec = self.get_clock().now().nanoseconds * 1e-9
        elapsed_time = now_sec - self.start_time

        self._cmdVel.publish(self._msg)

        if now_sec - self._last_speed_time >= 1:
            self.get_logger().info(
                f"Vx: {self._msg.linear.x:.2f} m/s, "
                f"Vy: {self._msg.linear.y:.2f} m/s, "
                f"w: {self._msg.angular.z:.2f} rad/s | Time: {elapsed_time:.1f}s"
            )
            self._last_speed_time = now_sec

        if elapsed_time >= self._time_to_stop:
            self.stop()
            self.timer.cancel()
            self.get_logger().info("Robot stopped")
            rclpy.try_shutdown()

    def laser_callback(self, scan):
        if self._shutting_down:
            return

        angle_min_deg = scan.angle_min * 180.0 / 3.14159
        angle_increment_deg = scan.angle_increment * 180.0 / 3.14159

        # Filter valid readings within [-150°, 150°]
        custom_range = []
        for i, distance in enumerate(scan.ranges):
            angle_robot_deg = angle_min_deg + i * angle_increment_deg
            if angle_robot_deg > 180.0:
                angle_robot_deg -= 360.0
            if not math.isfinite(distance) or distance <= 0.0:
                continue
            if distance < scan.range_min or distance > scan.range_max:
                continue
            if -150 < angle_robot_deg < 150:
                custom_range.append((distance, angle_robot_deg))

        if not custom_range:
            return

        closest_distance, angle_closest_distance = min(custom_range)

        # Determine zone
        if -45 <= angle_closest_distance <= 45:
            zone = "FRONT"
        elif 45 < angle_closest_distance <= 110:
            zone = "LEFT"
        elif -110 <= angle_closest_distance < -45:
            zone = "RIGHT"
        elif 110 < angle_closest_distance <= 150:
            zone = "BACK_LEFT"
        elif -150 <= angle_closest_distance < -110:
            zone = "BACK_RIGHT"
        else:
            zone = "BACK"

        now = self.get_clock().now().nanoseconds * 1e-9
        if now - self._last_info_time >= 1:
            self.get_logger().info(
                f"[DETECTION] Zone: {zone} | "
                f"Distance: {closest_distance:.2f} m | "
                f"Angle: {angle_closest_distance:.0f}°"
            )
            self._last_info_time = now

        # React to obstacle: only add logs
        if closest_distance < self._distanceLimit:

            old_x = self._msg.linear.x
            old_y = self._msg.linear.y

            self._msg.angular.z = 0.0

            if zone == "FRONT":
                self._msg.linear.x = -self._msg.linear.x
                action = "Invert X (FRONT)"
            elif zone == "BACK_LEFT" or zone == "BACK_RIGHT":
                self._msg.linear.x = -self._msg.linear.x
                action = "Invert X (BACK)"
            elif zone == "LEFT":
                self._msg.linear.y = -self._msg.linear.y
                action = "Invert Y (LEFT)"
            elif zone == "RIGHT":
                self._msg.linear.y = -self._msg.linear.y
                action = "Invert Y (RIGHT)"
            elif zone == "BACK":
                action = "Keep X (BACK)"
            else:
                action = "None"

            # Keep magnitudes
            self._msg.linear.x = math.copysign(self._forwardSpeed * self._speedFactor, self._msg.linear.x)
            self._msg.linear.y = math.copysign(self._forwardSpeed * self._speedFactor, self._msg.linear.y)

            # LOGGER DE LA ACCIÓN
            self.get_logger().info(
                f"[ACTION] {action} | "
                f"X: {old_x:.2f} → {self._msg.linear.x:.2f} | "
                f"Y: {old_y:.2f} → {self._msg.linear.y:.2f}"
            )

        else:
            # No obstacle -> maintain diagonal
            if self._msg.linear.x == 0.0:
                self._msg.linear.x = self._forwardSpeed * self._speedFactor
            if self._msg.linear.y == 0.0:
                self._msg.linear.y = -self._forwardSpeed * self._speedFactor

            self._msg.linear.x = math.copysign(self._forwardSpeed * self._speedFactor, self._msg.linear.x)
            self._msg.linear.y = math.copysign(self._forwardSpeed * self._speedFactor, self._msg.linear.y)

    def stop(self):
        self._shutting_down = True
        stop_msg = Twist()
        stop_msg.linear.x = 0.0
        stop_msg.linear.y = 0.0
        stop_msg.angular.z = 0.0
        self._cmdVel.publish(stop_msg)
        rclpy.spin_once(self, timeout_sec=0.1)


def main(args=None):
    rclpy.init(args=args)
    robot = RobotSelfControl()
    try:
        rclpy.spin(robot)
    except KeyboardInterrupt:
        pass
    finally:
        robot.destroy_node()


if __name__ == '__main__':
    main()
