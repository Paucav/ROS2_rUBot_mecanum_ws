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
        # Moviment inicial: diagonal cap endavant-dreta
        self._msg.linear.x = self._forwardSpeed * self._speedFactor  # endavant
        self._msg.linear.y = -self._forwardSpeed * self._speedFactor  # dreta (left positive -> right negative)
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

        # Reté l'última zona que va provocar un canvi de direcció
        self._last_zone_triggered = None

    def timer_callback(self):
        if self._shutting_down:
            return
        now_sec = self.get_clock().now().nanoseconds * 1e-9
        elapsed_time = now_sec - self.start_time

        self._cmdVel.publish(self._msg)

        if now_sec - self._last_speed_time >= 1:
            self.get_logger().info(
                f"Vx: {self._msg.linear.x:.2f} m/s, Vy: {self._msg.linear.y:.2f} m/s, w: {self._msg.angular.z:.2f} rad/s | Time: {elapsed_time:.1f}s"
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

        # Filter valid readings within [-180°, 180°]
        custom_range = []
        for i, distance in enumerate(scan.ranges):
            # Angle on robot
            angle_robot_deg = angle_min_deg + i * angle_increment_deg
            # Normalize to (-180,180]
            if angle_robot_deg > 180.0:
                angle_robot_deg -= 360.0
            if angle_robot_deg <= -180.0:
                angle_robot_deg += 360.0

            if not math.isfinite(distance) or distance <= 0.0:
                continue
            if distance < scan.range_min or distance > scan.range_max:
                continue
            # Accept full 360 degrees (-180..180)
            if -180.0 <= angle_robot_deg <= 180.0:
                custom_range.append((distance, angle_robot_deg))

        if not custom_range:
            return
        closest_distance, angle_closest_distance = min(custom_range)

        # Determine 4 zones: FRONT, BACK, LEFT, RIGHT
        # ROS convention: 0 deg = front, +left, -right
        if -45 <= angle_closest_distance <= 45:
            zone = "FRONT"
        elif 45 < angle_closest_distance <= 135:
            zone = "LEFT"
        elif -135 <= angle_closest_distance < -45:
            zone = "RIGHT"
        else:
            # Covers angles >135 or <-135 -> BACK
            zone = "BACK"

        now = self.get_clock().now().nanoseconds * 1e-9
        if now - self._last_info_time >= 1:
            self.get_logger().info(
                f"[DETECTION] Distance: {closest_distance:.2f} m | Angle: {angle_closest_distance:.0f}° | Zone: {zone}"
            )
            self._last_info_time = now

        # React to obstacle: només si està dins del límit i en una ZONA diferent de l'última que va provocar canvi
        if closest_distance < self._distanceLimit:
            # Si la zona és la mateixa que la última que ja va canviar la direcció, no fem res
            if zone == self._last_zone_triggered:
                return

            # Apliquem la regla de rebot (sense rotacions)
            # FRONT o BACK -> invertir component X (preservar Y)
            # LEFT o RIGHT -> invertir component Y (preservar X)
            if zone == "FRONT" or zone == "BACK":
                self._msg.linear.x = -self._msg.linear.x
                # Assegurem magnitud correcte segons paràmetres (preservant signe)
                self._msg.linear.x = math.copysign(self._forwardSpeed * self._speedFactor, self._msg.linear.x)
                # mantenim linear.y tal qual
                self._msg.linear.y = math.copysign(self._forwardSpeed * self._speedFactor, self._msg.linear.y)
            elif zone == "LEFT" or zone == "RIGHT":
                self._msg.linear.y = -self._msg.linear.y
                # Assegurem magnitud correcte segons paràmetres (preservant signe)
                self._msg.linear.y = math.copysign(self._forwardSpeed * self._speedFactor, self._msg.linear.y)
                # mantenim linear.x tal qual
                self._msg.linear.x = math.copysign(self._forwardSpeed * self._speedFactor, self._msg.linear.x)

            # sempre sense rotació
            self._msg.angular.z = 0.0

            # marquem la zona que ha provocat aquest canvi
            self._last_zone_triggered = zone

            self.get_logger().info(
                f"[BOUNCE] Zone: {zone} | New Vx: {self._msg.linear.x:.2f}, Vy: {self._msg.linear.y:.2f}"
            )

        else:
            pass
            # No obstacle dins del límit -> mantenim moviment diagonal amb magnitud correcta
            #self._msg.angular.z = 0.0
            # Si alguna component està a zero (no hauria), assignem la direcció per defecte preservant signes
            #if self._msg.linear.x == 0.0:
                #self._msg.linear.x = self._forwardSpeed * self._speedFactor
            #if self._msg.linear.y == 0.0:
                #self._msg.linear.y = -self._forwardSpeed * self._speedFactor
            # Ajustem magnitud si no hem canviat (no alterem last_zone_triggered)
            #self._msg.linear.x = math.copysign(self._forwardSpeed * self._speedFactor, self._msg.linear.x)
            #self._msg.linear.y = math.copysign(self._forwardSpeed * self._speedFactor, self._msg.linear.y)

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
