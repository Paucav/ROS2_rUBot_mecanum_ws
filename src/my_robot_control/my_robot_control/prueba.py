import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
import math

class RobotSelfControl(Node):
    def __init__(self):
        super().__init__('robot_selfcontrol_node')

        # Parámetros
        self.declare_parameter('distance_limit', 0.3)
        self.declare_parameter('forward_speed', 0.2)
        self.declare_parameter('rotation_speed', 0.5)

        self._distanceLimit = self.get_parameter('distance_limit').value
        self._forwardSpeed = self.get_parameter('forward_speed').value
        self._rotationSpeed = self.get_parameter('rotation_speed').value

        self._msg = Twist()
        self._msg.linear.x = self._forwardSpeed
        self._msg.angular.z = 0.0

        self._cmdVel = self.create_publisher(Twist, '/cmd_vel', 10)
        self.timer = self.create_timer(0.05, self.timer_callback)
        self.subscription = self.create_subscription(LaserScan, '/scan', self.laser_callback, 10)

        # --- Estados del robot ---
        self.state = "MOVIENDO"
        self._turn_start_time = 0.0
        self._turn_duration = math.radians(45) / self._rotationSpeed  # 45° en segundos

    def timer_callback(self):
        now = self.get_clock().now().nanoseconds * 1e-9

        # Control del giro
        if self.state == "GIRANDO":
            if now - self._turn_start_time >= self._turn_duration:
                # Giro terminado
                self.state = "MOVIENDO"
                self._msg.angular.z = 0.0
                self._msg.linear.x = self._forwardSpeed
            self._cmdVel.publish(self._msg)
            return

        self._cmdVel.publish(self._msg)

    def laser_callback(self, scan):
        if self.state != "MOVIENDO":
            return  # Ignora láser mientras gira

        # Detectar obstáculo más cercano frontal
        angle_min_deg = scan.angle_min * 180.0 / math.pi
        angle_increment_deg = scan.angle_increment * 180.0 / math.pi

        custom_range = []
        for i, distance in enumerate(scan.ranges):
            angle_deg = angle_min_deg + i * angle_increment_deg
            if -45 <= angle_deg <= 45 and math.isfinite(distance):
                custom_range.append(distance)

        if not custom_range:
            return

        closest = min(custom_range)

        if closest < self._distanceLimit:
            # Obstáculo detectado → iniciar giro 45°
            self.state = "GIRANDO"
            self._turn_start_time = self.get_clock().now().nanoseconds * 1e-9
            self._msg.linear.x = 0.0
            self._msg.angular.z = self._rotationSpeed

    def stop(self):
        self._msg.linear.x = 0.0
        self._msg.angular.z = 0.0
        self._cmdVel.publish(self._msg)

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
