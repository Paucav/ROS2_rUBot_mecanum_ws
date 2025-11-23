import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
import math

class RobotStopOnObstacle(Node):

    def __init__(self):
        super().__init__('robot_stop_on_obstacle')

        # Paràmetres configurables
        self.declare_parameter('distance_limit', 0.3)
        self.declare_parameter('forward_speed', 0.2)

        self._distanceLimit = self.get_parameter('distance_limit').value
        self._forwardSpeed = self.get_parameter('forward_speed').value

        # Preparar missatge de velocitat
        self._msg = Twist()
        self._msg.linear.x = self._forwardSpeed
        self._msg.angular.z = 0.0

        # Publisher i subscripció
        self._cmdVel = self.create_publisher(Twist, '/cmd_vel', 10)

        # Timer per publicar velocitat contínuament
        self.timer = self.create_timer(0.05, self.timer_callback)

        self.subscription = self.create_subscription(
            LaserScan,
            '/scan',
            self.laser_callback,
            10
        )

        self.get_logger().info("Node STOP-ON-OBSTACLE inicialitzat.")

    def timer_callback(self):
        # Publica la comanda actual de moviment (linear.x, angular.z)
        self._cmdVel.publish(self._msg)

    def laser_callback(self, scan):
        # Busquem la distància mínima en el rang
        valid_distances = [
            d for d in scan.ranges
            if math.isfinite(d) and scan.range_min < d < scan.range_max
        ]

        if not valid_distances:
            return

        closest = min(valid_distances)

        # DEBUG opcional
        self.get_logger().info(f"Distància mínima: {closest:.2f} m")

        if closest < self._distanceLimit:
            # ATURAR robot
            self._msg.linear.x = 0.0
            self._msg.angular.z = 0.0
        else:
            # Manté moviment endavant
            self._msg.linear.x = self._forwardSpeed
            self._msg.angular.z = 0.0

def main(args=None):
    rclpy.init(args=args)
    node = RobotStopOnObstacle()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
