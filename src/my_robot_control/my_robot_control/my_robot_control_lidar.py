#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
import tf_transformations
from sensor_msgs.msg import LaserScan
import math   # NECESARIO para isfinite()

class RobotController(Node):
    def __init__(self):
        super().__init__('robot_control_lidar_node')
        
        # Initialize robot state
        self.robot_x = 0.0
        self.robot_y = 0.0
        self.robot_f = 0.0
        self.start_time = self.get_clock().now().seconds_nanoseconds()[0]
        self.closest_distance = 999.0
        self.last_print_time = 0
        
        # Subscribe to odometry topic
        self.create_subscription(Odometry, '/odom', self.odom_callback, 10)

        # Subscribe to LaserScan topic
        self.create_subscription(LaserScan,'/scan',self.listener_callback,10)

        # Create a publisher for velocity commands
        self.publisher = self.create_publisher(Twist, '/cmd_vel', 10)
        
        # Declare and get parameters
        self.declare_parameter('vx', 0.3)
        self.declare_parameter('vy', 0.0)
        self.declare_parameter('w', 0.0)
        self.declare_parameter('td', 3.0)
        
        self.vx = self.get_parameter('vx').value
        self.vy = self.get_parameter('vy').value
        self.w = self.get_parameter('w').value
        self.td = self.get_parameter('td').value
        
        # Start timer to control robot motion
        self.timer = self.create_timer(0.1, self.move_robot)

    def odom_callback(self, msg):
        """ Updates robot position from odometry """
        self.robot_x = msg.pose.pose.position.x
        self.robot_y = msg.pose.pose.position.y
        q = msg.pose.pose.orientation
        _, _, self.robot_f = tf_transformations.euler_from_quaternion([q.x, q.y, q.z, q.w])
    
    def listener_callback(self, scan):
        current_time = self.get_clock().now().seconds_nanoseconds()[0]
        if current_time - self.last_print_time < 1:
            return  # Skip printing if less than 1 second has passed

        angle_min_deg = scan.angle_min * 180.0 / 3.14159
        angle_max_deg = scan.angle_max * 180.0 / 3.14159
        angle_increment_deg = scan.angle_increment * 180.0 / 3.14159

        # Indices for specific angles in rUBot
        index_0_deg = int((0 - angle_min_deg)/ angle_increment_deg)
        index_neg90_deg = int((-90 - angle_min_deg) / angle_increment_deg)
        index_pos90_deg = int((90 - angle_min_deg) / angle_increment_deg)
        dist_0_deg = scan.ranges[index_0_deg]
        dist_neg90_deg = scan.ranges[index_neg90_deg]
        dist_pos90_deg = scan.ranges[index_pos90_deg]

        self.get_logger().info("---- LIDAR readings ----")
        self.get_logger().info(f"Number of scan points: {len(scan.ranges)}")
        self.get_logger().info(f"Distance at 0º: {dist_0_deg:.2f} m" if dist_0_deg else "No valid reading at 0°")
        self.get_logger().info(f"Distance at -90°: {dist_neg90_deg:.2f} m" if dist_neg90_deg else "No valid reading at -90°")
        self.get_logger().info(f"Distance at +90°: {dist_pos90_deg:.2f} m" if dist_pos90_deg else "No valid reading at +90°")
        self.get_logger().info(f"Distance at index 0: {scan.ranges[0]:.2f} m")
        self.get_logger().info(f"Distance at index 100: {scan.ranges[100]:.2f} m")
        self.get_logger().info(f"Distance at index 200: {scan.ranges[200]:.2f} m")
        self.get_logger().info(f"Distance at index 400: {scan.ranges[400]:.2f} m")
        self.get_logger().info(f"Distance at index 600: {scan.ranges[600]:.2f} m")
        self.get_logger().info(f"Distance at index 700: {scan.ranges[700]:.2f} m")

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
            else:
                continue

        if not custom_range:
            return
            
        closest_distance, angle_closest_distance = min(custom_range)
        self.closest_distance = closest_distance

        self.get_logger().info("---- LIDAR readings: Min distance ----")
        self.get_logger().info(f"Minimum distance: {closest_distance:.2f} m at angle {angle_closest_distance:.2f}°")

        self.last_print_time = current_time

    def move_robot(self):
        """ Publishes velocity commands until time limit is reached """
        elapsed_time = self.get_clock().now().seconds_nanoseconds()[0] - self.start_time
    
 
        if elapsed_time < self.td:
            self.get_logger().info(f'Robot moving... Time elapsed: {elapsed_time:.2f} sec')
            vel = Twist()
            vel.linear.x = self.vx
            vel.linear.y = self.vy
            vel.angular.z = self.w
            self.publisher.publish(vel)

            if self.closest_distance < 0.3:
                self.get_logger().warn('Obstacle detected! Stopping robot.')
                self.publisher.publish(Twist())  # Stop the robot
                self.timer.cancel()
                rclpy.try_shutdown()
            return
        else:
            self.get_logger().warn('Stopping robot')
            self.publisher.publish(Twist())  # Stop the robot
            self.timer.cancel()
            self.get_logger().info(f"Robot stopped")
            rclpy.try_shutdown()

def main():
    rclpy.init()
    rubot1_controller = RobotController()
    try:
        rclpy.spin(rubot1_controller)
    except KeyboardInterrupt:
        pass
    finally:
        rubot1_controller.destroy_node()

if __name__ == '__main__':
    main()



