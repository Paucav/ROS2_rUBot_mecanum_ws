import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
import math


def normalize_angle_deg(angle_deg):
    """Normalitza un angle a l'interval [-180, 180)."""
    return (angle_deg + 180.0) % 360.0 - 180.0


class LidarTest(Node):

    def __init__(self):
        super().__init__('lidar_test_rubot_node')

        self.subscription = self.create_subscription(
            LaserScan,
            '/scan',
            self.listener_callback,
            10
        )
        self.last_print_time = self.get_clock().now().seconds_nanoseconds()[0]

    def listener_callback(self, scan: LaserScan):
        current_time = self.get_clock().now().seconds_nanoseconds()[0]
        if current_time - self.last_print_time < 1:
            return  # imprimeix com a màxim cada segon

        # Converteix a graus amb més precisió
        angle_min_deg = math.degrees(scan.angle_min)
        angle_increment_deg = math.degrees(scan.angle_increment)
        n_samples = len(scan.ranges)

        # Calcula la llista d'angles corresponents a cada index i normalitza a [-180,180)
        angles_deg = [normalize_angle_deg(angle_min_deg + i * angle_increment_deg) for i in range(n_samples)]

        # Funció auxiliar: trobar l'índex amb l'angle més proper a target_deg
        def find_closest_index(target_deg):
            best_i = None
            best_diff = float('inf')
            for i, a in enumerate(angles_deg):
                diff = abs(normalize_angle_deg(a - target_deg))
                if diff < best_diff:
                    best_diff = diff
                    best_i = i
            return best_i

        # Índexs buscats (això és robust passi el que passi amb angle_min)
        idx_0 = find_closest_index(0.0)
        idx_pos90 = find_closest_index(90.0)
        idx_neg90 = find_closest_index(-90.0)

        # Funció per obtenir distància segura
        def safe_distance_at(idx):
            if idx is None or idx < 0 or idx >= n_samples:
                return float('nan')
            d = scan.ranges[idx]
            if not math.isfinite(d):
                return float('nan')
            if d <= 0.0:
                return float('nan')
            if d < scan.range_min or d > scan.range_max:
                return float('nan')
            return d

        dist_0_deg = safe_distance_at(idx_0)
        dist_pos90_deg = safe_distance_at(idx_pos90)
        dist_neg90_deg = safe_distance_at(idx_neg90)

        self.get_logger().info("---- LIDAR readings ----")
        if math.isfinite(dist_0_deg):
            self.get_logger().info(f"Distance at 0° (index {idx_0}, angle {angles_deg[idx_0]:.2f}°): {dist_0_deg:.2f} m")
        else:
            self.get_logger().info(f"No valid reading at 0° (closest index {idx_0}, angle {angles_deg[idx_0]:.2f}°)" if idx_0 is not None else "No valid index for 0°")

        if math.isfinite(dist_pos90_deg):
            self.get_logger().info(f"Distance at +90° (index {idx_pos90}, angle {angles_deg[idx_pos90]:.2f}°): {dist_pos90_deg:.2f} m")
        else:
            self.get_logger().info(f"No valid reading at +90° (closest index {idx_pos90}, angle {angles_deg[idx_pos90]:.2f}°)" if idx_pos90 is not None else "No valid index for +90°")

        if math.isfinite(dist_neg90_deg):
            self.get_logger().info(f"Distance at -90° (index {idx_neg90}, angle {angles_deg[idx_neg90]:.2f}°): {dist_neg90_deg:.2f} m")
        else:
            self.get_logger().info(f"No valid reading at -90° (closest index {idx_neg90}, angle {angles_deg[idx_neg90]:.2f}°)" if idx_neg90 is not None else "No valid index for -90°")

        # Troba la distància mínima entre valors vàlids i obté el seu angle
        valid_entries = []
        for i, d in enumerate(scan.ranges):
            if not math.isfinite(d):
                continue
            if d <= 0.0:
                continue
            if d < scan.range_min or d > scan.range_max:
                continue
            # opcional: filtrar per sector si vols (ara busquem globalment)
            valid_entries.append((d, i))

        if not valid_entries:
            self.get_logger().info("No valid LIDAR readings to compute minimum.")
            self.last_print_time = current_time
            return

        closest_distance, idx_closest = min(valid_entries, key=lambda x: x[0])
        angle_closest = angles_deg[idx_closest]

        self.get_logger().info("---- LIDAR readings: Min distance ----")
        self.get_logger().info(f"Minimum distance: {closest_distance:.2f} m at index {idx_closest} angle {angle_closest:.2f}°")

        self.last_print_time = current_time


def main(args=None):
    rclpy.init(args=args)
    lidar1_test = LidarTest()
    rclpy.spin(lidar1_test)
    lidar1_test.destroy_node()
    rclpy.shutdown()
