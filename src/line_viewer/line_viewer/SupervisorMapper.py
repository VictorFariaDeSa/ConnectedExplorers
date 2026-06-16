import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from mobile_robot_interfaces.msg import SparseMappingData
from nav_msgs.msg import OccupancyGrid
import math
import numpy as np
import cv2
from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy

class SupervisorMapper(Node):
    def __init__(self):
        super().__init__('supervisor_mapper')

        latching_qos = QoSProfile(
                    depth=1,
                    reliability=ReliabilityPolicy.RELIABLE,
                    durability=DurabilityPolicy.TRANSIENT_LOCAL
                )

        # Grid parameters
        self.resolution = 0.05  # 5cm per cell
        self.width = 600        # 30 meters wide
        self.height = 600       # 30 meters high
        self.origin_x = -15.0   # Center the map at (0,0)
        self.origin_y = -15.0

        # FIX 1 & 3: Internal master grid MUST be floats and initialized to 0.0 (Unknown in log-odds)
        self.grid = np.zeros((self.height, self.width), dtype=np.float32)

        self.scan_sub = self.create_subscription(SparseMappingData, '/network_mapping_data', self.mapping_callback, 10)
        self.map_pub = self.create_publisher(OccupancyGrid, '/built_map', latching_qos)

    def world_to_grid(self, wx, wy):
        gx = int((wx - self.origin_x) / self.resolution)
        gy = int((wy - self.origin_y) / self.resolution)
        return gx, gy

    def bresenham_line(self, x0, y0, x1, y1):
        """Raytrace line algorithm to clear free space."""
        points = []
        dx = abs(x1 - x0)
        dy = abs(y1 - y0)
        x, y = x0, y0
        sx = -1 if x0 > x1 else 1
        sy = -1 if y0 > y1 else 1
        if dx > dy:
            err = dx / 2.0
            while x != x1:
                points.append((x, y))
                err -= dy
                if err < 0:
                    y += sy
                    err += dx
                x += sx
        else:
            err = dy / 2.0
            while y != y1:
                points.append((x, y))
                err -= dx
                if err < 0:
                    x += sx
                    err += dy
                y += sy
        points.append((x, y))
        return points

    def mapping_callback(self, msg):
        alpha = msg.alpha

        if alpha <= 0.01:
            return
        scan_msg = msg.scan
        pose = msg.pose

        rx = pose.position.x
        ry = pose.position.y

        q = pose.orientation
        ryaw = math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))

        rgx, rgy = self.world_to_grid(rx, ry)

        # --- Bayes Filter Constants (Log-Odds) ---
        l_occ = 0.85
        l_free = -0.4

        max_log_odds = 3.5
        min_log_odds = -3.5

        for i, range_val in enumerate(scan_msg.ranges):
            if range_val < scan_msg.range_min or range_val > scan_msg.range_max or math.isinf(range_val):
                continue

            angle = ryaw + scan_msg.angle_min + (i * scan_msg.angle_increment)
            hit_x = rx + range_val * math.cos(angle)
            hit_y = ry + range_val * math.sin(angle)
            hit_gx, hit_gy = self.world_to_grid(hit_x, hit_y)

            if 0 <= hit_gx < self.width and 0 <= hit_gy < self.height:

                # FIX 4: Slice the array [:-1] to exclude the final obstacle hit from the free-space ray
                free_cells = self.bresenham_line(rgx, rgy, hit_gx, hit_gy)[:-1]

                for (fx, fy) in free_cells:
                    if 0 <= fx < self.width and 0 <= fy < self.height:
                        self.grid[fy, fx] += (alpha * l_free)
                        self.grid[fy, fx] = max(min_log_odds, self.grid[fy, fx])

                # Update Obstacle (The Hit)
                self.grid[hit_gy, hit_gx] += (alpha * l_occ)
                self.grid[hit_gy, hit_gx] = min(max_log_odds, self.grid[hit_gy, hit_gx])

        self.publish_map()

    def publish_map(self):
            # FIX 2: Convert log-odds to probabilities (0.0 to 1.0)
            # We use np.exp with standard sigmoid math.
            prob_grid = 1.0 / (1.0 + np.exp(-self.grid))

            # Convert to ROS scale [0, 100]
            ros_map = (prob_grid * 100).astype(np.int8)

            # Re-apply the ROS "Unknown" state (-1) to cells that haven't been touched
            # Since initialized to exactly 0.0, untouched cells stay 0.0
            ros_map[self.grid == 0.0] = -1

            # 1. Prepare the ROS message
            msg = OccupancyGrid()
            msg.header.stamp = self.get_clock().now().to_msg()
            msg.header.frame_id = 'map'
            msg.info.resolution = self.resolution
            msg.info.width = self.width
            msg.info.height = self.height
            msg.info.origin.position.x = self.origin_x
            msg.info.origin.position.y = self.origin_y
            msg.data = ros_map.flatten().tolist()

            self.map_pub.publish(msg)

            # # 2. SAVE PNG FOR DEBUGGING
            # # Make a visual copy directly from our calculated ros_map
            # img = np.full((self.height, self.width), 127, dtype=np.uint8) # Default Gray
            # img[ros_map >= 65] = 0    # Obstacles -> Black
            # img[ros_map < 35] = 255   # Free Space -> White
            # img[ros_map == -1] = 127  # Unknown -> Gray

            # # Flip vertically because OccupancyGrid origin is bottom-left, but image origin is top-left
            # img = cv2.flip(img, 0)

            # cv2.imwrite('map_debug.png', img)

def main(args=None):
    rclpy.init(args=args)
    node = SupervisorMapper()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
