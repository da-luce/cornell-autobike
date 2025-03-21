import numpy as np
import rclpy
from nav_msgs.msg import OccupancyGrid
from rclpy.node import Node


class OccupancyGridPublisher(Node):
    def __init__(self):
        super().__init__('occupancy_grid_publisher')
        self.publisher_ = self.create_publisher(
            OccupancyGrid, 'fake_occupancy_grid', 10)
        timer_period = 1.0  # seconds
        self.timer = self.create_timer(
            timer_period, self.publish_fake_occupancy_grid)

    def publish_fake_occupancy_grid(self):
        grid = OccupancyGrid()
        grid.header.stamp = self.get_clock().now().to_msg()
        grid.header.frame_id = "map"

        # Define the grid size (e.g., 10x10)
        grid.info.width = 10
        grid.info.height = 10
        grid.info.resolution = 1.0

        # Fake data generation
        fake_data = np.random.randint(0, 100, size=(
            grid.info.width * grid.info.height)).tolist()
        grid.data = fake_data

        self.publisher_.publish(grid)
        self.get_logger().info('Publishing fake occupancy grid.')


def main(args=None):
    rclpy.init(args=args)
    node = OccupancyGridPublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
