import numpy as np
import rclpy
from nav_msgs.msg import OccupancyGrid
from rclpy.node import Node

from fake_occupancy.noise import biased_easing, perlin


class OccupancyGridPublisher(Node):
    """
    ROS2 node for generating and publishing occupancy grids.
    """

    def __init__(self):
        super().__init__('occupancy_grid_publisher')

        self.publisher_ = self.create_publisher(
            OccupancyGrid, 'occupancy_grid', 1)
        self.get_logger().info("Occupancy grid publisher node started")
        self.frame_number = 0
        timer_period = 0.5  # seconds
        self.timer = self.create_timer(
            timer_period, self.publish_occupancy_grid)
        self.i = 0

    def generate_grid(self, size_y=100, size_x=100, scale=2, time=0, seed=0) -> np.ndarray:
        """
        Generate an occupancy grid using Perlin noise.
        """
        lin_x = np.linspace(0, 5, size_x, endpoint=False)
        lin_y = np.linspace(0, 5, size_y, endpoint=False)
        x, y = np.meshgrid(lin_y, lin_x)
        noise = perlin(x / scale, y / scale + time, seed)
        noise = (noise - noise.min()) / (noise.max() - noise.min() + 1e-6)
        noise = biased_easing(noise, 5)
        return (noise * 100).astype(np.int8)

    def publish_occupancy_grid(self):
        """
        Publish the generated occupancy grid as a ROS2 OccupancyGrid message.
        """
        grid = self.generate_grid(time=self.frame_number * 0.05)
        occupancy_msg = OccupancyGrid()

        # Populate the OccupancyGrid message
        occupancy_msg.header.stamp = self.get_clock().now().to_msg()
        occupancy_msg.header.frame_id = 'map'
        occupancy_msg.info.resolution = 0.1  # 10cm per cell
        occupancy_msg.info.width = grid.shape[1]
        occupancy_msg.info.height = grid.shape[0]
        occupancy_msg.info.origin.position.x = 0.0
        occupancy_msg.info.origin.position.y = 0.0
        occupancy_msg.info.origin.position.z = 0.0
        occupancy_msg.info.origin.orientation.w = 1.0


        occupancy_msg.data = grid.flatten().tolist()

        # Publish the message
        self.publisher_.publish(occupancy_msg)
        self.get_logger().info(
            f"Published occupancy grid frame {self.frame_number}")
        self.frame_number += 1


def main(args=None):
    """
    Entry point for the ROS2 node.
    """
    rclpy.init(args=args)
    node = OccupancyGridPublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
