import heapq
from typing import List, Optional, Tuple

import numpy as np
import rclpy
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import OccupancyGrid, Path
from rclpy.node import Node


class DijkstraPathPlanner(Node):
    """
    ROS2 node for generating and publishing the shortest path using Dijkstra's algorithm.
    """

    def __init__(self, grid: List[List[int]], start: Tuple[int, int], end: Tuple[int, int]):
        super().__init__('dijkstra_path_planner')
        self.grid = grid
        self.start = start
        self.end = end
        self.rows = len(grid)
        self.cols = len(grid[0])
        self.directions = [(-1, 0), (1, 0), (0, -1), (0, 1)
                           ]  # Up, Down, Left, Right

        # Publisher setup for ROS2
        self.publisher_ = self.create_publisher(
            Path, 'Path', 10)

        self.get_logger().info("Dijkstra path planner node started")

        self.subscription = self.create_subscription(
            OccupancyGrid, 'occupancy_grid', self.grid_callback, 10)

    def grid_callback(self, msg: OccupancyGrid):
        """Callback function to process the occupancy grid."""
        width = msg.info.width
        height = msg.info.height
        self.rows = height
        self.cols = width

        # Process occupancy grid into a 2D list
        self.grid = [
            [1 if msg.data[i * width + j] >= 50 else 0 for j in range(width)]
            for i in range(height)
        ]

        self.get_logger().info("Occupancy grid processed and updated.")
        path = self.dijkstra(self.start, self.end)
        if path:
            self.path_callback(path)
        else:
            self.get_logger().error("No path found from start to end.")


        # Set up threshold-based binary conversion
        threshold = 50  # threshold of 50% probability for obstacles
        binary_grid = []
        for row in grid:
            binary_row = []
            for cell in row:
                binary_row.append(1 if cell >= threshold else 0)
            binary_grid.append(binary_row)

        if self.grid is not None:
            self.get_logger().info(f"received grid of {self.grid}")
            path = self.dijkstra(self.start, self.end)
            if path:
                self.path_callback(path)
            else:
                self.get_logger().error("No path found from start to end")

    def is_valid(self, row: int, col: int) -> bool:
        """Check if the cell is within bounds and not an obstacle."""
        return 0 <= row < self.rows and 0 <= col < self.cols and self.grid[row][col] == 0

    def dijkstra(self, start: Tuple[int, int], end: Tuple[int, int]) -> Optional[List[Tuple[int, int]]]:
        """Find the shortest path from start to end using Dijkstra's algorithm."""
        priority_queue = [(0, start)]  # (cost, position)
        costs = {start: 0}
        parents = {start: None}
        visited = set()

        while priority_queue:
            current_cost, current_position = heapq.heappop(priority_queue)

            if current_position in visited:
                continue
            visited.add(current_position)

            # If we reached the end, reconstruct the path
            if current_position == end:
                return self.reconstruct_path(parents, end)

            row, col = current_position
            for dr, dc in self.directions:
                neighbor = (row + dr, col + dc)
                if self.is_valid(neighbor[0], neighbor[1]):
                    new_cost = current_cost + 1  # Each step costs 1
                    if neighbor not in costs or new_cost < costs[neighbor]:
                        costs[neighbor] = new_cost
                        heapq.heappush(priority_queue, (new_cost, neighbor))
                        parents[neighbor] = current_position

        return None  # No path found

    def reconstruct_path(self, parents, end):
        """Reconstruct the path from start to end."""
        path = []
        current = end
        while current is not None:
            path.append(current)
            current = parents[current]
        path.reverse()
        return path

    def path_callback(self, waypoints: List[Tuple[int, int]]):
        """Publish the path as a ROS2 Path message."""
        path_msg = self.path_from_waypoints(waypoints)
        self.publisher_.publish(path_msg)
        self.get_logger().info(
            f"Published path with {len(path_msg.poses)} waypoints")

def path_from_waypoints(self, waypoints: List[Tuple[int, int]]) -> Path:
    """Convert a list of waypoints into a ROS2 Path message."""
    path_msg = Path()
    path_msg.header.stamp = self.get_clock().now().to_msg()
    path_msg.header.frame_id = "map"

    for row, col in waypoints:
        pose_stamped = PoseStamped()
        pose_stamped.header.stamp = path_msg.header.stamp
        pose_stamped.header.frame_id = "map"
        pose_stamped.pose.position.x = float(row)
        pose_stamped.pose.position.y = float(col)
        pose_stamped.pose.position.z = 0.0
        path_msg.poses.append(pose_stamped)

    self.get_logger().info(f"Generated path message with {len(path_msg.poses)} waypoints.")
    return path_msg





def main(args=None):
    rclpy.init(args=args)

    # Define start and end coordinates for the path
    start = (0, 0)
    end = (7, 4)

    # Create the DijkstraPathPlanner node
    node = DijkstraPathPlanner(None, start, end)  # Start with no initial grid

    # Spin to process incoming messages
    rclpy.spin(node)

    # Clean up
    node.destroy_node()
    rclpy.shutdown()



if __name__ == "__main__":
    main()
