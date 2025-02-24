import heapq
import math
from typing import List, Optional, Tuple

import rclpy
from geometry_msgs.msg import PoseStamped, Twist
from nav_msgs.msg import Path
from rclpy.node import Node

#to connect with the path planning algo, + ros2
class PurePursuitController(Node):
    """pure pursuit algo to be connected with the path planning algo"""
    def __init__(self):
        super().__init__('pure_pursuit_controller')
        self.subscription = self.create_subscription(
            Path, '/path', self.path_callback, 10
        )
        self.publisher_ = self.create_publisher(Path, '/steering_angle', 10)
        self.get_logger().info("pure pursuit node started")
       # self.cmd_publisher = self.create_publisher(Twist, '/cmd_vel', 10)

        #temp settings
        self.lookahead_distance = 2.0  # Lookahead distance in meters
        self.wheelbase = 2.5  # Distance between front and rear axles
        self.current_path = None
        self.timer = self.create_timer(0.1, self.update_control)  # Control loop

    def path_callback(self, path_msg):
        """Store the latest path"""
        self.current_path = path_msg.poses
        self.get_logger().info(
            f"Received new path with {len(self.current_path)} waypoints"
        )

    def find_lookahead_point(self, current_position):
        """Find the first point in the path at least `lookahead_distance` away """
        if not self.current_path:
            return None
        for pose in self.current_path:
            distance = math.sqrt(
                (pose.pose.position.x - current_position[0]) ** 2
                + (pose.pose.position.y - current_position[1]) ** 2
            )
            if distance >= self.lookahead_distance:
                return (pose.pose.position.x, pose.pose.position.y)
        return None

    def calculate_steering_angle(self, current_position, heading):
        """Find lookahead point"""
        lookahead_point = self.find_lookahead_point(current_position)
        if lookahead_point is None:
            return 0.0  # No lookahead point; stop steering

        dx = lookahead_point[0] - current_position[0]
        dy = lookahead_point[1] - current_position[1]
        angle_to_point = math.atan2(dy, dx) - heading
        steering_angle = math.atan2(
            2 * self.wheelbase * math.sin(angle_to_point), self.lookahead_distance
        )
        return steering_angle

    def update_control(self):
        """Initialize vehicle state"""
        current_position = (0, 0)
        heading = 0

        if self.current_path is not None:
            # Calculate the steering angle
            steering_angle = self.calculate_steering_angle(current_position, heading)
            cmd_msg = Twist()
            cmd_msg.angular.z = steering_angle
            cmd_msg.linear.x = 1.0  # Constant speed
            self.cmd_publisher.publish(cmd_msg)

            # Log the steering angle
            self.get_logger().info(
                f"Published steering command with angle: {steering_angle:.2f} radians"
            )


def main(args=None):
    """main
    -subscribes to a planned path via ros, finds point on path lookahead dist away,
    finds angle required, publishes steering commands (Twist)
    """
    rclpy.init(args=args)
    controller = PurePursuitController()
    rclpy.spin(controller)
    controller.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
