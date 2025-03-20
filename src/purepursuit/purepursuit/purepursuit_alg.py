import heapq
import math
from typing import List, Optional, Tuple

import rclpy
from geometry_msgs.msg import PoseStamped, Twist, Point, Pose
from nav_msgs.msg import Path
from rclpy.node import Node
#import tf2_ros
#import tf_transformations
#import tf2_geometry_msgs
#from tf_transformations import euler_from_quaternion
from transforms3d.euler import quat2euler

class PurePursuitController(Node):
    """Pure Pursuit Algorithm integrated with Pose and Twist data"""

    def __init__(self):
        super().__init__('pure_pursuit_controller')

        # Subscriptions
        self.subscription_pose = self.create_subscription(
            PoseStamped, '/robot_pose', self.pose_callback, 10
        )
        self.subscription_twist = self.create_subscription(
            Twist, '/robot_twist', self.twist_callback, 10
        )

        self.publisher_ = self.create_publisher(Path, '/steering_angle', 10)
        self.get_logger().info("Pure pursuit node started")

        # Parameters
        self.lookahead_distance = 2.0  # Lookahead distance in meters
        self.wheelbase = 2.5  # Distance between front and rear axles
        self.current_path = None
        self.current_position = (0, 0)  # Initial position (x, y)
        self.current_heading = 0  # Initial heading (yaw)
        self.current_twist = None  # Initial twist (velocity)

        self.timer = self.create_timer(0.1, self.update_control)  # Control loop

    def pose_callback(self, pose_msg):
        """Callback to store the latest Pose (position and orientation)"""
        self.current_position = (pose_msg.pose.position.x, pose_msg.pose.position.y)

        # Convert quaternion orientation to Euler angles (roll, pitch, yaw)
        quat = pose_msg.pose.orientation
        _, _, self.current_heading = self.quaternion_to_euler(quat)

        self.get_logger().info(f"Received pose: {self.current_position}, heading: {self.current_heading}")

    def twist_callback(self, twist_msg):
        """Callback to store the latest Twist (velocity)"""
        self.current_twist = twist_msg
        self.get_logger().info(f"Received twist: {self.current_twist.linear.x}, {self.current_twist.angular.z}")

    def quaternion_to_euler(self, quat):
        """Convert quaternion to Euler angles (roll, pitch, yaw) using transforms3d"""
        # Note: transforms3d uses [w, x, y, z] format for quaternions
        euler = quat2euler([quat.w, quat.x, quat.y, quat.z], axes='sxyz')
        return euler
    
    def path_callback(self, path_msg):
        """Store the latest path"""
        self.current_path = path_msg.poses
        self.get_logger().info(
            f"Received new path with {len(self.current_path)} waypoints"
        )

    def find_lookahead_point(self, current_position):
        """Find the first point in the path at least `lookahead_distance` away"""
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
        """Calculate the steering angle to the lookahead point"""
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
        """Update control loop, factoring Pose and Twist into steering calculation"""
        if self.current_position and self.current_twist:
            # Use the current position and heading from Pose
            steering_angle = self.calculate_steering_angle(self.current_position, self.current_heading)

            # Create and publish the Twist message for velocity control
            cmd_msg = Twist()
            cmd_msg.angular.z = steering_angle  # Steering angle
            cmd_msg.linear.x = self.current_twist.linear.x  # Use the current linear velocity from Twist
            self.publisher_.publish(cmd_msg)

            # Log the steering angle and twist
            self.get_logger().info(
                f"Published steering command with angle: {steering_angle:.2f} radians, speed: {cmd_msg.linear.x:.2f}"
            )


def main(args=None):
    """Main function to start the ROS 2 node"""
    rclpy.init(args=args)
    controller = PurePursuitController()
    rclpy.spin(controller)
    controller.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
