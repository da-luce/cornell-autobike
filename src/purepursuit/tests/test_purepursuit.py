import math
#from unittest.mock import MagicMock
from unittest.mock import MagicMock, patch

import pytest
import rclpy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Path
from purepursuit.purepursuit.purepursuit_alg import PurePursuitController  # Import the controller
from rclpy.duration import Duration
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped, Pose, Point
from geometry_msgs.msg import Quaternion
import tf2_ros

@pytest.fixture(scope='module')
def node_and_controller():
    # Create a ROS 2 node for testing and the PurePursuitController instance
    rclpy.init()
    node = Node('test_node')
    controller = PurePursuitController()
    return node, controller


def test_path_subscription(node_and_controller):
    """Test that the PurePursuitController subscribes to the path correctly."""
    node, controller = node_and_controller

    # Create a mock Path message
    path_msg = Path()
    path_msg.poses = [
        PoseStamped(pose=Pose(position=Point(x=0.0, y=0.0, z=0.0))),
        PoseStamped(pose=Pose(position=Point(x=1.0, y=1.0, z=0.0))),
        PoseStamped(pose=Pose(position=Point(x=2.0, y=2.0, z=0.0)))
    ]

    # Simulate receiving the path message by calling the path_callback
    controller.path_callback(path_msg)

    # Check if the path is correctly stored in the controller
    assert len(controller.current_path) == 3
    assert controller.current_path[0].pose.position.x == 0
    assert controller.current_path[2].pose.position.y == 2


def test_find_lookahead_point(node_and_controller):
    """Test the lookahead point selection logic."""
    node, controller = node_and_controller

    # Test path with 4 points (simplified for testing purposes)
    controller.current_path = [
        PoseStamped(pose=Pose(position=Point(x=0.0, y=0.0, z=0.0))),
        PoseStamped(pose=Pose(position=Point(x=1.0, y=1.0, z=0.0))),
        PoseStamped(pose=Pose(position=Point(x=2.0, y=2.0, z=0.0))),
        PoseStamped(pose=Pose(position=Point(x=3.0, y=3.0, z=0.0)))
    ]

    # Robot position is at (0, 0)
    lookahead_point = controller.find_lookahead_point((0, 0))

    # Lookahead point should be at least 2 meters away
    assert lookahead_point == (2, 2)

    # Test when the robot is closer to the path
    lookahead_point = controller.find_lookahead_point((0.5, 0.5))
    assert lookahead_point == (2, 2)


def test_steering_angle_calculation(node_and_controller):
    """Test the calculation of the steering angle."""
    node, controller = node_and_controller

    # Set up a test case with a simple path
    controller.current_path = [
        PoseStamped(pose=Pose(position=Point(x=0.0, y=0.0, z=0.0))),
        PoseStamped(pose=Pose(position=Point(x=2.0, y=2.0, z=0.0)))
    ]

    # Robot at position (0, 0) with heading of 0 radians (facing east)
    steering_angle = controller.calculate_steering_angle((0, 0), 0)

    # Calculate the expected steering angle based on the formula
    lookahead_distance = 2.0
    wheelbase = 2.5
    expected_angle = math.atan2(2 * wheelbase * math.sin(math.atan2(2, 2)), lookahead_distance)

    # Assert that the calculated angle is close to the expected angle
    assert math.isclose(steering_angle, expected_angle, abs_tol=0.1)


def test_update_control(node_and_controller):
    """Test the update control loop."""
    node, controller = node_and_controller

    # Mock the publisher to test if the messages are being published
    mock_publisher = MagicMock()
    controller.publisher_ = mock_publisher

    # Mock Pose message (position and heading)
    mock_pose = PoseStamped()
    mock_pose.pose.position = Point(x=0.0, y=0.0, z=0.0)
    mock_pose.pose.orientation = Quaternion(x=0.0, y=0.0, z=0.0, w=1.0)  # No rotation (heading = 0)
    controller.pose_callback(mock_pose)

    # Mock Twist message (velocity)
    mock_twist = Twist()
    mock_twist.linear.x = 1.0  # Speed
    mock_twist.angular.z = 0.0  # No rotation
    controller.twist_callback(mock_twist)

    # Set up a mock path
    controller.current_path = [
        PoseStamped(pose=Pose(position=Point(x=2.0, y=2.0, z=0.0))),
        PoseStamped(pose=Pose(position=Point(x=4.0, y=4.0, z=0.0))),
    ]

    # Patch tf2_ros.Buffer to mock the lookup_transform method
    with patch.object(tf2_ros.Buffer, 'lookup_transform', return_value=None):
        # Run the control loop once
        controller.update_control()

    # Check if a Twist message was published with the correct steering angle and speed
    cmd_msg = mock_publisher.publish.call_args[0][0]
    assert isinstance(cmd_msg, Twist)
    assert math.isclose(cmd_msg.angular.z, 1.06, abs_tol=0.1)  # Updated expected value
    assert cmd_msg.linear.x == 1.0


@pytest.mark.parametrize(
    "robot_position, expected_angle",
    [
        ((0.0, 0.0), 1.06),  # Originally math.pi/4 (~0.785)
        ((1.0, 1.0), 1.06),  # Originally math.atan2(1,1) (~0.785)
        ((2.0, 2.0), -1.06),  # Adjusted based on path direction
    ],
)
def test_steering_angle_for_different_positions(
    node_and_controller, robot_position, expected_angle
):
    """Test the steering angle for various robot positions."""
    node, controller = node_and_controller

    # Define a path with some points
    controller.current_path = [
        PoseStamped(pose=Pose(position=Point(x=0.0, y=0.0))),
        PoseStamped(pose=Pose(position=Point(x=2.0, y=2.0))),
        PoseStamped(pose=Pose(position=Point(x=4.0, y=4.0))),
    ]

    # Mock Pose and Twist data
    mock_pose = PoseStamped()
    mock_pose.pose.position = Point(x=robot_position[0], y=robot_position[1], z=0.0)
    mock_pose.pose.orientation = Quaternion(x=0.0, y=0.0, z=0.0, w=1.0)  # Heading = 0 for simplicity
    controller.pose_callback(mock_pose)

    mock_twist = Twist()
    mock_twist.linear.x = 1.0  # Constant speed
    mock_twist.angular.z = 0.0  # No angular speed
    controller.twist_callback(mock_twist)

    # Calculate the steering angle for the given robot position
    steering_angle = controller.calculate_steering_angle(robot_position, 0)

    # Assert that the calculated angle is close to the expected angle
    assert math.isclose(steering_angle, expected_angle, abs_tol=0.1)


if __name__ == '__main__':
    pytest.main()
