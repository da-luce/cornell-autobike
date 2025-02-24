import math
from unittest.mock import MagicMock

import pytest
import rclpy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Path
from purepursuit.purepursuit import PurePursuitController  # Import the controller
from rclpy.duration import Duration
from rclpy.node import Node


@pytest.fixture
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
        # Example path (PoseStamped objects with x, y coordinates)
        {"pose": {"position": {"x": 0, "y": 0, "z": 0}}},
        {"pose": {"position": {"x": 1, "y": 1, "z": 0}}},
        {"pose": {"position": {"x": 2, "y": 2, "z": 0}}},
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
        {"pose": {"position": {"x": 0, "y": 0}}},
        {"pose": {"position": {"x": 1, "y": 1}}},
        {"pose": {"position": {"x": 2, "y": 2}}},
        {"pose": {"position": {"x": 3, "y": 3}}},
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
        {"pose": {"position": {"x": 0, "y": 0}}},
        {"pose": {"position": {"x": 2, "y": 2}}},
    ]

    # Robot at position (0, 0) with heading of 0 radians (facing east)
    steering_angle = controller.calculate_steering_angle((0, 0), 0)

    # The robot should steer toward the point (2, 2)
    # This is a simple test and we expect the angle to be non-zero
    expected_angle = math.atan2(2, 2)  # Should be 45 degrees (π/4 radians)
    assert math.isclose(steering_angle, expected_angle, abs_tol=0.1)


def test_update_control(node_and_controller):
    """Test the update control loop."""
    node, controller = node_and_controller

    # Mock the publisher to test if the messages are being published
    mock_publisher = MagicMock()
    controller.cmd_publisher = mock_publisher

    # Set up a mock path
    controller.current_path = [
        {"pose": {"position": {"x": 0, "y": 0}}},
        {"pose": {"position": {"x": 2, "y": 2}}},
    ]

    # Mock robot's position and heading
    controller.find_lookahead_point = MagicMock(return_value=(2, 2))
    controller.calculate_steering_angle = MagicMock(return_value=math.pi / 4)

    # Run the control loop once
    controller.update_control()

    # Check if a Twist message was published with the correct steering angle
    cmd_msg = mock_publisher.publish.call_args[0][0]
    assert isinstance(cmd_msg, Twist)
    assert math.isclose(cmd_msg.angular.z, math.pi / 4, abs_tol=0.1)
    assert cmd_msg.linear.x == 1.0  # Constant speed


@pytest.mark.parametrize(
    "robot_position, expected_angle",
    [
        ((0, 0), math.pi / 4),
        ((1, 1), math.atan2(1, 1)),
        ((2, 2), math.atan2(2, 2)),
    ],
)
def test_steering_angle_for_different_positions(
    node_and_controller, robot_position, expected_angle
):
    """Test the steering angle for various robot positions."""
    node, controller = node_and_controller

    # Define a path with some points
    controller.current_path = [
        {"pose": {"position": {"x": 0, "y": 0}}},
        {"pose": {"position": {"x": 2, "y": 2}}},
        {"pose": {"position": {"x": 4, "y": 4}}},
    ]

    # Calculate the steering angle for each test case
    steering_angle = controller.calculate_steering_angle(
        robot_position, 0
    )  # Heading is 0

    # Assert that the calculated angle is close to the expected angle
    assert math.isclose(steering_angle, expected_angle, abs_tol=0.1)


if __name__ == '__main__':
    pytest.main()
