
"""Unit tests for the DijkstraPathPlanner class."""

import pytest
import rclpy

from nav_msgs.msg import Path
from djikstra.alg import DijkstraPathPlanner  


@pytest.fixture(scope='module')
def node():
    """Create a DijkstraPathPlanner node"""
    rclpy.init()

    # Example occupancy grid: 0 = free space, 1 = obstacle
    grid = [
        [0, 0, 0, 0, 0],
        [0, 1, 1, 0, 0],
        [0, 0, 0, 1, 0],
        [0, 1, 0, 0, 0],
        [0, 0, 0, 0, 0]
    ]
    start = (0, 0)  # Start at top-left corner
    end = (4, 4)    # End at bottom-right corner

    # Create the DijkstraPathPlanner node
    dijkstra_node = DijkstraPathPlanner(grid, start, end)
    yield dijkstra_node

    rclpy.shutdown()


def test_is_valid(node):
    """Test if is_valid method correctly identifies valid and invalid grid cells."""
    
    # Test valid cell
    assert node.is_valid(0, 0) is True, "The cell (0, 0) should be valid."

    # Test obstacle cell
    assert node.is_valid(1, 1) is False, "The cell (1, 1) should be an obstacle."

    # Test cell outside the grid
    assert node.is_valid(5, 5) is False, "The cell (5, 5) should be outside the grid."


def test_dijkstra_path_exists(node):
    """Test if Dijkstra's algorithm finds a path in a grid with no obstacles."""
    
    path = node.dijkstra(node.start, node.end)
    assert path is not None, "A path should be there from start to end."
    assert len(path) > 1, "Path should have more than one waypoint."
    assert all(
        isinstance(coord, tuple) and len(coord) == 2 for coord in path
    ), "Each waypoint should be a tuple of (row, column)."


def test_publish_path(node):
    """Test if the publish_path method correctly publishes a ROS2 Path message."""
    
    # Simulate waypoints
    waypoints = [(0, 0), (0, 1), (0, 2), (1, 2), (2, 2)]

    # Call the method and validate the path
    path = node.path_from_waypoints(waypoints)
    assert isinstance(path, Path), "Result should be a ROS2 Path message."
    assert len(path.poses) == len(waypoints), "Path should have same number of poses as waypoints."
    assert path.header.frame_id == "map", "Frame ID should be 'map'."
    assert path.header.stamp is not None, "Timestamp should be set."


def test_empty_path(node):
    """Test if path_from_waypoints handles an empty list of waypoints."""
    
    waypoints = []
    path = node.path_from_waypoints(waypoints)
    assert isinstance(path, Path), "Result should be a ROS2 Path message."
    assert len(path.poses) == 0, "Path should contain zero poses when there are no waypoints."



def test_reconstruct_path(node):
    """Test if the reconstruct_path method correctly reconstructs a path."""
    
    # Simulate a simple path from (0, 0) to (2, 2) with parent references
    parents = {
        (2, 2): (1, 2),
        (1, 2): (0, 2),
        (0, 2): (0, 1),
        (0, 1): (0, 0),
        (0, 0): None
    }
    path = node.reconstruct_path(parents, (2, 2))
    
    expected_path = [(0, 0), (0, 1), (0, 2), (1, 2), (2, 2)]
    assert path == expected_path, "Peconstructed path does not match the expected path."


def test_publish_path(node):
    """Test if the publish_path method correctly publishes a ROS2 Path message."""
    
    # Simulate waypoints
    waypoints = [(0, 0), (0, 1), (0, 2), (1, 2), (2, 2)]

    path = node.path_from_waypoints(waypoints)
    assert isinstance(path, Path), "Result should be a ROS2 Path message."
    assert len(path.poses) == len(waypoints), "Path should have same number of poses as waypoints."
    assert path.header.frame_id == "map", "Frame ID should be 'map'."
    assert path.header.stamp is not None, "Timestamp should be set."


def test_empty_path(node):
    """Test if publish_path handles an empty list of waypoints."""
    
    waypoints = []
    path = node.path_from_waypoints(waypoints)
    assert isinstance(path, Path), "Result should be a ROS2 Path message."
    assert len(path.poses) == 0, "Path should have zero poses when there are no waypoints."
