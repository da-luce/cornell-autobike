"""Unit tests for the WaypointGenerator class."""

# pylint: disable=redefined-outer-name

import os
import pytest
import rclpy

from nav_msgs.msg import Path

# FIXME: this import works for pytest and colcon test but not pylint :(
# pylint: disable=import-error, no-name-in-module
from waypoints.waypoint_generator import WaypointGenerator


@pytest.fixture(scope='module')
def node():
    """Create a WaypointGenerator node"""
    rclpy.init()

    # IMPORTANT: this forces both colcon test and pytest to find the correct map.osm file (in the root of the ROS waypoints package)
    map_path = os.path.join(os.path.dirname(__file__), "..", "map.osm")
    waypoint_node = WaypointGenerator(map_path)
    yield waypoint_node

    rclpy.shutdown()


def test_coord_from_address(node):
    """Test if we can get GPS coordinates from an address"""

    location = node.coords_from_address("Upson Hall, Ithaca, NY")

    assert isinstance(location, tuple), "Location should be a tuple."
    assert (
        len(location) == 2
    ), "Location should contain exactly two elements (latitude, longitude)."
    assert all(
        isinstance(coord, float) for coord in location
    ), "Both latitude and longitude should be floats."

    lat, lon = location
    assert 42.443 <= lat <= 42.444, f"Latitude out of range: {lat}"
    assert -76.483 <= lon <= -76.482, f"Longitude out of range: {lon}"


def test_fetch_empty_map(node):
    """Test fetching map data within a bounding box."""

    coord_a = (42.443, -76.484)
    coord_b = (42.444, -76.482)

    # Test valid coordinates
    result = node.fetch_map_box(coord_a, coord_b)
    assert isinstance(result, str), "Result should be an XML string."
    assert "<osm" in result, "The result should contain valid OSM XML data."

    # Test with invalid bounding box (expecting None)
    result = node.fetch_map_box((0.0, 0.0), (0.0, 0.0))
    assert (
        result is None
    ), "Result should be None for an invalid or trivial bounding box."


def test_fetch_something_map(node):
    """Test fetching map data within a larger bounding box that should return meaningful data."""

    # Define a larger bounding box that covers an area in Ithaca, NY
    coord_a = (42.440, -76.500)
    coord_b = (42.450, -76.470)

    result = node.fetch_map_box(coord_a, coord_b)

    # Ensure the result is a valid XML string containing OSM data
    assert isinstance(result, str), "Result should be an XML string."
    assert "<osm" in result, "The result should contain valid OSM XML data."

    # Check that the result contains meaningful data (like nodes or ways)
    assert (
        "<node" in result or "<way" in result
    ), "The result should contain nodes or ways."


def test_route(node):
    """Test generating a route between two coordinates."""

    # Simulated coordinates for start and end locations
    start = (42.443, -76.483)
    end = (42.444, -76.482)

    route = node.route(start, end)
    assert route is not None, "Route should not be None."
    assert len(route) > 1, "Route should contain more than one waypoint."
    assert all(
        isinstance(coord, tuple) and len(coord) == 2 for coord in route
    ), "Each waypoint should be a tuple of (latitude, longitude)."


def test_path_from_waypoints(node):
    """Test converting waypoints into a ROS2 Path message."""

    # Simulated waypoints
    waypoints = [(42.443, -76.483), (42.444, -76.482)]

    path = node.path_from_waypoints(waypoints)
    assert isinstance(path, Path), "Result should be a ROS2 Path message."
    assert len(path.poses) == 2, "Path should contain two poses."
    assert path.header.frame_id == "map", "Frame ID should be 'map'."
    assert path.header.stamp is not None, "Timestamp should be set."


def test_invalid_address(node):
    """Test if coords_from_address returns None for an invalid address."""

    location = node.coords_from_address("Nonexistent Address, Nowhere")

    assert location is None, "Location should be None for an invalid address."
