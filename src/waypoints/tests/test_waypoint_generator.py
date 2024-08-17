import pytest
import rclpy
from waypoints.waypoint_generator import WaypointGenerator


@pytest.fixture(scope='module')
def rclpy_init_shutdown():
    """A pytest fixture to initialize and shutdown rclpy for ROS2 nodes"""
    rclpy.init()
    yield
    rclpy.shutdown()


def test_coord_from_address(rclpy_init_shutdown):
    """Test if we can get GPS coordinates from an address"""
    node = WaypointGenerator()
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
