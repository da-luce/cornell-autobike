import pytest
import math
from unittest.mock import patch, mock_open
from src.waypoints.route import (
    bounding_box,
    box_size,
    add_nodes_to_route,
    fetch_data,
    write_to_disk,
)


def test_bounding_box():
    """Test the bounding_box function with a buffer."""
    lat1, lon1 = 42.444, -76.484
    lat2, lon2 = 42.449, -76.477
    buffer = 0.01

    result = bounding_box(lat1, lon1, lat2, lon2, buffer)

    expected = (42.434, -76.494, 42.459, -76.467)
    assert result == pytest.approx(expected), f"Expected {expected}, but got {result}"


def test_box_size():
    """Test the box_size function."""
    box = (42.434, -76.494, 42.459, -76.467)
    result = box_size(box)
    expected_size = (42.459 - 42.434) * (-76.467 + 76.494)
    assert result == expected_size, f"Expected {expected_size}, but got {result}"


def test_add_nodes_to_route():
    """Test add_nodes_to_route for a simple case."""
    route = [(0, 0), (0, 10)]
    max_dist = 3

    result = add_nodes_to_route(route, max_dist)
    expected_length = (
        math.ceil(10 / max_dist) + 1
    )  # should have additional points added

    assert (
        len(result) == expected_length
    ), f"Expected {expected_length} nodes, but got {len(result)}"
    assert result[0] == (0, 0), "Expected the first node to be (0, 0)"
    assert result[-1] == (0, 10), "Expected the last node to be (0, 10)"


@patch('overpass.API')
def test_fetch_data(mock_overpass_api):
    """Test fetch_data by mocking the Overpass API."""
    pointA = (42.444, -76.484)
    pointB = (42.449, -76.477)

    mock_api_instance = mock_overpass_api.return_value
    mock_api_instance.get.return_value = "<osm></osm>"

    result = fetch_data(pointA, pointB)

    mock_api_instance.get.assert_called_once()
    assert result == "<osm></osm>", f"Expected '<osm></osm>', but got {result}"


@patch("builtins.open", new_callable=mock_open)
def test_write_to_disk(mock_file):
    """Test write_to_disk by mocking the file write operation."""
    data = "<osm></osm>"
    filepath = "src/waypoints/map.osm"

    write_to_disk(data, filepath)

    mock_file.assert_called_once_with(filepath, 'w', encoding='utf-8')
    mock_file().write.assert_called_once_with(data)
