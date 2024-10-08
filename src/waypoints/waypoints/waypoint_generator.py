"""
Waypoint generator for route planning.
"""

from typing import Tuple, Optional, List, cast
import xml.etree.ElementTree as ET

import overpass
import requests
from pyroutelib3 import Router

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Path

BOX_BUFFER = 0.01
START_ADDRESS = "Upson Hall, Rhodes Drive, Ithaca, NY"
END_ADDRESS = "Morrison Hall, Sisson Place, Ithaca, NY"


class WaypointGenerator(Node):
    """
    ROS2 node for generating and publishing waypoint paths.
    """

    # FIXME: how do these slots work?
    # FIXME: we pass the map_path to acount for running pytest and colcon tests--they expect it in different dirs (see the test file...)
    __slots__ = ("publisher_", "stamp", "frame_id", "map_path")

    def __init__(self, map_path, stamp=None, frame_id=None):
        super().__init__('waypoints_node')

        # Publish a Path to the /planning/waypoints topic, queue depth of 1
        self.publisher_ = self.create_publisher(Path, '/planning/waypoints', 1)
        self.get_logger().info("Waypoint routing node started")
        self.stamp = stamp
        self.frame_id = frame_id
        self.map_path = map_path

        # Run the main logic
        self.main()

    def coords_from_address(self, address: str) -> Optional[Tuple[float, float]]:
        """Obtain GPS coordinates (latitude and longitude) from an address."""

        # Open Street Map API
        url = f"https://nominatim.openstreetmap.org/search?q={address}&format=jsonv2&addressdetails=1&limit=1"

        headers = {"User-Agent": "Cornell Autobike/1.0 (dcl252@cornell.edu)"}
        response = requests.get(url, headers=headers, timeout=16)
        response.raise_for_status()

        # Parse the response JSON
        data = response.json()

        # Check if the data contains at least one result
        if data:
            lat = float(data[0].get("lat", 0.0))
            lon = float(data[0].get("lon", 0.0))
            return lat, lon

        return None

    def fetch_map_box(
        self, coord_a: Tuple[float, float], coord_b: Tuple[float, float]
    ) -> Optional[str]:
        """Fetch map data (in XML format) within the bounding box created by two coordinates. Used to update map.osm for routing if needed."""

        # Calculate the bounding box using the provided coordinates
        min_lat = min(coord_a[0], coord_b[0]) - BOX_BUFFER
        min_lon = min(coord_a[1], coord_b[1]) - BOX_BUFFER
        max_lat = max(coord_a[0], coord_b[0]) + BOX_BUFFER
        max_lon = max(coord_a[1], coord_b[1]) + BOX_BUFFER

        # Build the bounding box string for the Overpass API query
        box_string = f"{min_lat},{min_lon},{max_lat},{max_lon}"

        # Fetch the map data from the Overpass API
        try:
            api = overpass.API(timeout=600)
            # FIXME: what the heck does (._;>;) do?!?
            query = f'way({box_string});(._;>;)'
            response = cast(str, api.get(query, responseformat="xml"))

            # Treat trivial responses as None
            # Parse the XML response to check for meaningful map data
            root = ET.fromstring(response)
            if (
                not any(root.iter("node"))
                and not any(root.iter("way"))
                and not any(root.iter("relation"))
            ):
                return None

            return response

        except requests.exceptions.RequestException as e:
            self.get_logger().error(f"Network error: {e}")
        except overpass.errors.OverpassSyntaxError as e:
            self.get_logger().error(f"Overpass syntax error: {e}")
        except overpass.errors.OverpassError as e:
            self.get_logger().error(f"Overpass API error: {e}")

        return None

    def route(
        self, start: Tuple[float, float], end: Tuple[float, float]
    ) -> Optional[List[Tuple[float, float]]]:
        """Get the route between two positions."""

        # Setup the route
        router = Router("cycle", self.map_path, localfileType="xml")

        # Find the closest nodes to the start and end positions
        start_node = router.findNode(start[0], start[1])
        end_node = router.findNode(end[0], end[1])

        # Check if the nodes were found
        if start_node is None or end_node is None:
            return None

        # Calculate the route
        status, route = router.doRoute(start_node, end_node)

        # Check if routing was successful
        if status != 'success':
            return None

        return list(map(router.nodeLatLon, route))

    def path_from_waypoints(self, waypoints: List[Tuple[float, float]]) -> Path:
        """
        Convert a list of waypoints (latitude, longitude) to a ROS2 Path message.
        """

        # Initialize the Path message
        path = Path()
        path.header.stamp = rclpy.clock.Clock().now().to_msg()
        path.header.frame_id = "map"

        # Convert each waypoint to a PoseStamped and append it to the Path message
        for lat, lon in waypoints:
            pose_stamped = PoseStamped()
            pose_stamped.header.stamp = (
                path.header.stamp
            )  # Use the same timestamp for all poses
            pose_stamped.header.frame_id = "map"

            # Set the position (assuming a flat plane, so z = 0.0)
            pose_stamped.pose.position.x = lat
            pose_stamped.pose.position.y = lon
            pose_stamped.pose.position.z = 0.0

            # Orientation is left as default (no specific heading or rotation set)
            path.poses.append(pose_stamped)

        return path

    def main(self):
        """Main node logic"""
        # Get start and end positions from addresses
        start_coords = self.coords_from_address(START_ADDRESS)
        end_coords = self.coords_from_address(END_ADDRESS)

        if not start_coords or not end_coords:
            self.get_logger().error(
                "Could not retrieve coordinates for the start or end address."
            )
            return

        # Generate the route as a list of waypoints (latitude, longitude)
        route = self.route(start_coords, end_coords)
        if not route or len(route) <= 1:
            self.get_logger().error("Could not find a valid route.")
            return

        # Convert the route to a nav_msgs/Path message and publish
        path = self.path_from_waypoints(route)
        self.publisher_.publish(path)
        self.get_logger().info(f"Published path with {len(path.poses)} waypoints")


def main(args=None):
    """Run on `ros2 run waypoints waypoints`"""
    rclpy.init(args=args)

    node = WaypointGenerator(map_path="./src/waypoints/map.osm")
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


def run_once(args=None):
    """Run the waypoint generation node once and then exit (useful for testing)."""
    rclpy.init(args=args)

    # FIXME: It seems the node runs once on initialization?
    node = WaypointGenerator(map_path="./src/waypoints/map.osm")
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
