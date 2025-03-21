# Waypoint Generator Node

## Summary

The Waypoint Generator calculates and publishes a navigation path between two addresses using OpenStreetMap (OSM) data. The node queries GPS coordinates for the given addresses, retrieves map data, and computes an optimal route. The route is then converted into a ROS2 Path message and published for use in navigation planning.

## Dependencies

* `rclpy` (ROS2 Python client library)
* `requests` (For querying address coordinates from OSM)
* `overpass` (For fetching map data)
* `pyroutelib3` (For route planning)
* `geometry_msgs` and `nav_msgs` (For publishing path messages)

## Inputs

* Start Address: Upson Hall, Rhodes Drive, Ithaca, NY
* End Address: Morrison Hall, Sisson Place, Ithaca, NY
* Map File: map.osm (XML format, fetched from Overpass API if needed)

## Outputs

Publishes a `nav_msgs/Path` message to the `/planning/waypoints` topic, containing a list of waypoints (latitude, longitude) converted to `PoseStamped` messages.
