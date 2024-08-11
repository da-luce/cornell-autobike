"""
Waypoint routing
"""

# Fetch and write data to src/waypoints/map.osm
# TODO: add command line args
import sys
import statistics as stat
import math

import tkinter
import tkintermapview
import overpass
from pyroutelib3 import Router
import osmnx


# Parameters

# Space to include around start and end positions
BOX_BUFFER = 0.01  # This seems to be a reasonable value


# Defaults

START_ADDRESS = '''Upson Hall, Rhodes Drive, Ward 4,
                   City of Ithaca, Tompkins County,
                   New York, 14853, United States'''
END_ADDRESS = '''Morrison Hall, 10, Sisson Place, Ward 5,
                 City of Ithaca, Tompkins County,
                 New York, 14850, United States'''

start_coords = tkintermapview.convert_address_to_coordinates(START_ADDRESS)
end_coords = tkintermapview.convert_address_to_coordinates(END_ADDRESS)


# Functions


def initialize_tk():
    """
    Initialize tk
    """

    # Create tkinter window
    root_tk = tkinter.Tk()
    root_tk.geometry(f"{800}x{600}")
    root_tk.title("Choose Start and End")

    return root_tk


def create_map_widget(root_tk):
    """
    Create tk map widget
    """

    # Create map widget
    map_widget = tkintermapview.TkinterMapView(
        root_tk, width=800, height=600, corner_radius=0
    )
    map_widget.place(relx=0.5, rely=0.5, anchor=tkinter.CENTER)

    map_widget.set_zoom(10)

    return map_widget


def bounding_box(lat1, lon1, lat2, lon2, buffer):
    """
    Create a bounding box with the given buffer
    """

    # TODO: check for negatives?

    # Buffer: increase each edge of the box defined by lat1, lon1,
    # lat2, lon2 by a buffer

    return (
        min(lat1, lat2) - buffer,
        min(lon1, lon2) - buffer,
        max(lat1, lat2) + buffer,
        max(lon1, lon2) + buffer,
    )


def generate_box(point_a, point_b):
    """
    Generate a bounding box given two points
    """

    box = bounding_box(point_a[0], point_a[1], point_b[0], point_b[1], BOX_BUFFER)
    return test_map_widget.set_polygon(
        [(box[0], box[1]), (box[2], box[1]), (box[2], box[3]), (box[0], box[3])],
        name="Bounding Box",
    )


def box_size(box):
    """
    Return the size of a bounding box
    """
    return (box[2] - box[1]) * (box[3] - box[0])


def set_start(coords):
    """
    Set the widget start indicator at the coords
    """
    print("Start set to ", coords)
    start_marker.set_text("Start")
    start_marker.set_position(coords[0], coords[1])

    test_map_widget.delete_all_polygon()
    generate_box(start_marker.position, end_marker.position)


def set_end(coords):
    """
    Set the widget end indicator at the coords
    """
    print("End set to ", coords)
    end_marker.set_text("End")
    end_marker.set_position(coords[0], coords[1])

    test_map_widget.delete_all_polygon()
    generate_box(start_marker.position, end_marker.position)


def left_click_event(coordinates_tuple):
    """
    Left click event handler
    """
    print("Left click event with coordinates:", coordinates_tuple)


def bind_events(map_widget):
    """
    Bind click events to the map widget
    """

    map_widget.add_left_click_map_command(left_click_event)

    map_widget.add_right_click_menu_command(
        label="Set start", command=set_start, pass_coords=True
    )

    map_widget.add_right_click_menu_command(
        label="Set end", command=set_end, pass_coords=True
    )


def yes_or_no(question):
    """
    Yes or no helper function
    """
    while "the answer is invalid":
        reply = str(input(question + ' (y/n): ')).lower().strip()
        if reply[0] == 'y':
            return True
        if reply[0] == 'n':
            return False


def fetch_data(point_a, point_b):
    """
    Fetch map data for two points
    """

    # Get the final bounding box
    box = bounding_box(point_a[0], point_a[1], point_b[0], point_b[1], BOX_BUFFER)

    if box_size(box) > 0.25:
        print("Your bounding box is too large! (over 25 square degrees)")
        sys.exit()

    box_string = f"{box[0]},{box[1]},{box[2]},{box[3]}"

    # FIXME: this may generate edges that are split (nodes outside bounding box
    # are dropped)
    # FIXME: what does ;(._;>;) do!?!? But it works now!!!
    api = overpass.API(timeout=600)
    return api.get(f'way({box_string});(._;>;)', responseformat="xml")


def write_to_disk(response, filepath):
    """
    File writing helper
    """
    open(filepath, 'w').write(response)


def test_route(start_pos, end_pos, filepath):
    """
    Get the route between two positions
    """

    router = Router("cycle", filepath, localfileType="xml")

    start = router.findNode(start_pos[0], start_pos[1])  # Find start and end nodes
    end = router.findNode(end_pos[0], end_pos[1])

    status, route = router.doRoute(start, end)  # Find the route - a list of OSM nodes

    if status == 'success':
        route_lat_lons = list(
            map(router.nodeLatLon, route)
        )  # Get actual`` route coordinates
    else:
        route_lat_lons = []

    return route_lat_lons, (status == 'success')


# FIXME: this is a really shitty algorithm
def add_nodes_to_route(route, max_dist):
    """
    Added additional nodes to `route` (a of list of tuples (lat, lon)) such that
    the distance between any two nodes in the route is less than or equivalent
    to `max_dist`.

    The current implementation simply loops through each pair of points in the
    original route and adds point a distance `max_dist` away from the first
    point in the pair until the added point is within `max_dist` of the second
    point in the pair.
    """

    def node_distance(node_a, node_b):
        return math.dist(node_a, node_b)

    i = 0
    while i < len(route) - 1:

        node_a = route[i]
        node_b = route[i + 1]

        dist = node_distance(node_a, node_b)

        # start at node_a and iterate until we are within max_dist of node_b
        current_node = node_a

        # track how many nodes we have added
        added_nodes = 0

        # Add new points along the edge formed by node_a , node_b
        while dist > max_dist:

            # algo as described by
            # https://math.stackexchange.com/questions/175896/finding-a-point-along-a-line-a-certain-distance-away-from-another-point
            dist_ratio = max_dist / dist

            # TODO: perhaps using numpy would be better
            lat = (1 - dist_ratio) * current_node[0] + dist_ratio * node_b[0]
            lon = (1 - dist_ratio) * current_node[1] + dist_ratio * node_b[1]

            current_node = (lat, lon)

            # insert the new node (remember how List.insert works)
            route.insert(i + added_nodes + 1, current_node)

            added_nodes += 1

            dist = node_distance(current_node, node_b)

        i += 1
        i += added_nodes  # offset by the added nodes

    return route


if __name__ == '__main__':

    # Initialize map
    test_root_tk = initialize_tk()
    test_map_widget = create_map_widget(test_root_tk)
    bind_events(test_map_widget)

    test_map_widget.set_position(
        stat.mean([start_coords[0], end_coords[0]]),
        stat.mean([start_coords[1], end_coords[1]]),
    )

    start_marker = test_map_widget.set_marker(
        start_coords[0], start_coords[1], text="Default Start"
    )

    end_marker = test_map_widget.set_marker(
        end_coords[0], end_coords[1], text="Default End"
    )

    # Generate initial bounding box
    generate_box(start_marker.position, end_marker.position)

    # Display the map
    test_root_tk.mainloop()

    # Once window closes:
    # FIXME: random errors here...

    # Query if user wants to download new data
    if yes_or_no("Fetch new data"):

        print("Fetching data...", end="", flush=True)
        test_response = fetch_data(start_marker.position, end_marker.position)
        print("✅")

        print("Writing data to src/waypoints/map.osm...", end="", flush=True)
        write_to_disk(test_response, "src/waypoints/map.osm")
        print("✅")

        # For debugging
        print("Generating debug map...", end="", flush=True)
        graph = osmnx.graph_from_xml("src/waypoints/map.osm")
        fig = osmnx.plot_graph(graph, node_color='r')
        print("✅")

    else:
        print("Enter path to local file: (not implemented yet...)")

    # Route the bike!
    print("Routing bike...", end="", flush=True)
    test_route, success = test_route(
        start_marker.position, end_marker.position, "src/waypoints/map.osm"
    )
    if success and len(test_route) > 1:
        print("✅")
        print(test_route)
    else:
        print("🔥")
        print("Could not route bike.")
        sys.exit()

    # Open a new tkinter window
    # FIXME: this is janky
    test_root_tk = initialize_tk()
    test_map_widget = create_map_widget(test_root_tk)

    test_map_widget.set_path(test_route)
    test_map_widget.set_marker(
        start_marker.position[0], start_marker.position[1], text="Start"
    )
    test_map_widget.set_marker(
        end_marker.position[0], end_marker.position[1], text="End"
    )

    test_map_widget.set_position(
        stat.mean([start_marker.position[0], end_marker.position[0]]),
        stat.mean([start_marker.position[1], end_marker.position[1]]),
    )

    generate_box(start_marker.position, end_marker.position)

    # Display route
    test_root_tk.mainloop()
