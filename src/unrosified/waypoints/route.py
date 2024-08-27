"""
Waypoint routing
"""

import math
import statistics as stat
import sys
import tkinter

import osmnx
import overpass
import requests
import tkintermapview
from pyroutelib3 import Router

# Parameters
BOX_BUFFER = 0.01  # Space to include around start and end positions

# Defaults
START_ADDRESS = '''Upson Hall, Rhodes Drive, Ward 4,
                City of Ithaca, Tompkins County, New York, 14853,
                United States'''
END_ADDRESS = '''Morrison Hall, 10, Sisson Place, Ward 5,
                City of Ithaca, Tompkins County, New York, 14850,
                United States'''


def initialize_tk():
    """Initialize tkinter window."""
    root_tk = tkinter.Tk()
    root_tk.geometry(f"{800}x{600}")
    root_tk.title("Choose Start and End")

    return root_tk


def on_closing(root_tk, map_widget):
    """Cleanup function to be called when the window is closed."""
    map_widget.destroy()
    root_tk.destroy()  # Safely close the window


def create_map_widget(root_tk):
    """Create tkinter map widget."""
    map_widget = tkintermapview.TkinterMapView(
        root_tk, width=800, height=600, corner_radius=0
    )
    map_widget.place(relx=0.5, rely=0.5, anchor=tkinter.CENTER)
    map_widget.set_zoom(10)
    return map_widget


def bounding_box(lat1, lon1, lat2, lon2, buffer):
    """Create a bounding box with the given buffer."""
    return (
        min(lat1, lat2) - buffer,
        min(lon1, lon2) - buffer,
        max(lat1, lat2) + buffer,
        max(lon1, lon2) + buffer,
    )


def generate_box(point_a, point_b, map_widget):
    """Generate a bounding box given two points."""
    # TODO: check for negatives?
    box = bounding_box(point_a[0], point_a[1], point_b[0], point_b[1], BOX_BUFFER)
    return map_widget.set_polygon(
        [(box[0], box[1]), (box[2], box[1]), (box[2], box[3]), (box[0], box[3])],
        name="Bounding Box",
    )


def box_size(box):
    """Return the size of a bounding box."""
    return (box[2] - box[0]) * (box[3] - box[1])


def set_start(coords, map_widget, start_marker, end_marker):
    """Set the widget start indicator at the coords."""
    print("Start set to ", coords)
    start_marker.set_text("Start")
    start_marker.set_position(coords[0], coords[1])

    map_widget.delete_all_polygon()
    generate_box(start_marker.position, end_marker.position, map_widget)


def set_end(coords, map_widget, start_marker, end_marker):
    """Set the widget end indicator at the coords."""
    print("End set to ", coords)
    end_marker.set_text("End")
    end_marker.set_position(coords[0], coords[1])

    map_widget.delete_all_polygon()
    generate_box(start_marker.position, end_marker.position, map_widget)


def left_click_event(coordinates_tuple):
    """Left click event handler."""
    print("Left click event with coordinates:", coordinates_tuple)


def bind_events(map_widget, start_marker, end_marker):
    """Bind click events to the map widget."""
    map_widget.add_left_click_map_command(left_click_event)
    map_widget.add_right_click_menu_command(
        label="Set start",
        command=lambda coords: set_start(coords, map_widget, start_marker, end_marker),
        pass_coords=True,
    )
    map_widget.add_right_click_menu_command(
        label="Set end",
        command=lambda coords: set_end(coords, map_widget, start_marker, end_marker),
        pass_coords=True,
    )


def yes_or_no(question):
    """Yes or no helper function."""
    while True:
        reply = str(input(question + ' (y/n): ')).lower().strip()
        if reply[0] == 'y':
            return True
        if reply[0] == 'n':
            return False


def fetch_data(point_a, point_b):
    """Fetch map data for two points."""
    box = bounding_box(point_a[0], point_a[1], point_b[0], point_b[1], BOX_BUFFER)

    if box_size(box) > 0.25:
        print("Your bounding box is too large! (over 25 square degrees)")
        sys.exit()

    box_string = f"{box[0]},{box[1]},{box[2]},{box[3]}"

    # FIXME: this may generate edges that are split (nodes outside bounding box)
    # are dropped)
    # FIXME: what does ;(._;>;) do!?!? But it works now!!!
    api = overpass.API(timeout=600)
    return api.get(f'way({box_string});(._;>;)', responseformat="xml")


def write_to_disk(response, filepath):
    """File writing helper."""
    with open(filepath, 'w', encoding='utf-8') as file:
        file.write(response)


def test_route(start_pos, end_pos, filepath):
    """Get the route between two positions."""
    router = Router("cycle", filepath, localfileType="xml")

    start = router.findNode(start_pos[0], start_pos[1])  # Find start and end nodes
    end = router.findNode(end_pos[0], end_pos[1])

    status, route = router.doRoute(start, end)  # Find the route - a list of OSM nodes

    if status == 'success':
        route_lat_lons = list(
            map(router.nodeLatLon, route)
        )  # Get actual route coordinates
    else:
        route_lat_lons = []

    return route_lat_lons, (status == 'success')


# FIXME: this is a really shitty algorithm
# FIXME: need to account for curvature in road?
def add_nodes_to_route(route, max_dist):
    """
    Add additional nodes to `route` so that the distance between
    any two nodes is less than or equal to `max_dist`.
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
        # algo as described by
        # https://math.stackexchange.com/questions/175896/finding-a-point-along-a-line-a-certain-distance-away-from-another-point
        while dist > max_dist:
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


def address_to_location(address):
    """Convert an address to an OSM place. Make sure to include a user agent!"""
    url = f"https://nominatim.openstreetmap.org/search?q={address}&format=jsonv2&addressdetails=1&limit=1"
    headers = {"User-Agent": "Cornell Autobike/1.0 (dcl252@cornell.edu)"}
    response = requests.get(url, headers=headers, timeout=16)
    response.raise_for_status()
    return response.json()


def main():
    """Main function to encapsulate the script logic."""
    # Initialize map
    start_location = address_to_location(START_ADDRESS)[0]
    end_location = address_to_location(END_ADDRESS)[0]
    start_lat = float(start_location['lat'])
    start_lon = float(start_location['lon'])
    end_lat = float(end_location['lat'])
    end_lon = float(end_location['lon'])

    root_tk = initialize_tk()
    map_widget = create_map_widget(root_tk)
    root_tk.protocol("WM_DELETE_WINDOW", root_tk.quit)

    start_marker = map_widget.set_marker(start_lat, start_lon, text="Default Start")
    end_marker = map_widget.set_marker(end_lat, end_lon, text="Default End")
    map_widget.set_position(start_lat, start_lon)
    map_widget.set_zoom(14)

    bind_events(map_widget, start_marker, end_marker)
    generate_box(start_marker.position, end_marker.position, map_widget)

    # Display the map
    root_tk.mainloop()
    map_widget.destroy()

    # Query if user wants to download new data
    if yes_or_no("Fetch new data"):

        print("Fetching data...", end="", flush=True)
        response = fetch_data((start_lat, start_lon), (end_lat, end_lon))
        print("✅")

        print("Writing data to src/waypoints/map.osm...", end="", flush=True)
        write_to_disk(response, "src/waypoints/map.osm")
        print("✅")

        print("Generating debug map...", end="", flush=True)
        graph = osmnx.graph_from_xml("src/waypoints/map.osm")
        osmnx.plot_graph(graph, node_color='r')
        print("✅")

    else:
        print("Enter path to local file: (not implemented yet...)")

    # Route the bike
    print("Routing bike...", end="", flush=True)
    route, success = test_route(
        start_marker.position, end_marker.position, "src/waypoints/ithaca.osm"
    )
    if success and len(route) > 1:
        print("✅")
        print(route)
    else:
        print("🔥 Could not route bike.")
        sys.exit()

    # Open a new tkinter window to display the route
    # FIXME: this is janky
    root_tk = initialize_tk()
    map_widget = create_map_widget(root_tk)
    # root_tk.protocol("WM_DELETE_WINDOW", root_tk.destroy)

    map_widget.set_path(route)
    map_widget.set_marker(
        start_marker.position[0], start_marker.position[1], text="Start"
    )
    map_widget.set_marker(end_marker.position[0], end_marker.position[1], text="End")

    map_widget.set_position(
        stat.mean([start_marker.position[0], end_marker.position[0]]),
        stat.mean([start_marker.position[1], end_marker.position[1]]),
    )

    generate_box(start_marker.position, end_marker.position, map_widget)

    # Display route
    root_tk.mainloop()


if __name__ == '__main__':
    main()
