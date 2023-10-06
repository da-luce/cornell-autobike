#!/usr/bin/env python

# Fetch and write data to map.osm
# TODO: add command line args

import tkinter
import tkintermapview
import requests
import overpass

# PARAMETERS

box_buffer = 0.01

# Create tkinter window
root_tk = tkinter.Tk()
root_tk.geometry(f"{800}x{600}")
root_tk.title("Choose Start and End")

# Create map widget
map_widget = tkintermapview.TkinterMapView(root_tk, width=800,
                                           height=600, corner_radius=0)
map_widget.place(relx=0.5, rely=0.5, anchor=tkinter.CENTER)

# Center Map
center_address = '''Cornell University'''
center_coords = tkintermapview.convert_address_to_coordinates(center_address)

map_widget.set_position(center_coords[0], center_coords[1])
map_widget.set_zoom(15)

# Markers
start_address = '''Upson Hall, Rhodes Drive, Ward 4,
                   City of Ithaca, Tompkins County,
                   New York, 14853, United States'''
end_address = '''Morrison Hall, 10, Sisson Place, Ward 5,
                 City of Ithaca, Tompkins County,
                 New York, 14850, United States'''

start_coords = tkintermapview.convert_address_to_coordinates(start_address)
end_coords = tkintermapview.convert_address_to_coordinates(end_address)

start_marker = map_widget.set_marker(start_coords[0],
                                     start_coords[1],
                                     text="Default Start")
end_marker = map_widget.set_marker(end_coords[0],
                                   end_coords[1],
                                   text="Default End")

def bounding_box(lat1, lon1, lat2, lon2, buffer):

    # TODO: check for negatives?

    # Buffer: increase each edge of the box defined by lat1, lon1,
    # lat2, lon2 by a buffer

    return (min(lat1, lat2) - buffer,
            min(lon1, lon2) - buffer,
            max(lat1, lat2) + buffer,
            max(lon1, lon2) + buffer)

def generate_box(pointA, pointB):

    box = bounding_box(pointA[0],
                       pointA[1],
                       pointB[0],
                       pointB[1],
                       box_buffer)
    return map_widget.set_polygon([(box[0], box[1]),
                                   (box[2], box[1]),
                                   (box[2], box[3]),
                                   (box[0], box[3])],
                                  name="Bounding Box")

def box_size(bounding_box):
    return (box[2] - box[1]) * (box[3] - box[0])

generate_box(start_marker.position, end_marker.position)

def set_start(coords):
    print("Start set to ", coords)
    start_marker.set_text("Start")
    start_marker.set_position(coords[0], coords[1])

    map_widget.delete_all_polygon()
    generate_box(start_marker.position, end_marker.position)

def set_end(coords):
    print("End set to ", coords)
    end_marker.set_text("End")
    end_marker.set_position(coords[0], coords[1])

    map_widget.delete_all_polygon()
    generate_box(start_marker.position, end_marker.position)

def left_click_event(coordinates_tuple):
    print("Left click event with coordinates:", coordinates_tuple)

map_widget.add_left_click_map_command(left_click_event)

map_widget.add_right_click_menu_command(label="Set start",
                                        command=set_start,
                                        pass_coords=True)

map_widget.add_right_click_menu_command(label="Set end",
                                        command=set_end,
                                        pass_coords=True)

root_tk.mainloop()

# Get the final bounding box
box = bounding_box(start_marker.position[0],
                   start_marker.position[1],
                   end_marker.position[0],
                   end_marker.position[1],
                   box_buffer)

if (box_size(box) > 25):
    print("Your bounding box is too large! (over 25 square degrees)")
    exit()

box_string = f"{box[0]}, {box[1]}, {box[2]}, {box[3]}"

print("Fetching data...")

# FIXME: this may generate edges that are split (nodes outside bounding box
# are dropped)
api = overpass.API(timeout=600)
response = api.get(f'nwr({box_string})',
                   responseformat="xml")

print("Writing data to file...")

open('map.osm', 'w').write(response)
