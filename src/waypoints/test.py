from pyroutelib3 import Router
import osmnx as ox
import matplotlib.pyplot as plt

router = Router("cycle", "ithaca.osm", localfileType="xml")

start = router.findNode(42.460581, -76.481905)  # Find start and end nodes
end = router.findNode(42.436135, -76.525622)

status, route = router.doRoute(start, end)  # Find the route - a list of OSM nodes

if status == 'success':
    routeLatLons = list(map(router.nodeLatLon, route))  # Get actual`` route coordinates

region_name = 'City of Ithaca,New York' 

graph = ox.graph_from_xml("ithaca.osm")

# Plot the streets
fig = plt.figure()
ax = fig.add_subplot(111)
fig = ox.plot_graph(graph, ax=ax, node_size=2, node_color='r', edge_color='w', edge_linewidth=0.2)

ax.set_title('custom picker for line data')
def click_handler(event):
    if event.inaxes is not None:
        print(event.xdata, event.ydata)
    else:
        print('Clicked ouside axes bounds but inside plot window')

fig.canvas.callbacks.connect('button_press_event', click_handler)
# fig.canvas.mpl_connect('pick_event', on_pick)

# x, y = zip(*routeLatLons)

# fix2, ax = plt.scatter(x, y)

plt.show()
