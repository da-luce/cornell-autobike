from pyroutelib3 import Router
import osmnx as ox
import matplotlib.pyplot as plt

router = Router("cycle", "/usr/app/src/waypoints/ithaca.osm", localfileType="xml")

start = router.findNode(42.460581, -76.481905)  # Find start and end nodes
end = router.findNode(42.436135, -76.525622)

status, route = router.doRoute(start, end)  # Find the route - a list of OSM nodes

if status == 'success':
    routeLatLons = list(map(router.nodeLatLon, route))  # Get actual`` route coordinates


x, y = zip(*routeLatLons)
plt.scatter(x, y)
plt.show()
