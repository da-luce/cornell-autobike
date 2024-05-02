# Waypoint Design Doc

## Problem Statement

> Currently we have the ability to navigate based on where we see, avoiding
> obstacles in the way. However, we don't have a way to do large-distance
> navigation. I was recently reading some of [navigation's plans from the past](https://drive.google.com/file/d/0B6KdkZUjU006VWdEQTdoZzRERDQ/view?resourcekey=0-UbjFfBvkBSd_YXUj8sl95g) and I
> found that we were planning on doing path planning from waypoints using google
> maps' API. Because we can currently only navigate to things within sight range,
> I think it would make sense to have google maps give us the path between two
> locations and to have waypoints every 50 feet or so along the way for our
> current navigation algorithm to take us between. The google maps API looks to
> be free for only 90 days, but looks to have relatively reasonable pricing with
> $0.60 per 1000 API calls. We can charge this to the team if we end up using it,
> but should have caching systems in place to minimize how many calls we make.
>
> * The first step would be to translate a geolocation to a point in our state
>   matrix to properly set up the reward function.
> * The second step is to determine when we get "close enough" to a finishing state
>   to say that we completed that leg of the journey.
> * The third step is to transition between geolocation ending states.
> * The fourth step is to use google maps path planning to create waypoints every
>   50ft or so along a path for the big to travel.
>
> ~ Ari

## Design Proposal

**Inputs:** two GPS coordinates representing the starting and ending position of
the paths

* [Download](https://wiki.openstreetmap.org/wiki/Downloading_data) appropriate
  region of data from OpenSteetMap (OSM) for offline use
* Convert input GPS coordinates to _node_ _elements_
* Use [PyrouteLib](https://wiki.openstreetmap.org/wiki/PyrouteLib) to determine
  effective route (it even comes with a cycle mode!)
* Get waypoints from generated route

New ideas:

**Online:**
Choose start and end points using <https://github.com/TomSchimansky/TkinterMapView>

Automatically generate bounding box and download data using
<https://wiki.openstreetmap.org/wiki/Overpass_API>

Proceed with same offline plan...

## Fetching Map Data

In order to run the bike offline, we need street map data downloaded to the
repository! Use `data_fetch.py` to do this.

Run `data_fetch.py` within a X11 window using the `way` service (see [Docker Setup](/README.md))

For instance, the command on MacOS would be:

```text
docker-compose run --rm --env DISPLAY=host.docker.internal:0 way python data_fetch.py
```

Once the GUI has loaded

* Right click to set the start and end positions
* Close the window to download data to the local file `src/waypoints/map.osm`

## Visualizing Map Data

WIP

## Creating a Route

WIP