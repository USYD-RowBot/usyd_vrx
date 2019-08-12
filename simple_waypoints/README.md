# Simple Waypoints
Publishes waypoints from a CSV or GEOJSON to a nav_msgs/Path message

To run:

```bash
roslaunch simple_waypoints simple_waypoints.launch
```

Parameters:
- recycle: When the request is received, re-transmit waypoints
- geo: Waypoints are in geo form, converts to local ENU coords:
  - need params: datum_latitude, datum_longitude, datum_altitude to be set beforehand
- filename: File of waypoints, can be either a CSV or GEOJSON (Not tested with GEOJSON yet #TODO)
