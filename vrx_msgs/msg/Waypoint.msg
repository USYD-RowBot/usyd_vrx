# Constants describing different waypoint behaviours.
#   NAV_WAYPOINT: A waypoint for the vesesl to pass through on a route, position required only.
#   NAV_STATION:  A station for the vessel to reach and align a position and yaw, for a duration.

uint8 NAV_WAYPOINT=0     
uint8  NAV_STATION=1

# MANDATORY:
uint8 nav_type           # Type of waypoint, specify with NAV constants above
geometry_msgs/Pose pose  # Desired pose for waypoint/station

# IF NAV_STATION:
float32 station_duration # Time in seconds to keep station, negative is indefinite
