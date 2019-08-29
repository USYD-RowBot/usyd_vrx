uint8 NAV_WAYPOINT=0     # Constants describing different waypoint behaviours
uint8  NAV_STATION=1

# MANDATORY:
uint8 nav_type           # Type of waypoint, specify with NAV constants above
geometry_msgs/Pose pose  # Desired pose for waypoint/station

# IF NAV_STATION:
float32 station_duration # Time in seconds to keep station, negative is indefinite
