# Command an absolute yaw and velocity.

# Yaw is specified in radians counter-clockwise from true east.
float32 yaw

# Velocity is specified in meters/s. Negative values correspond to reversing.
float32 speed

# If true, vessel will maintain station (speed ignored, yaw commanded)
bool keep_station

# If station keeping, need to keep heading aligned with station_yaw 
float32 station_yaw

# Distance from vessel to station, used for stationkeeping PID
float32 station_dist_x
float32 station_dist_y