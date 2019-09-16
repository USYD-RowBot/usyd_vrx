#!/usr/bin/env python

# Checks info messages to determine what needs to be done.

import rospy
from vrx_gazebo.msg import Task
params = {
    "taskTopic": "/waypoints",
}

rospy.init_node("missionControl.py",anonymous=True)

for i in params:
    params[i] = rospy.get_param('~'+i, params[i])

def cb(data):
    if data.name=="something":
        # do something

pub = rospy.Subscriber(params['taskTopic'], Task,cb);

def cb(data):
    print("RECEIVED WAYPOINT MESSAGE")
    global pub
    global listener
    route = WaypointRoute()
    waypoints = []
    for dps in data.poses:
        wp=Waypoint()
        wp.pose=dps.pose # it starts as a posestamped so extract the pose
        wp.nav_type=wp.NAV_WAYPOINT
        waypoints.append(wp)
    route.waypoints=waypoints
    route.speed=params["speed"]
    pub.publish(route)

sub = rospy.Subscriber(params['inTopic'], Path, cb)
rospy.spin()