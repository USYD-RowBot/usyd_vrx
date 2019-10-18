#!/usr/bin/env python

# Checks info messages to determine what needs to be done.

import rospy
from vrx_gazebo.msg import Task
params = {
    "taskTopic": "/vrx/task/info",
}

rospy.init_node("missionControl.py",anonymous=True)

for i in params:
    params[i] = rospy.get_param('~'+i, params[i])

def cb(data):
    if data.name=="station_keeping":
        pass
    elif data.name=="wayfinding":
        pass
    elif data.name=="perception":
        pass
    elif data.name=="navigation_course":
        pass
    elif data.name=="scan":
        pass
    elif data.name=="scan_and_dock":
        pass
    
        # do something

pub = rospy.Subscriber(params['taskTopic'], Task,cb)
rospy.spin()