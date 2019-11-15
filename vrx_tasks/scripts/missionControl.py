#!/usr/bin/env python

# Checks info messages to determine what needs to be done.

from components import geoPathToPath, geoPoseToPose, poseToCourse, navigationAI
import rospy
from vrx_msgs.msg import Task
params = {
    "taskTopic": "/vrx/task/info",
}

rospy.init_node("missionControl.py",anonymous=True)

for i in params:
    params[i] = rospy.get_param('~'+i, params[i])
initialised=False
def cb(data):
    global initialised
    if (not initialised):
        if data.name=="station_keeping":
            ## geo pose to pose
            geoPoseToPose.geoPoseToPoseConverter()
            ## pose to course
            poseToCourse.poseToCourseConverter()
            pass
        elif data.name=="wayfinding":
            geoPathToPath.geoPathToPathConverter()
            pass
        elif data.name=="perception":
            pass
        elif data.name=="navigation_course":
            navigationAI.navigationAI()
            pass
        elif data.name=="scan":
            pass
        elif data.name=="scan_and_dock":
            pass
        initialised=True
        # do something

rospy.Subscriber(params['taskTopic'], Task,cb)
rospy.spin()