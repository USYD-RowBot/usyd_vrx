#!/usr/bin/env python

# Checks info messages to determine what needs to be done.

from components import geoPathToPath, geoPoseToPose, poseToRoute, pathToRoute
from components.navigation_task import NavigationTask
from components.dockmaster import DockMaster
import rospy
from vrx_msgs.msg import Task
params = {
    "taskTopic": "/vrx/task/info",
}

rospy.init_node("missionControl")
rospy.loginfo("Starting Mission Planner")

for i in params:
    params[i] = rospy.get_param('~'+i, params[i])
initialised=False
def cb(data):
    global initialised
    if (not initialised):
        if data.name=="stationkeeping":
            rospy.loginfo("Executing Station Keeping")
            ## geo pose to pose
            geoPoseToPose.geoPoseToPoseConverter()
            ## pose to route
            poseToRoute.poseToRouteConverter()
            pass
        elif data.name=="wayfinding":
            geoPathToPath.geoPathToPathConverter()
            pathToRoute.pathToRouteConverter()
            pass
        elif data.name=="perception":
            pass
        elif data.name=="navigation_course":
            print("DOING NAVIGATION")
            nav_task = NavigationTask()
            nav_task.startNavigation()
            pass
        elif data.name=="scan" or data.name=="dock":
            rospy.loginfo("On Task: docking")

            dm = DockMaster()

        elif data.name=="scan_and_dock":
            rospy.loginfo("On Task: scan and dock")

        initialised=True
        # do something

rospy.Subscriber(params['taskTopic'], Task,cb)
rospy.spin()
