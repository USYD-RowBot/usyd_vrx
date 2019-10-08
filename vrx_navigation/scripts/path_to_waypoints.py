#!/usr/bin/env python
import rospy
from nav_msgs.msg import Path
from vrx_msgs.msg import Waypoint,WaypointRoute


class WaypointConverter:
    def __init__(self):
        self.pub=rospy.Publisher("waypoints_cmd",WaypointRoute,queue_size = 10)
        rospy.Subscriber("wamv/waypoints",Path,self.pathCallback)


    def pathCallback(self,path_msg):
        #WaypointRoute msg
        wpr_msg = WaypointRoute()
        speed = rospy.get_param("speed", 1)
        wpr_msg.speed = speed
        count = 0;

        # for pose_stamped in path_msg.poses:
        #     if count%10 ==0:
        #
        #         waypoint = Waypoint()
        #         if count == 0:
        #             waypoint.nav_type = Waypoint.NAV_STATION
        #             waypoint.station_duration = 3
        #         else:
        #             waypoint.nav_type = Waypoint.NAV_WAYPOINT
        #         waypoint.pose = pose_stamped.pose
        #         wpr_msg.waypoints.append(waypoint)
        #     count = count +1
        waypoint = Waypoint()
        waypoint.nav_type = Waypoint.NAV_STATION
        waypoint.pose = path_msg.poses[len(path_msg.poses)-1].pose
        wpr_msg.waypoints.append(waypoint)
        self.pub.publish(wpr_msg)




if __name__=="__main__":
    rospy.init_node("path_to_waypoints")
    wt = WaypointConverter()
    rospy.spin()
