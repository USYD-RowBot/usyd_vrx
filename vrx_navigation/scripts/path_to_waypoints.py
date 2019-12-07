#!/usr/bin/env python
import rospy
import math
import tf
from nav_msgs.msg import Path
from vrx_msgs.msg import Waypoint,WaypointRoute
from visualization_msgs.msg import Marker

class WaypointConverter:
    def __init__(self):
        self.wp_cmd_pub = rospy.Publisher(
            "waypoints_cmd", WaypointRoute, queue_size=10)
        self.vis_pub    = rospy.Publisher(
            "nav_marker_global_planner", Marker, queue_size=1)

        self.markers        = rospy.get_param("~markers", 1)
        self.speed          = rospy.get_param("~default_speed", 1)
        self.thinning_value = rospy.get_param("~thinning_value", 1)

        self.marker_id = 0

        rospy.Subscriber("wamv/waypoints", Path, self.pathCallback)

    def getStationMsg(self, pose1, pose2, dest_pose, duration, set_yaw):
        # Calculate angle vessel needs to be at
        if set_yaw:
            delta_x = pose2.position.x - pose1.position.x
            delta_y = pose2.position.y - pose1.position.y
            yaw = math.atan2(delta_y, delta_x)
            quat = tf.transformations.quaternion_from_euler(0, 0, yaw)

        wp_msg = Waypoint() # Waypoint msg for initial station
        wp_msg.nav_type = Waypoint.NAV_STATION
        wp_msg.station_duration = duration
        wp_msg.pose = dest_pose
        if set_yaw:
            wp_msg.pose.orientation.x = quat[0]
            wp_msg.pose.orientation.y = quat[1]
            wp_msg.pose.orientation.z = quat[2]
            wp_msg.pose.orientation.w = quat[3]

        return wp_msg

    def pubMarker(self, pose):
        if self.markers:
            marker = Marker()
            marker.header.frame_id = "odom"
            marker.header.stamp = rospy.Time.now()
            marker.ns = "wps"
            marker.id = self.marker_id
            marker.type = Marker.SPHERE
            marker.action = Marker.ADD
            marker.pose = pose
            marker.scale.x = 1
            marker.scale.y = 1
            marker.scale.z = 1
            marker.color.a = 1.0
            marker.color.r = 0.0
            marker.color.g = 1.0
            marker.color.b = 0.0
            marker.lifetime = rospy.Duration(60)
            self.vis_pub.publish(marker)
            self.marker_id += 1

    def pathCallback(self, path_msg):
        if len(path_msg.poses) == 0:
            return

        wp_route_msg = WaypointRoute() # WaypointRoute msg
        wp_route_msg.speed = self.speed
        count = 0

        first_wp_msg = self.getStationMsg( # Get first station
            path_msg.poses[0].pose, path_msg.poses[10].pose,
            path_msg.poses[0].pose, 2.50, True)
        wp_route_msg.waypoints.append(first_wp_msg)
        self.pubMarker(first_wp_msg.pose)

        for pose_stamped in path_msg.poses:
            count += 1 # Put at start to avoid setting the immediate waypoint

            if count % self.thinning_value == 0:
                wp_msg = Waypoint() # Waypoint msg
                wp_msg.nav_type = Waypoint.NAV_WAYPOINT
                wp_msg.pose = pose_stamped.pose
                self.pubMarker(wp_msg.pose)
                wp_route_msg.waypoints.append(wp_msg) # Append wp to route

        half_thin_value = round(self.thinning_value / 2)

        # Remove current last waypoint if too close to actual last waypoint
        if (len(path_msg.poses) % self.thinning_value) < half_thin_value:
            wp_route_msg.waypoints.pop()

        last_wp_msg = self.getStationMsg( # Get last station
            path_msg.poses[-11].pose, path_msg.poses[-1].pose,
            path_msg.poses[-1].pose, -1, False)
        wp_route_msg.waypoints.append(last_wp_msg)
        self.pubMarker(last_wp_msg.pose)

        self.wp_cmd_pub.publish(wp_route_msg)

if __name__=="__main__":
    rospy.init_node("path_to_waypoints")
    _wc = WaypointConverter()
    rospy.spin()
