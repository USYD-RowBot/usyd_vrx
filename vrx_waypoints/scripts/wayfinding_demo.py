#!/usr/bin/env python

import rospy
from vrx_msgs.msg import *

def main():
  rospy.init_node('wayfinding_demo')
  
  pub_wps = rospy.Publisher("/waypoints_cmd", WaypointRoute, queue_size=1, latch=True)

  coords = [[85.5, 79.5], [40, 62], [82, 47], [60, 103], [116, 94]]

  wp_route = WaypointRoute()
  wp_route.speed = 2.0

  wps = []

  for coord in coords:
    wp = Waypoint()
    wp.nav_type = 0
    wp.pose.position.x = coord[0]
    wp.pose.position.y = coord[1]
    wps.append(wp)

  wp_route.waypoints = wps

  pub_wps.publish(wp_route)

  rospy.spin()

if __name__ == "__main__":
  main()