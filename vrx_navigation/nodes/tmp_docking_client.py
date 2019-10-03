#!/usr/bin/env python

import sys
import rospy
import actionlib
import vrx_msgs.msg
from geometry_msgs.msg import Pose

def dock_client():
  client = actionlib.SimpleActionClient('/wamv/docking', vrx_msgs.msg.DockAction)

  # Waits until the action server has started up and started listening for goals.
  client.wait_for_server()

  pose = Pose()
  pose.position.x = 9.071014   # Inside docking bay
  pose.position.y = -12.724601
  pose.orientation.w = 1

  yaw = -2.941593654

  # Creates a goal to send to the action server.
  goal = vrx_msgs.msg.DockGoal(pose, yaw)  

  # Sends the goal to the action server.
  client.send_goal(goal)

  # Waits for the server to finish performing the action.
  client.wait_for_result()

  # Prints out the result of executing the action
  return client.get_result()  # A DockResult

if __name__ == '__main__':
  try:
    # Initializes a rospy node so that the SimpleActionClient can
    # publish and subscribe over ROS.
    rospy.init_node('tmp_docking_client_py')
    result = dock_client()
    #print("Result:", ', '.join([str(n) for n in result.sequence]))
  except rospy.ROSInterruptException:
    print("program interrupted before completion")