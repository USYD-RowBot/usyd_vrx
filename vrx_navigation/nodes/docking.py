#!/usr/bin/env python

import cv2
import numpy as np
from enum import Enum
import signal
import sys

import rospy
import tf
import actionlib
from cv_bridge import CvBridge, CvBridgeError
from vrx_msgs.srv import ClassifyPlacard,ClassifyPlacardResponse
from sensor_msgs.msg import Image
from nav_msgs.msg import Odometry
from vrx_msgs.msg import *
from std_srvs.srv import SetBool

import sys
import rospkg
rospack = rospkg.RosPack()
sys.path.insert(1, rospack.get_path('vrx_vision')+"/scripts")
from placard_classifier import PlacardClassifier

# State machine for docking procedure
class DockSM:

  class DockState(Enum):
    DOCK_STANDBY  = 1     # Waiting for instruction
    DOCK_ENTER    = 2     # Entering the dock
    DOCK_HOLD     = 3     # Waiting inside the dock
    DOCK_EXIT     = 4     # Exiting the dock

  def __init__(self, hold_duration, entry_duration):
    self.state = self.DockState.DOCK_STANDBY # Initialise state machine state
    self.hold_time_target = 0           # Timer for HOLD state
    self.hold_duration = hold_duration  # Duration for HOLD state

    self.entry_time_target = 0
    self.entry_duration = entry_duration  # Duration for ENTER

  def getState(self):
    return self.state

  def beginDocking(self):
    if self.state is self.DockState.DOCK_STANDBY:
      self.state = self.DockState.DOCK_ENTER
      self.entry_time_target = rospy.get_time() + self.entry_duration

  def hold(self):
    if self.state is self.DockState.DOCK_ENTER:
      # Reset hold timer and change state to HOLD
      self.hold_time_target = rospy.get_time() + self.hold_duration
      self.state = self.DockState.DOCK_HOLD

  def tryExitDock(self):
    exiting = False # Are we going to exit the dock now?

    if self.state is self.DockState.DOCK_HOLD:
      if rospy.get_time() > self.hold_time_target: # Check hold timer expired
        self.state = self.DockState.DOCK_EXIT
        self.entry_time_target = rospy.get_time() + self.entry_duration
        exiting = True

    return exiting

  def finished(self):
    if self.state is self.DockState.DOCK_EXIT:
      self.state = self.DockState.DOCK_STANDBY

  def getEntryPercentage(self):
    if self.state is self.DockState.DOCK_ENTER or self.state is self.DockState.DOCK_EXIT:
      if rospy.get_time() <= self.entry_time_target:
        return 1.0 - float(self.entry_time_target - rospy.get_time())/self.entry_duration
      else:
        return 1.0
    elif self.state is self.DockState.DOCK_HOLD:
      return 1.0


class Docker:

  def __init__(self):

    self.x_cam_error = 0 # Detected x error from image processing
    self.x_gps_error = 0 # Detected x error from gps readings

    self.vessel_pos  = [0, 0] # Current vessel x, y position
    self.vessel_yaw  = 0
    self.initial_pos = [0, 0] # Position to return to when exiting dock

    self.bay_pos = [0, 0] # Position of desired docking bay
    self.dock_yaw = 0      # Yaw at which to enter dock
    
    self.placard_classifier = PlacardClassifier()
    self.getROSParams()

    self.dest_pos = [] # Interpolate dock/entry exit into wps
    for _ in range(self.n_dests):
      self.dest_pos.append([0, 0])

    # Initialise docking state machine
    self.dock_sm = DockSM(self.hold_duration, self.entry_duration) 
    self.bridge  = CvBridge()                 # For converting ROS images to CV images

    rospy.loginfo("docking: Waiting for vessel odometry.")
    self.odomCb(rospy.wait_for_message("/odom", Odometry)) # Wait to fill vessel_pos
    rospy.loginfo("docking: Vessel odometry received.")

    self.pub_course = rospy.Publisher("/course_cmd", Course, queue_size=1)

    #self.sub_img  = rospy.Subscriber(
    #  "/wamv/sensors/cameras/middle_camera/image_raw", Image, self.imageCb)
    self.sub_odom = rospy.Subscriber("/odom", Odometry, self.odomCb)

    hfov     = 1.3962634
    cam_x_px = 1280
    self.focal_length = (float(cam_x_px)/2)/np.tan(float(hfov)/2)

    self.server = actionlib.SimpleActionServer('docking', DockAction, self.execute, False)
    self.server.start()

  def getROSParams(self):
    self.hold_duration      = rospy.get_param("~hold_duration")
    self.goal_tolerance_pos = rospy.get_param("~goal_tolerance_pos")
    self.goal_tolerance_ang = rospy.get_param("~goal_tolerance_ang")
    #self.pix_threshold      = rospy.get_param("~pix_threshold")
    self.pix_offset         = rospy.get_param("~pix_offset")
    self.entry_duration     = rospy.get_param("~entry_duration")
    self.n_dests            = rospy.get_param("~n_entry_wps")

  def execute(self, goal):

    rospy.loginfo("docking: Goal received.")

    # Disable waypoint follower to gain control
    if not self.enableWaypointFollower(False):
      self.server.set_aborted() # Return failure from server
      return # Early exit the execute function if service failed

    # Set goal of docking bay position and yaw
    self.bay_pos = [goal.bay_position.position.x, goal.bay_position.position.y]
    self.dock_yaw = goal.dock_yaw
    self.initial_pos = list(self.vessel_pos) # Store initial vessel position for exiting

    # Call placard classifier and get pixel x position of centre of placard symbol
    self.setPlacardXError(goal.dock_position)

    self.dock_sm.beginDocking() # Tell state machine we are beginning
    rospy.loginfo("Entering state ENTER")

    rate = rospy.Rate(10) # 10hz loop
    while not rospy.is_shutdown():

      # ENTERING DOCK
      if self.dock_sm.getState() is self.dock_sm.DockState.DOCK_ENTER:

        # If close enough to docking bay position
        if self.reachedGoalPos(self.bay_pos):
          self.dock_sm.hold() # Tell state machine we will wait in dock bay
          self.server.publish_feedback(DockFeedback("Entering state HOLD"))
          rospy.loginfo("Entering state HOLD")

      # HOLDING POSITION IN DOCK
      elif self.dock_sm.getState() is self.dock_sm.DockState.DOCK_HOLD:

        if self.dock_sm.tryExitDock(): # Wait for duration of HOLD before exiting dock
          self.server.publish_feedback(DockFeedback("Entering state EXIT"))
          rospy.loginfo("Entering state EXIT")

      # EXITING DOCK
      elif self.dock_sm.getState() is self.dock_sm.DockState.DOCK_EXIT:

        # If close enough to initial position outside dock
        if self.reachedGoalPos(self.initial_pos):
          self.dock_sm.finished() # Tell state machine we have exited successfully
          self.server.publish_feedback(DockFeedback("Entering state STANDBY"))
          self.server.publish_feedback(DockFeedback("Docking successful!"))
          self.server.set_succeeded() # Return success from server
          rospy.loginfo("docking: Finished docking procedure!")

          self.enableWaypointFollower(True) # Re-enable waypoint follower to transfer control
          cv2.destroyAllWindows() # Destroy opencv windows

          return # Exit execute() function

      course_cmd = Course() # Generate course cmd for strafing
      course_cmd.station_yaw = self.dock_yaw
      course_cmd.keep_station = True

      station_dist_x, station_dist_y = self.getStationDistance()
      course_cmd.station_dist_x = station_dist_x
      course_cmd.station_dist_y = station_dist_y

      self.pub_course.publish(course_cmd) # Publish course command
      rate.sleep()
    # end while loop

    self.server.set_aborted()         # Abort if we get here somehow
    self.enableWaypointFollower(True) # Re-enable waypoint follower to transfer control

  def setPlacardXError(self, dock_pose):
    label = ""
    while label == "" and not rospy.is_shutdown(): # Get one single camera image
      ros_img = rospy.wait_for_message("/wamv/sensors/cameras/middle_camera/image_raw", Image)
      cv_img = self.bridge.imgmsg_to_cv2(ros_img, "bgr8")
      label, conf, cX = self.placard_classifier.classifyPlacard(cv_img)
      cv2.imwrite("/home/nickyx/Pictures/cv_img.png", cv_img)

    x = 640 - cX + self.pix_offset # x position of placard symbol relative to centre of img
    theta = np.arctan(float(x)/self.focal_length) 

    # Distance to dock centre
    d = np.sqrt((dock_pose.position.x - self.vessel_pos[0])**2 + (dock_pose.position.y - self.vessel_pos[1])**2)
    X = d*np.tan(theta) # Vessel's horizontal offset from dock in metres

    rospy.logdebug("cX is %d px." % cX)
    rospy.logdebug("x is %f px."  % x)
    rospy.logdebug("X is %f metres." % X)

    error_vec = [0, X] # Modification to bay_pos in boat frame

    # Transform from boat frame to world frame
    error_x = error_vec[0]*np.cos(-self.vessel_yaw) + error_vec[1]*np.sin(-self.vessel_yaw)
    error_y = error_vec[1]*np.cos(-self.vessel_yaw) - error_vec[0]*np.sin(-self.vessel_yaw)

    rospy.logdebug("bay_pos is %f, %f." % (self.bay_pos[0], self.bay_pos[1]))

    self.bay_pos[0] += error_x # Adjust bay_pos according to the x error
    self.bay_pos[1] += error_y

    self.initial_pos[0] += error_x # Also adjust initial position for smooth exit
    self.initial_pos[1] += error_y

    rospy.logdebug("new bay_pos is %f, %f.\n" % (self.bay_pos[0], self.bay_pos[1]))

    init_to_bay_vec = [self.bay_pos[0] - self.initial_pos[0], self.bay_pos[1] - self.initial_pos[1]]
    self.dest_pos[0][0] = self.initial_pos[0] # Set first interpolated pos to initial pos
    self.dest_pos[0][1] = self.initial_pos[1]

    for i in range(self.n_dests-1):
      self.dest_pos[i+1][0] = self.initial_pos[0] + init_to_bay_vec[0]*float(i+1)/(self.n_dests-1)
      self.dest_pos[i+1][1] = self.initial_pos[1] + init_to_bay_vec[1]*float(i+1)/(self.n_dests-1)
  
  def getStationDistance(self):
    station_dist_x = 0
    station_dist_y = 0

    if self.dock_sm.getState() is not self.dock_sm.DockState.DOCK_STANDBY:
      entry_percent = self.dock_sm.getEntryPercentage() # Interpolate entry/exit
      dest_index = int(np.floor(entry_percent*self.n_dests))

      if dest_index > self.n_dests - 1:
        dest_index = self.n_dests - 1

      if self.dock_sm.getState() is self.dock_sm.DockState.DOCK_EXIT:
        dest_index = int(self.n_dests - 1 - dest_index) # Invert index to exit

      station_dist_x = self.dest_pos[dest_index][0] - self.vessel_pos[0]
      station_dist_y = self.dest_pos[dest_index][1] - self.vessel_pos[1]

    else:
      station_dist_x = self.initial_pos[0] - self.vessel_pos[0]
      station_dist_y = self.initial_pos[1] - self.vessel_pos[1]

    return station_dist_x, station_dist_y

  def enableWaypointFollower(self, do_enable):
    try: # Attempt to enable/disable waypoint follower
        wp_follower_enable = rospy.ServiceProxy(
          '/wamv/wamv_waypoint_follower/enable_follower', SetBool)
        wp_follower_enable(do_enable)

    except rospy.ServiceException, e:
        rospy.loginfo("docking: Unable to transfer control to/from waypoint follower: %s"%e)
        return False # Server failed

    rospy.loginfo("docking: Transferred control to/from waypoint follower.")
    return True # Succesfully contacted server

  def reachedGoalPos(self, goal_pos):
    dist = np.sqrt((goal_pos[0]-self.vessel_pos[0])**2 + (goal_pos[1]-self.vessel_pos[1])**2)
    return dist < self.goal_tolerance_pos

  def odomCb(self, odom):
    # Store current vessel position in odom frame
    self.vessel_pos[0] = odom.pose.pose.position.x
    self.vessel_pos[1] = odom.pose.pose.position.y

    # Get vessel yaw
    quaternion = (odom.pose.pose.orientation.x, odom.pose.pose.orientation.y,
      odom.pose.pose.orientation.z, odom.pose.pose.orientation.w)
    euler = tf.transformations.euler_from_quaternion(quaternion)
    self.vessel_yaw = euler[2]

  '''def calculatePlacardSymbolX(self, ros_img):
      cv_img = self.bridge.imgmsg_to_cv2(ros_img, "bgr8")
      label,conf,error = self.placard_classifier.classifyPlacard(cv_img)
      self.x_cam_error = error - 640 + self.pix_offset'''

  '''def getStrafeYaw(self):
    yaw = 0
    k = 0.2
    if self.dock_sm.getState() is self.dock_sm.DockState.DOCK_ENTER:

      #yaw = self.dock_yaw - ((np.pi/2)/self.pix_threshold )*self.x_cam_error
      if abs(self.x_cam_error) > self.pix_threshold:
        yaw = self.dock_yaw - np.sign(self.x_cam_error)*(np.pi/2)
      else:
        yaw = self.dock_yaw # Move forwards

    elif self.dock_sm.getState() is self.dock_sm.DockState.DOCK_HOLD:
        yaw = self.dock_yaw - np.sign(self.x_cam_error)*np.pi/2

    elif self.dock_sm.getState() is self.dock_sm.DockState.DOCK_EXIT:

      #yaw = self.dock_yaw-np.pi - ((np.pi/2)/self.pix_threshold )*self.x_cam_error

      if abs(self.x_cam_error) > self.pix_threshold:
        yaw = self.dock_yaw - np.sign(self.x_cam_error)*np.pi/2
      else:
        yaw = (self.dock_yaw - np.pi) # Move backwards

    # Limit yaw from -PI to PI
    if (abs(yaw) > np.pi):
      yaw = -np.sign(yaw)*(2*np.pi - abs(yaw))

    return yaw'''

def signalHandler(sig, frame):
  cv2.destroyAllWindows()
  rospy.signal_shutdown("Shutting down docking node on Ctrl+C.")
  sys.exit(0)

def main():
  rospy.init_node('docking_server')

  signal.signal(signal.SIGINT, signalHandler) # Register signal handler

  docker = Docker() # Docker object handles docking procedure

  rospy.spin()

if __name__ == "__main__":
  main()
