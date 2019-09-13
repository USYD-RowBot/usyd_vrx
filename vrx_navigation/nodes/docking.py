#!/usr/bin/env python

import cv2
import numpy as np
from enum import Enum
import signal
import sys

import rospy
import actionlib
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image
from nav_msgs.msg import Odometry
from vrx_msgs.msg import *

# State machine for docking procedure
class DockSM:

  class DockState(Enum):
    DOCK_STANDBY  = 1     # Waiting for instruction
    DOCK_ENTER    = 2     # Entering the dock
    DOCK_HOLD     = 3     # Waiting inside the dock
    DOCK_EXIT     = 4     # Exiting the dock

  def __init__(self, hold_duration):
    self.state = self.DockState.DOCK_STANDBY # Initialise state machine state
    self.hold_time_target = 0           # Timer for HOLD state
    self.hold_duration = hold_duration  # Duration for HOLD state

  def getState(self):
    return self.state

  def beginDocking(self):
    if self.state is self.DockState.DOCK_STANDBY:
      self.state = self.DockState.DOCK_ENTER

  def hold(self):
    if self.state is self.DockState.DOCK_ENTER:
      # Reset hold timer and change state to HOLD
      self.hold_time_target = rospy.get_time() + self.hold_duration
      self.state = self.DockState.DOCK_HOLD

  def tryExitDock(self):
    exiting = False # Are we going to exit the dock now?

    if self.state is self.DockState.DOCK_HOLD:
      if rospy.get_time() > hold_time_target: # Check hold timer expired
        self.state = self.DockState.DOCK_EXIT
        exiting = True

    return exiting

  def finished(self):
    if self.state is self.DockState.DOCK_EXIT:
      self.state = self.DockState.DOCK_STANDBY

class Docker:

  def __init__(self):

    self.x_cam_error = 0 # Detected x error from image processing
    self.x_gps_error = 0 # Detected x error from gps readings

    self.vessel_pos  = [0, 0] # Current vessel x, y position
    self.initial_pos = [0, 0] # Position to return to when exiting dock

    self.dock_pos = [0, 0] # Position of desired docking bay
    self.dock_yaw = 0      # Yaw at which to enter dock

    self.getROSParams()

    self.dock_sm = DockSM(self.hold_duration) # Initialise docking state machine
    self.bridge  = CvBridge()                 # For converting ROS images to CV images

    self.pub_course = rospy.Publisher("/course_cmd", Course, queue_size=1)

    self.sub_img  = rospy.Subscriber(
      "/wamv/sensors/cameras/middle_right_camera/image_raw", Image, self.imageCb)
    self.sub_odom = rospy.Subscriber(
      "/wamv/sensors/position/p3d_wamv", Odometry, self.odomCb)

    self.server = actionlib.SimpleActionServer('docking', DockAction, self.execute, False)
    self.server.start()

  def getROSParams(self):
    self.hold_duration      = rospy.get_param("~hold_duration")
    self.goal_tolerance_pos = rospy.get_param("~goal_tolerance_pos")
    self.goal_tolerance_ang = rospy.get_param("~goal_tolerance_ang")

  def execute(self, goal):

    # Set goal of docking bay position and yaw
    self.dock_pos = [goal.dock_position.position.x, goal.dock_position.position.y]
    self.dock_yaw = goal.dock_yaw
    self.initial_pos = self.vessel_pos # Store initial vessel position for exiting

    self.dock_sm.beginDocking() # Tell state machine we are beginning

    rate = rospy.Rate(10) # 10hz loop
    while not rospy.is_shutdown:

      # ENTERING DOCK
      if self.dock_sm.getState() is self.dock_sm.DockState.DOCK_ENTER:

        # If close enough to docking bay position
        if self.reachedGoalPos(self.dock_pos):
          self.dock_sm.hold() # Tell state machine we will wait in dock bay
          self.server.publish_feedback(DockFeedback("Entering state HOLD"))

      # HOLDING POSITION IN DOCK
      elif self.dock_sm.getState() is self.dock_sm.DockState.DOCK_HOLD:

        if self.dock_sm.tryExitDock(): # Wait for duration of HOLD before exiting dock          
          self.server.publish_feedback(DockFeedback("Entering state EXIT"))

      # EXITING DOCK
      elif self.dock_sm.getState() is self.dock_sm.DockState.DOCK_EXIT:

        # If close enough to initial position outside dock
        if self.reachedGoalPos(self.initial_pos):         
          self.dock_sm.finished() # Tell state machine we have exited successfully
          self.server.publish_feedback(DockFeedback("Entering state STANDBY"))
          self.server.publish_feedback(DockFeedback("Docking successful!"))
          self.server.set_succeeded() # Return success from server
          return # Exit execute() function

      course_cmd = Course() # Generate course cmd for strafing
      course_cmd.station_yaw = self.dock_yaw
      course_cmd.keep_station = True
      course_cmd.yaw = self.getStrafeYaw() # Get strafe yaw based on state

      pub_course.publish(course_cmd) # Publish course command
      rate.sleep
    # end while loop

    self.server.set_aborted() # Abort if we get here somehow

  def reachedGoalPos(self, goal_pos):
    dist = np.sqrt((goal_pos[0]-self.vessel_pos[0])**2, (goal_pos[1]-self.vessel_pos[1])**2)
    return dist < goal_tolerance_pos
    
  def getStrafeYaw(self):
    if self.dock_sm.getState() is self.dock_sm.DockState.DOCK_ENTER:
      # Forward thrust
      pass
    elif self.dock_sm.getState() is self.dock_sm.DockState.DOCK_HOLD:
      # No forward or backward thrust
      pass
    elif self.dock_sm.getState() is self.dock_sm.DockState.DOCK_EXIT:
      # Backward thrust
      pass
    return 0

  def imageCb(self, ros_img):
    # If we aren't in the standby state
    if self.dock_sm.getState() is not self.dock_sm.DockState.DOCK_STANDBY:
      
      try: # Convert ROS image to CV image
        cv_img = self.bridge.imgmsg_to_cv2(ros_img, "bgr8")
      except CvBridgeError as e:
        print(e)
        return

      self.processImage(cv_img) # Pass converted image to processing algorithm

  def odomCb(self, odom):
    # Store current vessel position in odom frame
    self.vessel_pos[0] = odom.pose.pose.position.x
    self.vessel_pos[1] = odom.pose.pose.position.y

  def processImage(self, img):

    #test_imgs = ['0_dock_approach.png', '1_edge_dock_boundary.png', '2_halfway_cross_dock_boundary.png', '3_dock.png']
    #img = cv2.imread('/home/benjamin/Pictures/VRX/' + test_imgs[0], cv2.IMREAD_GRAYSCALE)

    new_width = 300 # Scale image down so width is 300 pixels
    aspect = float(img.shape[0]) / float(img.shape[1])
    scaled_img = cv2.resize(img, (new_width, int(new_width * aspect)))
 
    # Generate Gabor filter kernel and filter the scaled image
    g_kernel  = cv2.getGaborKernel((3,3), 16, np.radians(0), 5, 0.5, 0, ktype=cv2.CV_32F)
    gabor_img = cv2.filter2D(scaled_img, -1, g_kernel)

    bordered_img = cv2.copyMakeBorder(gabor_img, 1, 1, 1, 1, # Add white border
      cv2.BORDER_CONSTANT, value=(255, 255, 255))

    params = cv2.SimpleBlobDetector_Params() # Set blob detector parameters
    #params.filterByArea = True  # Filter by area.
    #params.maxArea = 200
    
    detector = cv2.SimpleBlobDetector_create(params) # Create blob detector
     
    keypoints = detector.detect(bordered_img) # Detect blobs

    # Only record x error if exactly 1 blob is detected.
    if len(keypoints) is 1:
      # Calculate error in x direction, in pixels.
      self.x_cam_error = keypoints[0].pt[0] - np.round(300/2)
      print(self.x_cam_error)

    elif len(keypoints) is 0:
      print("docking: Error: no blob detected.")
    else:
      print("docking: Error: more than 1 blob detected.")

    #cv2.imshow('gabor filtered image', gabor_img) # Show filtered image.

    # Draw detected blobs on gabor filtered image as red circles.
    blob_img = cv2.drawKeypoints(bordered_img, keypoints, np.array([]), (0,0,255), 
      cv2.DRAW_MATCHES_FLAGS_DRAW_RICH_KEYPOINTS)
    cv2.imshow("detected blobs", blob_img)
    cv2.waitKey(1)

    #h, w     = g_kernel.shape[:2] # Visualise the gabor kernel
    #g_kernel = cv2.resize(g_kernel, (20*w, 20*h), interpolation=cv2.INTER_CUBIC)
    #cv2.imshow('gabor kernel', g_kernel)
    
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