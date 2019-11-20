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
      if rospy.get_time() > self.hold_time_target: # Check hold timer expired
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
    self.vessel_yaw  = 0
    self.initial_pos = [0, 0] # Position to return to when exiting dock

    self.dock_pos = [0, 0] # Position of desired docking bay
    self.dock_yaw = 0      # Yaw at which to enter dock
    self.prev_x_error = 0
    self.getROSParams()

    self.dock_sm = DockSM(self.hold_duration) # Initialise docking state machine
    self.bridge  = CvBridge()                 # For converting ROS images to CV images

    rospy.loginfo("docking: Waiting for vessel odometry.")
    self.odomCb(rospy.wait_for_message("/odom", Odometry)) # Wait to fill vessel_pos
    rospy.loginfo("docking: Vessel odometry received.")

    self.pub_course = rospy.Publisher("/course_cmd", Course, queue_size=1)

    self.sub_img  = rospy.Subscriber(
      "/wamv/sensors/cameras/middle_camera/image_raw", Image, self.imageCb)
    self.sub_odom = rospy.Subscriber(
      "/odom", Odometry, self.odomCb)

    rospy.wait_for_service('/wamv/wamv/classify_placard')

    self.server = actionlib.SimpleActionServer('docking', DockAction, self.execute, False)
    self.server.start()

  def getROSParams(self):
    self.hold_duration      = rospy.get_param("~hold_duration")
    self.goal_tolerance_pos = rospy.get_param("~goal_tolerance_pos")
    self.goal_tolerance_ang = rospy.get_param("~goal_tolerance_ang")
    self.pix_threshold      = rospy.get_param("~pix_threshold")
    self.pix_offset         = rospy.get_param("~pix_offset")
    self.default_thrust     = rospy.get_param("~default_thrust")

  def execute(self, goal):

    rospy.loginfo("docking: Goal received.")

    # Disable waypoint follower to gain control
    if not self.enableWaypointFollower(False):
      self.server.set_aborted() # Return failure from server
      return # Early exit the execute function if service failed

    # Set goal of docking bay position and yaw
    self.dock_pos = [goal.dock_position.position.x, goal.dock_position.position.y]
    self.dock_yaw = goal.dock_yaw
    self.initial_pos = list(self.vessel_pos) # Store initial vessel position for exiting

    self.dock_sm.beginDocking() # Tell state machine we are beginning

    rate = rospy.Rate(10) # 10hz loop
    while not rospy.is_shutdown():

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
          rospy.loginfo("docking: Finished docking procedure!")

          self.enableWaypointFollower(True) # Re-enable waypoint follower to transfer control
          cv2.destroyAllWindows() # Destroy opencv windows

          return # Exit execute() function

      course_cmd = Course() # Generate course cmd for strafing
      course_cmd.station_yaw = self.dock_yaw
      course_cmd.keep_station = True
      course_cmd.yaw = self.getStrafeYaw() # Get strafe yaw based on state
      k = 0.01
      min_thrust = 0.1
      if abs(self.x_cam_error) > self.pix_threshold and self.x_cam_error>0:
          thrust = abs(float(abs(self.x_cam_error)-self.pix_threshold)/float(self.pix_threshold)*self.default_thrust)
          thrust = thrust -  float(abs(self.x_cam_error-self.prev_x_error))*k
          if thrust < min_thrust:
              thrust = min_thrust
      else:
          thrust = self.default_thrust

      self.prev_x_error = self.x_cam_error
      if thrust < 0:
          thrust = 0
      course_cmd.speed = thrust
      print(course_cmd)

      self.pub_course.publish(course_cmd) # Publish course command
      rate.sleep()
    # end while loop

    self.server.set_aborted()         # Abort if we get here somehow
    self.enableWaypointFollower(True) # Re-enable waypoint follower to transfer control

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

  def getStrafeYaw(self):
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

    return yaw

  def imageCb(self, ros_img):
    # If we aren't in the standby state
    if self.dock_sm.getState() is not self.dock_sm.DockState.DOCK_STANDBY:

      self.calculatePlacardSymbolX(ros_img)
      '''try: # Convert ROS image to CV image
        cv_img = self.bridge.imgmsg_to_cv2(ros_img, "bgr8")
      except CvBridgeError as e:
        rospy.logdebug(e)
        return

      self.processImage(cv_img) # Pass converted image to processing algorithm'''

  def calculatePlacardSymbolX(self, ros_img):
    try:
      classifyPlacard = rospy.ServiceProxy('wamv/classify_placard', ClassifyPlacard)
      res = classifyPlacard(ros_img)
      self.x_cam_error = res.centre_x - 640 + self.pix_offset

    except rospy.ServiceException, e:
      rospy.loginfo("Service call to /wamv/classify_placard failed: %s"%e)

  def odomCb(self, odom):
    # Store current vessel position in odom frame
    self.vessel_pos[0] = odom.pose.pose.position.x
    self.vessel_pos[1] = odom.pose.pose.position.y

    # Get vessel yaw
    quaternion = (odom.pose.pose.orientation.x, odom.pose.pose.orientation.y,
      odom.pose.pose.orientation.z, odom.pose.pose.orientation.w)
    euler = tf.transformations.euler_from_quaternion(quaternion)
    self.vessel_yaw = euler[2]

  '''def processImage(self, img):

    new_width = 300 # Scale image down so width is 300 pixels
    aspect = float(img.shape[0]) / float(img.shape[1])
    scaled_img = cv2.resize(img, (new_width, int(new_width * aspect)))

    # Generate Gabor filter kernel and filter the scaled image
    g_kernel  = cv2.getGaborKernel((3,3), 16, np.radians(0), 5, 0.5, 0, ktype=cv2.CV_32F)
    gabor_img = cv2.filter2D(scaled_img, -1, g_kernel)

    bordered_img = cv2.copyMakeBorder(gabor_img, 1, 1, 1, 1, # Add white border
      cv2.BORDER_CONSTANT, value=(255, 255, 255))

    params = cv2.SimpleBlobDetector_Params() # Set blob detector parameters
    params.filterByArea = True  # Filter by area.
    params.minArea = 50

    detector = cv2.SimpleBlobDetector_create(params) # Create blob detector

    keypoints = detector.detect(bordered_img) # Detect blobs

    # Only record x error if exactly 1 blob is detected.
    if len(keypoints) is 1:
      # Calculate error in x direction, in pixels.
      self.x_cam_error = keypoints[0].pt[0] - np.round(300/2) + self.pix_offset

    elif len(keypoints) is 0:
      rospy.logdebug("docking: Error: no blob detected.")
    else:
      rospy.logdebug("docking: Error: more than 1 blob detected.")

    # Draw detected blobs on gabor filtered image as red circles.
    blob_img = cv2.drawKeypoints(bordered_img, keypoints, np.array([]), (0,0,255),
      cv2.DRAW_MATCHES_FLAGS_DRAW_RICH_KEYPOINTS)
    cv2.imshow("detected blobs", blob_img)
    cv2.waitKey(1)'''

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
