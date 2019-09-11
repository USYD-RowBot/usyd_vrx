#!/usr/bin/env python
import numpy as np
import cv2
import rospy
from enum import Enum
import signal
import sys

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

  def beginDocking():
    if self.state is self.DockState.DOCK_STANDBY:
      self.state = self.DockState.DOCK_ENTER

  def hold():
    if self.state is self.DockState.DOCK_ENTER:
      # Reset hold timer and change state to HOLD
      self.hold_time_target = rospy.get_time() + self.hold_duration
      self.state = self.DockState.DOCK_HOLD

  def exitDock():
    if self.state is self.DockState.DOCK_HOLD:
      if rospy.get_time() > hold_time_target: # Check hold timer expired
        self.state = self.DockState.DOCK_EXIT

  def finished():
    if self.state is self.DockState.DOCK_EXIT:
      self.state = self.DockState.DOCK_STANDBY

def signal_handler(sig, frame):
  cv2.destroyAllWindows()
  sys.exit(0)

def main():

  signal.signal(signal.SIGINT, signal_handler)

  hold_duration = 15 # Number of seconds to hold position inside dock
  dock_sm = DockSM(hold_duration) # Initialise docking state machine

  #src = cv2.imread('/home/benjamin/Pictures/VRX/0_dock_approach.png', cv2.IMREAD_GRAYSCALE)
  #src = cv2.imread('/home/benjamin/Pictures/VRX/1_edge_dock_boundary.png', cv2.IMREAD_GRAYSCALE)
  #src = cv2.imread('/home/benjamin/Pictures/VRX/2_halfway_cross_dock_boundary.png', cv2.IMREAD_GRAYSCALE)
  src = cv2.imread('/home/benjamin/Pictures/VRX/3_dock.png', cv2.IMREAD_GRAYSCALE)

  # Introduce consistency in width
  const_width = 300
  aspect = float(src.shape[0]) / float(src.shape[1])
  src = cv2.resize(src, (const_width, int(const_width * aspect)))

  # Filter that can isolate poster shapes, with const width 300
  #g_kernel = cv2.getGaborKernel((3,3), 16, np.radians(0), 5, 0.5, 0, ktype=cv2.CV_32F)

  # Closest to verticals so far, with const width 150
  #g_kernel = cv2.getGaborKernel((16,16), 7, np.radians(0), 5.9, 0.3, 3.14, ktype=cv2.CV_32F)

  # Apply gabor kernel to identify vertical edges
  g_kernel = cv2.getGaborKernel((3,3), 16, np.radians(0), 5, 0.5, 0, ktype=cv2.CV_32F)
  #g_kernel = cv2.getGaborKernel((9,9), 8, np.radians(0), 5, 0.5, 0, ktype=cv2.CV_32F)
  gabor    = cv2.filter2D(src, -1, g_kernel)

  # Visualise the gabor kernel
  h, w     = g_kernel.shape[:2]
  g_kernel = cv2.resize(g_kernel, (20*w, 20*h), interpolation=cv2.INTER_CUBIC)

  gaborder = cv2.copyMakeBorder(gabor, 1, 1, 1, 1, cv2.BORDER_CONSTANT, value=(255, 255, 255))

  # Setup SimpleBlobDetector parameters.
  params = cv2.SimpleBlobDetector_Params()
  
  # Filter by Area.
  #params.filterByArea = True
  #params.maxArea = 200 
  
  # Set up the detector with default parameters.
  detector = cv2.SimpleBlobDetector_create(params)

  # Detect blobs.
  keypoints = detector.detect(gaborder)

  print("x: %f" % keypoints[0].pt[0])
  print("y: %f" % keypoints[0].pt[1])

  # Draw detected blobs as red circles.
  im_with_keypoints = cv2.drawKeypoints(gaborder, keypoints, np.array([]), (0,0,255), 
    cv2.DRAW_MATCHES_FLAGS_DRAW_RICH_KEYPOINTS)

  # Show keypoints
  cv2.imshow('src', src)
  cv2.imshow('gabor kernel', g_kernel)
  cv2.imshow('gabor', gabor) # gabor is just black
  cv2.imshow("Keypoints", im_with_keypoints)

  while(True):
    cv2.waitKey(1)

if __name__ == "__main__":
  main()