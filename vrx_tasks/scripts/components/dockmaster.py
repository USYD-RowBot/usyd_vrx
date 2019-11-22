#!/usr/bin/env python

# MISSION PLANNER FOR DOCKING WEWW

import rospy
import actionlib
import tf
from tf.transformations import quaternion_from_euler, euler_from_quaternion
import numpy as np
import math
from copy import deepcopy

from std_msgs.msg import Empty, String
from vrx_msgs.msg import *
from geometry_msgs.msg import Pose, Quaternion, PoseStamped
from nav_msgs.msg import Odometry
from vrx_msgs.srv import *
from sensor_msgs.msg import Image
from vrx_gazebo.msg import Task
from visualization_msgs.msg import Marker
from mission_base import Mission, quatToEuler ,eulerToQuat
from placard_classifier import PlacardClassifier
from cv_bridge import CvBridge, CvBridgeError
import cv2
# MAKE SURE TO CHANGE NAMESPACE OF /ODOM IN LAUNCH FILE




class DockMaster(Mission):

  def __init__(self,placard_symbol = None):
    rospy.loginfo("INitalizing MIssion base")
    Mission.__init__( self )
    self.bridge = CvBridge()
    self.logDock("Initialising dock master.")
    self.placard_symbol = placard_symbol
    self.placardClassifier = PlacardClassifier()
    self.initMission()
    self.executePlan()




  def initMission(self):
    self.do_scan = False   # Scan the sequence on the buoy?
    self.n_onspot_wps = 4  # Number of waypoints constituting spin on spot
    self.n_circle_wps = 4  # Number of waypoints constituting circling an object
    self.general_speed = 2 # Circling speed

    self.scan_radius    = 10 # Radius at which to circle scan buoy
    self.dock_radius    = 25 # Radius at which to circle dock
    self.explore_radius = 75

    self.align_dist   = 15 #  Distance from center of dock to align position
    self.bay_dist     = 5  # Distance from center of dock to center of bay
    self.explore_dist = 75
    self.current_pose = Pose()
    self.tf_listener = tf.TransformListener()
    self.route_pub = rospy.Publisher("/waypoints_cmd", WaypointRoute, queue_size=1, latch=True)

    #rospy.get_param("~hold_duration")

    # Wait for any required services to be active
    #rospy.wait_for_service('scan_buoy')

    #rospy.loginfo("Waiting of classif_placard service")
    #rospy.wait_for_service('/wamv/wamv/classify_placard')

    #ready = False # Wait until competition is in Ready state.
    #while not ready:
     # task_msg = rospy.wait_for_message("/vrx/task/info", Task)
     # if task_msg.state == "ready":
        #ready = True

  def executePlan(self):
    self.logDock("Executing mission plan.")
    self.logDock("Sleeping 5 seconds")
    rospy.sleep(5)
    dock = None
    if self.placard_symbol is None:
        #Search for dock
        rospy.loginfo("Exporing for Dock")
        dock = self.exploreFor(type = "dock",conf_thresh = 0.4)
        if dock is None:
            rospy.logerr("Cant find dock :(")
            return

    #self.spinOnSpot(1)
    #self.circleObject("scan_buoy")
    #self.scan_code()

    rospy.loginfo("Found Dock")
    #self.spinOnSpot(1)
    self.logDock("Exectuing circling of dock")

    self.circleObject("dock", revs=0.6, clockwise=True)
    #self.circleObject("dock", revs=0.2, clockwise=False)
#
    if self.placard_symbol is None:
        rospy.loginfo("Requesting placard Symbol")
        self.placard_symbol = self.getRequestedPlacardSymbol()
    self.logDock("Requested placard symbol is %s."%self.placard_symbol)

    correct_bay = None
    # TODO: GET THE CLOSEST DOCK LOCATION FIRST
    dock = self.findClosest(self.unused_objects, type="dock", conf_thresh = 0.6)
    if dock is None:
        rospy.logerr("Cannot find Dock")
        return

    loc1 = self.translatePose(dock.pose,self.align_dist,0,np.pi/2 )
    loc2 = self.translatePose(dock.pose,-self.align_dist,0,-np.pi/2 )
    loc3 = self.self.translatePose(dock.pose,0,-self.align_dist,-np.pi/2 )



    bay_pose1 = self.translatePose(dock.pose,self.bay_dist,0,np.pi/2  )
    bay_pose2 = self.translatePose(dock.pose,-self.bay_dist,0,-np.pi/2 )



    if self.getDist(self.current_pose, loc1) > self.getDist(self.current_pose, loc2):
        #loc 2 is closer swap the values around
        temp = loc1
        loc1 = loc2
        loc2 = temp

        temp2 = bay_pose1
        bay_pose1 = bay_pose2
        bay_pose2 = temp2

    self.navigateTo(loc1,ang_thresh = 0.1)
    rospy.sleep(2)
    if self.checkPlacard() == True:
        self.logDock("Found correct bay. Ready to dock.")
        self.publishMarker(bay_pose1)
        self.performDock(bay_pose1)

    else:
        loc3.orientation = self.current_pose.orientation
        self.navigateTo(loc3,dist_thresh = 3, ang_thresh = 0.5)
        self.navigateTo(loc2,ang_thresh = 0.1)
        rospy.sleep(2)
        if self.checkPlacard() == True:
            self.logDock("Found correct bay. Ready to dock.")
            self.publishMarker(bay_pose2)
            self.performDock(bay_pose2)
        else:
            rospy.logwarn("No correct bay found, attempting to dock either way")
            self.publishMarker(bay_pose2)
            self.performDock(bay_pose2)




      #self.circleObject("dock", revs=0.5, look_dock=False) # Circle to other side of dock


    self.logDock("Completed Docking!")

  def waitForWaypointRequest(self):
    rospy.wait_for_message("/request_waypoints", Empty)

  def setPoseQuat(self, pose, angle_rad):
    quat = quaternion_from_euler(0, 0, angle_rad)
    pose.orientation.x = quat[0]
    pose.orientation.y = quat[1]
    pose.orientation.z = quat[2]
    pose.orientation.w = quat[3]

  def makeWaypoint(self, pose, nav_type=Waypoint.NAV_STATION, duration=0):
    wp = Waypoint() # Create wp message
    wp.nav_type = nav_type
    wp.pose = deepcopy(pose)
    wp.station_duration = duration
    return wp

  def quatToYaw(self, x, y, z, w):
    euler = euler_from_quaternion([x, y, z, w])
    return euler[2]

  def checkPlacard(self):
    label = ""
    attempts = 0
    while label == "":
        rospy.loginfo("Classifying placard")
        ros_img = rospy.wait_for_message("/wamv/sensors/cameras/middle_camera/image_raw", Image)
        res = None

        image = self.bridge.imgmsg_to_cv2(ros_img, desired_encoding="bgr8")
        label, _, _ = self.placardClassifier.classifyPlacard(image)

        if attempts > 30:
            rospy.logwarn("Could not classify placard")
            image_path = '/home/johnsumskas/images/placard.png'
            cv2.imwrite(image_path,image)
            break
        attempts = attempts+1




    # while res is None or res.success==False:
    #   self.logDock("Attempting to Classify")
    #   try:
    #     classifyPlacard = rospy.ServiceProxy('/wamv/wamv/classify_placard', ClassifyPlacard)
    #     res = classifyPlacard(ros_img)
    #   except rospy.ServiceException, e:
    #     self.logDock("Service call to /wamv/classify_placard failed: %s"%e)
    #     return False
    #
    # self.logDock("Placard classifier result: %s"%res.label)
    if label == self.placard_symbol:
      return True
    else:
      return False

  def spinOnSpot(self, n_times):
    self.logDock("Spinning on the spot %d times."%n_times)

    spin_wp_route = WaypointRoute()
    spin_wps = []
    odom_msg = rospy.wait_for_message("/wamv/odom", Odometry) # Current odom

    boat_pose = Pose() # Set current boat position
    boat_pose.position.x = odom_msg.pose.pose.position.x
    boat_pose.position.y = odom_msg.pose.pose.position.y

    # Get current boat angle
    boat_yaw = self.quatToYaw(odom_msg.pose.pose.orientation.x, odom_msg.pose.pose.orientation.y,
                              odom_msg.pose.pose.orientation.z, odom_msg.pose.pose.orientation.w)

    for _ in range(n_times): # How many revolutions
      for i in range(self.n_onspot_wps): # How many angular divisions

        angle = boat_yaw + (float(i+1)/self.n_onspot_wps)*2*np.pi # Set angle
        self.setPoseQuat(boat_pose, angle)

        spin_wp = self.makeWaypoint(boat_pose) # Create wp message
        spin_wps.append(spin_wp)

    spin_wp_route.waypoints = spin_wps
    spin_wp_route.speed = 1
    self.route_pub.publish(spin_wp_route) # Publish route
    self.waitForWaypointRequest()    # Wait til route complete

  def dumbFunction(self):
    return True

  def getRequestedPlacardSymbol(self):
    self.logDock("Waiting for placard symbol message.")
    return (rospy.wait_for_message('/vrx/scan_dock/placard_symbol', String)).data

  def findFrontOfScanBuoy(self):
    image_msg = rospy.wait_for_message( # Get image from middle camera
      'sensors/cameras/middle_camera/image_raw', Image)
    try:
      scanBuoy = rospy.ServiceProxy('scan_buoy', ClassifyBuoy)
      colour_string = scanBuoy(image_msg, 0)
      if (colour_string is not "none"):
        return True  # Found a colour from the scan buoy!
      else:
        return False # Bad angle on the scan buoy

    except rospy.ServiceException, e:
        self.logDock("Service call failed: %s"%e)
        return False # I dunno man

  def circleObject(self, object_string, revs=1.0, look_dock=True, clockwise=True,thresh = 0):
    ''' object_string (string): "dock", "any_dock", "scan_buoy", "explore"
    '''
    self.logDock("Circling the " + object_string + ".")

    radius = 0  # Assign these variables to either circle the dock or scan buoy
    object_pos = []
    identify_function = self.dumbFunction

    dir = 1.0
    if not clockwise:
      dir = -1.0

    nav_msg_type = 0 # Choose whether to look at dock while circling or just boat around
    if look_dock == True:
      nav_msg_type = Waypoint.NAV_STATION
    else:
      nav_msg_type = Waypoint.NAV_WAYPOINT

    if (object_string == "dock" or object_string == "any_dock"):
      object_pos, _ = self.getDockPose() # Dock centroid
      radius = self.dock_radius
    elif (object_string == "scan_buoy"):
      object_pos, _ = self.getScanBuoyPose() # Scan buoy centroid
      radius = self.scan_radius
    elif (object_string == "explore"): # Explore!
      object_pos, _ = self.getExplorePose()
      radius = self.explore_radius

    '''if (object_string == "dock"):
      identify_function = self.checkPlacard
    else:
      identify_function = self.dumbFunction'''

    odom_msg = rospy.wait_for_message("/wamv/odom", Odometry) # Current odom

    init_vec = [object_pos[0] - odom_msg.pose.pose.position.x, # Boat-to-object vector
                object_pos[1] - odom_msg.pose.pose.position.y]
    init_dist = np.sqrt(init_vec[0]**2 + init_vec[1]**2) # Boat-to-object distance

    init_pose = Pose() # First position, at given radius from the object
    init_pose.position.x = (object_pos[0] + float(radius)*(-init_vec[0]/init_dist))
    init_pose.position.y = (object_pos[1] + float(radius)*(-init_vec[1]/init_dist))

    init_angle = math.atan2(init_vec[1], init_vec[0]) # Set angle to face object

    if clockwise:
        init_angle = init_angle+1.5707
    else:
        init_angle = init_angle-1.5707
    self.setPoseQuat(init_pose, init_angle)

    circle_wp_route = WaypointRoute() # Waypoint list
    circle_wps = []

    #init_wp = self.makeWaypoint(init_pose, nav_type=nav_msg_type) # Create waypoint from above data
    #circle_wps.append(init_wp)

    rev_angle = revs*2*np.pi
    n_wps = int(np.ceil(revs*self.n_circle_wps))

    for i in range(n_wps):
      wp_pose = Pose() # Set first circling waypoint pose
      rot = dir*(i+1)*(rev_angle/n_wps)

      x = init_pose.position.x # Rotate object viewing position
      y = init_pose.position.y
      ox = object_pos[0]
      oy = object_pos[1]

      # Rotation (2x2 matrix essentially)
      wp_pose.position.x = ox + math.cos(rot) * (x - ox) + math.sin(rot) * (y - oy)
      wp_pose.position.y = oy + -math.sin(rot) * (x - ox) + math.cos(rot) * (y - oy)

      # Work out angle to object
      wp_vec = [object_pos[0] - wp_pose.position.x, object_pos[1] - wp_pose.position.y]
      wp_angle = math.atan2(wp_vec[1], wp_vec[0])

      if clockwise:
          wp_angle = wp_angle+1.5707
      else:
          wp_angle = wp_angle-1.5707

      self.setPoseQuat(init_pose, init_angle)
      self.setPoseQuat(wp_pose, wp_angle)

      if i == n_wps-1: # Always set last waypoint to be astation
        spin_wp = self.makeWaypoint(wp_pose, nav_type=Waypoint.NAV_STATION, duration=5.0)
      else:
        spin_wp = self.makeWaypoint(wp_pose, nav_type=nav_msg_type) # Set waypoint

      self.publishMarker(wp_pose, id = i)

      circle_wps.append(spin_wp)

    circle_wp_route.waypoints = circle_wps
    circle_wp_route.speed = self.general_speed
    self.route_pub.publish(circle_wp_route) # Start on route

    #r = rospy.Rate(2) # Wait until objective identified
    #while not identify_function() and not rospy.is_shutdown:
    # r.sleep()
    if thresh == 0:
        self.logDock("Waiting for wp request")
        self.waitForWaypointRequest()
        return
    else:
        conf = 0
        while(conf <thresh ):
            #TODO: Make a function to find the object base on frame_id
            dock = self.findClosest(self.unused_objects, type="dock")
            if len(dock.confidences) != 0:
                conf = dock.confidences[0]


  def alignWithDock(self, bay_index, duration=0.0):
    self.logDock("Aligning with the dock.")
    dock_pos, dock_yaw = self.getDockPose()

    align_pose = Pose()

    if bay_index == 0:
      align_pose.position.x = dock_pos[0] + self.align_dist*math.cos(dock_yaw)
      align_pose.position.y = dock_pos[1] + self.align_dist*math.sin(dock_yaw)
      align_angle = dock_yaw + np.pi
    else:
      align_pose.position.x = dock_pos[0] - self.align_dist*math.cos(dock_yaw)
      align_pose.position.y = dock_pos[1] - self.align_dist*math.sin(dock_yaw)
      align_angle = dock_yaw

    if (align_angle > np.pi):
      align_angle -= 2*np.pi
    self.setPoseQuat(align_pose, align_angle)

    #align_wp = self.makeWaypoint(align_pose)
    #align_wps = WaypointRoute()
    #align_wps.waypoints = [align_wp]
    #align_wps.speed = self.general_speed
    #self.route_pub.publish(align_wps)
    rospy.loginfo("Naviating to allign with dock")
    self.navigateTo(align_pose,ang_thresh=0.1)

    #self.waitForWaypointRequest()

  def getDockPose(self):
    objects_msg = rospy.wait_for_message('/wamv/objects', ObjectArray)
    dock_frame_id = None
    dock_pose = None
    max_conf = 0

    while dock_frame_id is None:
        for object in objects_msg.objects:
          if object.best_guess == "dock":
            if object.confidences[0] > max_conf:
              dock_frame_id = object.frame_id
              dock_pose = object.pose
              max_conf = object.confidences[0]
        rospy.sleep(0.5)
    if dock_frame_id is None: # Couldn't find dock
      self.logDock("Dock not found.")
      return None, None

    # dock_pos = [dock_pose.position.x, dock_pose.position.y, 0]
    # dock_yaw = self.quatToYaw(dock_pose.orientation.x, dock_pose.orientation.y,
    #                           dock_pose.orientation.z, dock_pose.orientation.w)
    # dock_yaw -= np.pi/2
    # return dock_pos, dock_yaw

    try:
      tf_pos, tf_rot = self.tf_listener.lookupTransform('/map', dock_frame_id, rospy.Time(0))
      dock_yaw = tf.transformations.euler_from_quaternion(tf_rot)[2]
      dock_yaw -= np.pi/2
      return tf_pos, dock_yaw
    except tf.LookupException as e:
      self.logDock(e)
      return None, None

    '''tf_pos = [137, 100, 0] # TODO uncomment above code when dock position estimation is improved
    tf_rot = [0, 0, 0, 1]
    return tf_pos, tf_rot'''

  def getExplorePose(self):

    odom_msg = rospy.wait_for_message("/wamv/odom", Odometry) # Current odom

    boat_yaw = self.quatToYaw(odom_msg.pose.pose.orientation.x, odom_msg.pose.pose.orientation.y,
                              odom_msg.pose.pose.orientation.z, odom_msg.pose.pose.orientation.w)

    pos_x = odom_msg.pose.pose.position.x + self.explore_dist*math.cos(boat_yaw)
    pos_y = odom_msg.pose.pose.position.y + self.explore_dist*math.sin(boat_yaw)

    tf_pos = [pos_x, pos_y, 0]
    tf_rot = [0, 0, 0, 1]
    return tf_pos, tf_rot

  def performDock(self, bay_pose):
    self.logDock("Starting docking procedure.")

    client = actionlib.SimpleActionClient('/wamv/docking', vrx_msgs.msg.DockAction)
    rospy.loginfo("Waiting for docking server")
    client.wait_for_server()
    rospy.loginfo("Found Docking Server, Executing Dock")
    # bay_pose = Pose()
    # dock_pos, dock_yaw = self.getDockPose()
    #
    #
    #
    # if bay_index == 0:
    #   bay_pose.position.x = dock_pos[0] + self.bay_dist*math.cos(dock_yaw)
    #   bay_pose.position.y = dock_pos[1] + self.bay_dist*math.sin(dock_yaw)
    #   align_angle = dock_yaw + np.pi
    # else:
    #   bay_pose.position.x = dock_pos[0] - self.bay_dist*math.cos(dock_yaw)
    #   bay_pose.position.y = dock_pos[1] - self.bay_dist*math.sin(dock_yaw)
    #   align_angle = dock_yaw

    # if (align_angle > np.pi):
    #   align_angle -= 2*np.pi
    align_angle = quatToEuler(bay_pose.orientation)[2]
    # Creates a goal to send to the action server.
    goal = vrx_msgs.msg.DockGoal(bay_pose, align_angle)

    # Sends the goal to the action server.
    client.send_goal(goal)

    # Waits for the server to finish performing the action.
    client.wait_for_result()

  def logDock(self, msg):
    rospy.loginfo("~dockmaster: %s"%msg)

def main():
  rospy.init_node("dockmaster")
  dm = DockMaster()
  rospy.spin()


if __name__ == "__main__":
  main()
