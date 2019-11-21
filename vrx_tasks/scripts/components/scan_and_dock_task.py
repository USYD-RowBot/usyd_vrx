#!/usr/bin/env python
import rospy
from mission_base import Mission, quatToEuler ,eulerToQuat
from vrx_msgs.msg import ObjectArray, Object, Waypoint, WaypointRoute
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Pose, Quaternion, PoseStamped
from visualization_msgs.msg import Marker
import math
import tf
from buoy_scanner import Scanner
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
from dockmaster import DockMaster

class ScanDock(Mission):
    def __init__(self):
        self.bridge = CvBridge()
        rospy.loginfo("Initalizing Scan and Docking")
        Mission.__init__( self )
        middle = rospy.Subscriber("wamv/sensors/cameras/middle_camera/image_raw",Image,self.imageCb)
        self.image = None


    def start(self):
        rospy.loginfo("Starting Scan and Docking")


        rospy.loginfo("Sleeping for 5 seconds")
        rospy.sleep(1)
        ##Find the Scan Buoy
        scan = self.findClosest(self.unused_objects, type="buoy")

        if scan is None:
            rospy.logwarn("No buoys found")
            return

        #print(scan)
        target = Pose()
        target.position = scan.pose.position
        target.orientation = self.current_pose.orientation
        target = self.translatePose(target,0,-8,0)

        self.navigateToDirect(target)

        rospy.logdebug("Infrone of Buoy. Initalizing scan")

        sequence = self.scanBuoy()
        if sequence is None:
            rospy.logwarn("No sequence found, defaulting to rgb")
            sequence = ["red","green","blue"]

        print(sequence)
        self.reportSequence(sequence)
        colour = sequence[0]

        if sequence[2] == "red":
            shape = "_circle"
        elif sequence[2]== "green":
            shape = "_triangle"
        elif sequence[2]== "blue":
            shape = "_cross"

        placard = colour+shape

        rospy.loginfo("The Docking placard is %s",placard)

        self.used_objects.append(scan)
        self.updateUnused()

        target = Pose()
        target.position.x = 100
        target.position.y = 100
        target.orientation = self.current_pose.orientation
        target = self.navigateTo(target)

        rospy.loginfo("attempting to find nearest dock")
        dock = self.findClosest(self.unused_objects, type="dock",conf_thresh = 0.35)

        if dock is None:
            dock = self.findClosest(self.unused_objects, type="object")

            if dock is None:
                rospy.logwarn("Cannot find anything")
                return
                #TODO: Expore


        target = Pose()
        target.position = dock.pose.position
        target.orientation = self.current_pose.orientation
        target = self.translatePose(target,0,-20,0)

        self.navigateTo(target)


        dm = DockMaster(placard_symbol = placard)

        rospy.loginfo("Finishing Scan and Docking")

    def scanBuoy(self):
        #scanner = Scanner()
        sequence = ["red","green","blue"]
        return sequence


    def reportSequence(self,sequence):
        rospy.logdebug("Reporting the sequence")
        pass


    def imageCb(self,image):
        self.image = self.bridge.imgmsg_to_cv2(image, desired_encoding="bgr8")
