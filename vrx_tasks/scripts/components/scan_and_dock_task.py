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
from vrx_gazebo.srv import ColorSequence, ColorSequenceResponse


class ScanDock(Mission):
    def __init__(self):
        self.bridge = CvBridge()
        self.scanner = Scanner()
        rospy.loginfo("Waiting for reporting service")

        try:
            rospy.wait_for_service('/vrx/scan_dock/color_sequence',timeout = 10)
            self.report_seqence_service = rospy.ServiceProxy('/vrx/scan_dock/color_sequence',ColorSequence)
        except:
            rospy.logwarn("Failed to register service")
        rospy.loginfo("Initalizing Scan and Docking")
        Mission.__init__( self )




    def start(self):
        rospy.loginfo("Starting Scan and Docking")


        rospy.loginfo("Sleeping for 5 seconds")
        rospy.sleep(1)
        ##Find the Scan Buoy
        scan = self.exploreFor(type="buoy",conf_thresh = 0.3)

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
        #self.reportSequence(sequence)
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

        # target = Pose()
        # target.position.x = 100
        # target.position.y = 100
        # target.orientation = self.current_pose.orientation
        # target = self.navigateTo(target)

        docker = self.exploreFor(type="dock", conf_thresh=0.35)

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
        found = False
        sequence = []
        last_colour = "none"
        count = 0
        rospy.loginfo("Attempting to Scan buoy")

        while(found == False):
            ros_image = rospy.wait_for_message("wamv/sensors/cameras/middle_camera/image_raw",Image)
            image = self.bridge.imgmsg_to_cv2(ros_image, desired_encoding="bgr8")
            colour = self.scanner.scanBuoy(image)
            #rospy.loginfo("Detected colour %s",colour)
            if colour == "none":
                if last_colour != "none":
                    #if it is the first out of a reading
                    last_colour = "none"
                    count = 0
                count = count +1
                sequence = []
                continue
            if colour != last_colour :
                sequence.append(colour)
                last_colour = colour
                count = 0
            else:
                count = count+1

            if len(sequence)>=3:
                rospy.loginfo("Detected sequence is %s, %s,%s,",sequence[0],sequence[1],sequence[2])
                found = True
                return sequence

            if count > 100:
                rospy.logwarn("No change in colour found for 100 frames")
                return ["red", "green", "blue"]






        return sequence


    def reportSequence(self,sequence):
        rospy.logdebug("Reporting the sequence")
        msg = ColorSequence()
        msg.color1 = sequence[0]
        msg.color2 = sequence[1]
        msg.color3 = sequence[2]
        self.report_seqence_service(sequence[0],sequence[1],sequence[2])
        return

    #def imageCb(self,image):
    #    self.image = self.bridge.imgmsg_to_cv2(image, desired_encoding="bgr8")
    #    if self.scan:
        #    colour = scanner()
