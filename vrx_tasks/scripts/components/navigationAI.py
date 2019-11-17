# state: either "moving" or "still"; depending on whether we are moving or still.

import rospy
from vrx_msgs.msg import ObjectArray, Object, Waypoint, WaypointRoute
from geometry_msgs.msg import Pose
from std_msgs.msg import Empty
import math
import tf
from tf.transformations import quaternion_from_euler
# issues:
# have to find a white one first - shall we just treat red

# distance at which we want to be curious


def verbosePrint(data):
    if True:
        print(data)


class navigationAI:
    def __init__(self):
        print("NAV AI STARTED")
        params = {
            "objTopic": "/wamv/objects",
            "outTopic": "/waypoints_cmd",
            "speed": 5,
            "movementTopic": "/request_waypoints"
        }
        self.state = "still"
        self.objectCache = None
        self.addressedState = {}
        self.tf_listener = tf.TransformListener()
        # addressedstate can either be identified, or navigated. (navigated implies identified)
        # start listening for objects
        rospy.Subscriber(params['objTopic'], ObjectArray, self.objectCB)
        self.pub = rospy.Publisher(
            params['outTopic'], WaypointRoute, queue_size=1)
        rospy.Subscriber(params['movementTopic'], Empty, self.motionCB)
        self.CURIO_DIST = 5
        self.WAITTIME = 20

    def objectCB(self, data):
        verbosePrint("object callback recieved: {}".format(len(data.objects)))
        # integrate the data into our list
        self.objectCache = data
        # if we are still, check what kinds of objects are there and activate curiosity or actual good moving
        if (self.state == "still"):
            verbosePrint("still,proceeding")
            # Find closest unnavigated red and green
            closest_ranges = {
                "surmark_46104": 1000,
                "surmark_950400": 1000,
                "surmark_950410": 1000
            }
            closest_frames = {
                "surmark_46104": None,
                "surmark_950400": None,
                "surmark_950410": None
            }
            for i in data.objects:
                if not((i.frame_id in self.addressedState) and (self.addressedState[i.frame_id] == "navigated")):
                    if i.best_guess in closest_ranges:
                        self.tf_listener.waitForTransform(
                            i.frame_id, "base_link", rospy.Time(0), rospy.Duration(self.WAITTIME))
                        (trans, rot) = self.tf_listener.lookupTransform(
                            i.frame_id, "base_link", rospy.Time(0))
                        dist = math.sqrt(trans[0]*trans[0]+trans[1]*trans[1])
                        if dist < closest_ranges[i.best_guess]:
                            closest_ranges[i.best_guess] = dist
                            closest_frames[i.best_guess] = i.frame_id
            closest_left = None
            if (closest_ranges["surmark_950400"] < closest_ranges["surmark_46104"]):
                closest_left = closest_frames["surmark_950400"]
            else:
                closest_left = closest_frames["surmark_46104"]
            if (not closest_left is None) and (not closest_frames["surmark_950410"] is None):
                course = WaypointRoute()
                self.tf_listener.waitForTransform(
                    closest_left, "map", rospy.Time(0), rospy.Duration(self.WAITTIME))
                self.tf_listener.waitForTransform(
                    closest_frames["surmark_950410"], "map", rospy.Time(0), rospy.Duration(self.WAITTIME))
                (leftPos, rot) = self.tf_listener.lookupTransform(
                    closest_left, "map", rospy.Time(0))
                (rightPos, rot) = self.tf_listener.lookupTransform(
                    closest_frames["surmark_950410"], "map", rospy.Time(0))
                midpoint = Waypoint()
                midpoint.pose.position.x = (leftPos[0]+rightPos[0])/2
                midpoint.pose.position.y = (leftPos[1]+rightPos[1])/2
                course.waypoints.append(midpoint)
                self.pub.publish(course)
                self.state = "moving"
                self.addressedState[closest_left] = "navigated"
                print("moving between {} and {}".format(
                    closest_left, closest_frames["surmark_950410"]))
            else:
                verbosePrint("marker find failed, curious mode")
                # Curiosity
                # Find closest unknown
                mindist = 1000
                minItem = None
                for i in data.objects:
                    if (i.best_guess == "buoy"):
                        self.tf_listener.waitForTransform(
                            i.frame_id, "base_link", rospy.Time(0),rospy.Duration(self.WAITTIME))
                        (trans, rot) = self.tf_listener.lookupTransform(
                            i.frame_id, "base_link", rospy.Time(0))
                        dist = math.sqrt(trans[0]*trans[0]+trans[1]*trans[1])
                        if dist < mindist:
                            mindist = dist
                            minItem = i.frame_id
                if (not minItem is None):
                    print("curiously inspecting {}".format(minItem))
                    course = WaypointRoute()
                    self.tf_listener.waitForTransform(
                        minItem, "base_link", rospy.Time(0), rospy.Duration(self.WAITTIME))
                    (relPos, rot) = self.tf_listener.lookupTransform(
                        minItem, "base_link", rospy.Time(0))
                    self.tf_listener.waitForTransform(
                        "base_link", "map", rospy.Time(0), rospy.Duration(self.WAITTIME))
                    (basePos, rot) = self.tf_listener.lookupTransform(
                        "base_link", "map", rospy.Time(0))

                    itemWP = Waypoint()
                    itemWP.pose.position.x = relPos[0] * \
                        self.CURIO_DIST/mindist + basePos[0]
                    itemWP.pose.position.y = relPos[1] * \
                        self.CURIO_DIST/mindist + basePos[1]

                    # orientation is also important here so we are looking at the bouy
                    q = quaternion_from_euler(
                        0, 0, math.atan2(relPos[1], relPos[0]))
                    itemWP.pose.orientation.x = q[0]
                    itemWP.pose.orientation.y = q[1]
                    itemWP.pose.orientation.z = q[2]
                    itemWP.pose.orientation.w = q[3]

                    course.waypoints.append(itemWP)
                    self.pub.publish(course)
                    self.state = "moving"

    def motionCB(self, data):
        self.state == "still"
