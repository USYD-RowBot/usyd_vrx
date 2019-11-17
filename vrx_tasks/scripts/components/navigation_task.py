import rospy
from vrx_msgs.msg import ObjectArray, Object, Waypoint, WaypointRoute
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Pose, Quaternion
from visualization_msgs.msg import Marker
import math
import tf


tf_listener = None

def quatToEuler(quat):

    if quat.x is None:
        #Must be a in tf form.
        quaternion = quat
    else:
        quaternion = (
        quat.x,
        quat.y,
        quat.z,
        quat.w)
    euler = tf.transformations.euler_from_quaternion(quaternion)

    return euler[0],euler[1],euler[2] #RPY

def eulerToQuat(euler):
    q = tf.transformations.quaternion_from_euler(euler[0], euler[1], euler[2])
    quat = Quaternion()
    quat.x = q[0]
    quat.y = q[1]
    quat.z = q[2]
    quat.w = q[3]
    return quat


class NavigationTask:
    def __init__(self):
        global tf_listener
        print("Initalising Navigation")
        #rospy.init_node("Navigation_task")
        self.object_sub = rospy.Subscriber("wamv/objects/", ObjectArray, self.objectsCb)
        self.odom_sub = rospy.Subscriber("wamv/odom/", Odometry, self.odomCb)
        self.waypoint_pub = rospy.Publisher("waypoints_cmd", WaypointRoute,queue_size = 10)
        self.marker_pub = rospy.Publisher("nav_marker", Marker, queue_size=10)
        self.object_list = None
        self.used_objects = []
        self.unused_objects = None
        self.current_pose = Pose()
        tf_listener = tf.TransformListener()

        return

    def startNavigation(self):
        rospy.loginfo("Starting Navigation")
        rospy.loginfo("Waiting 5 seconds")
        rospy.sleep(5)

        #Find first two buoys.
        white = self.findClosest(self.unused_objects,type="white")
        if white is None:
            rospy.loginfo("No White Buoy found")
            return
        red = self.findClosest(self.unused_objects,type="red")
        if red is None:
            rospy.loginfo("No Red Buoy found")
            return

        rospy.loginfo("Found Red and white buoys ID %s and ID %s", red.frame_id, white.frame_id )
        #Navigate to the gate in between
        target = self.getGatePose(white,red)
        self.publishMarker(target)
        self.navigateTo(target)

        self.used_objects.append(red)
        self.used_objects.append(white)

        self.updateUnused()
        while True:
            green = self.findClosest(self.unused_objects,type="green")
            red = self.findClosest(self.unused_objects,type="red")
            if green is None:
                rospy.loginfo("No green Buoy found")
                break

            elif red is None:
                rospy.loginfo("No Red Buoy found")
                break;
            else:
                rospy.loginfo("Found Red and green buoys ID %s and ID %s", red.frame_id, green.frame_id )
                #Navigate to the gate in between
                target = self.getGatePose(green,red)
                self.publishMarker(target)
                self.navigateTo(target)
                self.used_objects.append(red)
                self.used_objects.append(green)
                self.updateUnused()



        blue = self.findClosest(self.unused_objects,type="blue_totem")
        if blue is None:
            rospy.loginfo("No blue Buoy found")
            return
        red = self.findClosest(self.unused_objects,type="red")
        if red is None:
            rospy.loginfo("No Red Buoy found")
            return;

        rospy.loginfo("Found Red and blue buoys ID %s and ID %s", red.frame_id, blue.frame_id )
        #Navigate to the gate in between
        target = self.getGatePose(blue,red)
        self.publishMarker(target)
        self.navigateTo(target)
        self.used_objects.append(red)
        self.used_objects.append(blue)
        self.updateUnused()
        rospy.loginfo("END of Program")
        return


    def navigateTo(self, target, wait=True, timeout=0):
        """ Navigate to the location, if wait is True: Wait until destination is reached, if not, """

        rospy.loginfo("Navigating to a location x: %d. y:%d", target.position.x, target.position.y)
        waypoint_msg = WaypointRoute()
        waypoint_msg.speed = 2
        ##For Now, waypoints are 2 waypoints. At nav waypoint then a nav station
        loc0 = Waypoint()
        loc0.pose = self.current_pose
        loc0.nav_type = Waypoint.NAV_STATION
        loc0.station_duration = 2
        loc1 = Waypoint()
        loc1.pose = target
        loc1.nav_type = Waypoint.NAV_WAYPOINT
        loc2 = Waypoint()
        loc2.pose = target
        loc2.nav_type = Waypoint.NAV_STATION
        loc2.station_duration = -5
        waypoint_msg.waypoints=[loc0,loc1,loc2]
        self.waypoint_pub.publish(waypoint_msg)

        rate = rospy.Rate(20)
        while not self.inRange(target):
            rate.sleep()
        rospy.loginfo("Arrived at target")
        rospy.sleep(3)
        return


    def getGatePose(self,left_buoy,right_buoy):
        global tf_listener
        target = Pose()
        target.position.x = 0.0
        target.position.y = 0.0
        target.position.x = (left_buoy.pose.position.x + right_buoy.pose.position.x) /2.0
        target.position.y = (left_buoy.pose.position.y + right_buoy.pose.position.y) /2.0
        target.position.z = 0.0

        yaw = 0.0
        yaw = math.atan2(left_buoy.pose.position.y - right_buoy.pose.position.y, left_buoy.pose.position.x - right_buoy.pose.position.x)-1.507
        rospy.loginfo("Got gate location x: %f, y: %f, yaw: %f",target.position.x ,target.position.y,yaw)
        target.orientation = eulerToQuat([0,0,yaw])
        #print(left_buoy,right_buoy, target)
        return target



    def inRange(self, target, dist_thresh = 0.5, angle_thresh = 0.4):
        dist = math.sqrt((self.current_pose.position.x-target.position.x)**2 + (self.current_pose.position.y-target.position.y)**2)

        angle = abs(quatToEuler(self.current_pose.orientation)[2] - quatToEuler(target.orientation)[2])

        if (dist<=dist_thresh) and (angle <= angle_thresh):
            return True
        else:
            return False

    def odomCb(self, odom_msg):
        self.current_pose = odom_msg.pose.pose
        return



    def objectsCb(self,msg):
        self.object_list = msg.objects

        #Remove unused objects from a list.
        self.unused_objects = self.object_list[:]
        for object in self.unused_objects[:]:
            #Check if object in in the used object list.
            id = object.frame_id

            for i in self.used_objects:
                if i.frame_id == id:
                    #If there is a match, remove the obect
                    self.unused_objects.remove(object)
                    break
        return

    def updateUnused(self):
        for object in self.unused_objects[:]:
            #Check if object in in the used object list.
            id = object.frame_id

            for i in self.used_objects:
                print("Checking unused: %s with used %s",id,i.frame_id)
                if i.frame_id == id:
                    #If there is a match, remove the obect
                    rospy.loginfo("Found used object %s",object.frame_id)
                    try:
                        self.unused_objects.remove(object)
                        break
                    except:
                        continue

        return


    def publishMarker(self, pose):
        marker = Marker()
        marker.header.frame_id = "map"
        marker.header.stamp = rospy.Time.now()
        marker.ns = "nav_task"
        marker.id = 1
        marker.type = Marker.ARROW
        marker.action=Marker.ADD
        marker.pose = pose
        marker.scale.x = 2.0
        marker.scale.y = 2.0
        marker.scale.z = 2.0
        marker.color.r = 0.0
        marker.color.g = 0.0
        marker.color.b = 0.0
        marker.color.a = 1.0
        marker.lifetime = rospy.Duration(0)
        self.marker_pub.publish(marker)

    def findClosest(self,object_list, type="object"):

        if object_list is None:
            rospy.logwarn("Find Closest passed empty list")
            return None
        global tf_listener
        closest = None
        min_dist = None
        if type =="object":
            accepted_objects = ["dock", "buoy", "yellow_totem", "black_totem", "blue_totem", "green_totem", "red_totem", "polyform_a3", "polyform_a5", "polyform_a7", "surmark_46104", "surmark_950400", "surmark_950410"]
        elif type == "buoy":
            accepted_objects = ["buoy", "yellow_totem", "black_totem", "blue_totem", "green_totem", "red_totem", "polyform_a3", "polyform_a5", "polyform_a7", "surmark_46104", "surmark_950400", "surmark_950410"]
        elif type == "totem":
            accepted_objects = ["yellow_totem", "black_totem", "blue_totem", "green_totem", "red_totem"]
        elif type == "polyform":
            accepted_objects= ["polyform_a3", "polyform_a5", "polyform_a7"]
        elif type == "white":
            accepted_objects = ["surmark_46104"]
        elif type == "green":
            accepted_objects = ["surmark_950400"]
        elif type == "red":
            accepted_objects = ["surmark_950410"]
        else:
            accepted_objects = [type]

        for object in object_list:
            #Get distance of object
            if object.best_guess in accepted_objects:
                try:
                    (trans,rot) = tf_listener.lookupTransform("base_link",object.frame_id, rospy.Time(0))
                except Exception as e:
                    rospy.logwarn("Transform Lookup Error")
                    print(e)
                    continue

                dist = math.sqrt(trans[0]**2 +trans[1]**2)
                if min_dist is None or dist<min_dist:
                    closest = object
                    min_dist = dist

        if closest is None:
            rospy.loginfo("object type :%s not found", type)
        return closest
