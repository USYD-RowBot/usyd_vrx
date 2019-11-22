import rospy
from vrx_msgs.msg import ObjectArray, Object, Waypoint, WaypointRoute
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Pose, Quaternion, PoseStamped
from visualization_msgs.msg import Marker
import math
import tf


class Mission():

    def __init__(self):
        print("Initalising Mission Base")

        self.object_list = None
        self.used_objects = []
        self.unused_objects = None
        self.current_pose = Pose()
        self.tf_listener = tf.TransformListener()
        self.object_sub = rospy.Subscriber("wamv/objects/", ObjectArray, self.objectsCb)
        self.odom_sub = rospy.Subscriber("wamv/odom/", Odometry, self.odomCb)
        self.waypoint_pub = rospy.Publisher("waypoints_cmd", WaypointRoute,queue_size = 10)
        self.marker_pub = rospy.Publisher("nav_marker", Marker, queue_size=10)
        self.goal_pub = rospy.Publisher("wamv/global_planner/goal", PoseStamped, queue_size = 10)

    def getDist(self,pose1,pose2):
        return math.sqrt((pose1.position.x-pose2.position.x)**2 + (pose1.position.y-pose2.position.y)**2 )

    def translatePose(self,pose,x,y, yaw):
        """Return a pose that is moved relative to the given coordinates
        Y is ahead and x is to the right
        """
        _,_,current_yaw = quatToEuler(pose.orientation)
        new_pose = Pose()
        new_pose.position.y = pose.position.y + y*math.sin(current_yaw) - x*math.cos(current_yaw)
        new_pose.position.x = pose.position.x + y*math.cos(current_yaw) + x*math.sin(current_yaw)
        new_pose.orientation = eulerToQuat([0,0,current_yaw + yaw])

        return new_pose

    def navigateTo(self, target, wait=True, timeout = 0, dist_thresh = 1, ang_thresh = 0.4, repubish = False):
        self.publishMarker(target)
        # Navigate to the location, if wait is True: Wait until destination is reached, if not,
        self.navigateToDirect(target,wait=False,timeout=0)

        rospy.loginfo("Navigating to a location x: %f. y:%f", target.position.x, target.position.y)
        waypoint_msg = WaypointRoute()
        waypoint_msg.speed = 3
        ##For Now, waypoints are 2 waypoints. At nav waypoint then a nav station
        goal  = PoseStamped()
        goal.pose = target
        goal.header.frame_id = "map"
        goal.header.stamp = rospy.Time.now()

        self.goal_pub.publish(goal)
        start = rospy.Time.now().secs
        rate = rospy.Rate(20)
        expired = False
        while (not self.inRange(target,dist_thresh = dist_thresh, ang_thresh = ang_thresh)) and wait and not expired:
            rate.sleep()
            if timeout != 0 and rospy.Time.now().secs-start > timeout:
                expired = True

        #Republish a waypoint in its own position if wanted to wait, else, it will exit the function but continue on its trajectory.
        if repubish:
            waypoint_msg = WaypointRoute()
            waypoint_msg.speed = 2
            loc0 = Waypoint()
            loc0.pose = self.current_pose
            loc0.nav_type = Waypoint.NAV_STATION
            loc0.station_duration = -1
            waypoint_msg.waypoints=[loc0]
            self.waypoint_pub.publish(waypoint_msg)

        if wait:
            rospy.loginfo("Arrived at target")
        return

    def navigateToDirect(self, target, wait=True, timeout = 0, dist_thresh = 1, ang_thresh = 0.4):
        """ Navigate to the location, if wait is True: Wait until destination is reached, if not,  Not using the mission planner"""
        self.publishMarker(target)
        rospy.loginfo("Navigating to a location x: %f. y:%f", target.position.x, target.position.y)
        waypoint_msg = WaypointRoute()
        waypoint_msg.speed = 2
        ##For Now, waypoints are 2 waypoints. At nav waypoint then a nav station

        loc2 = Waypoint()
        loc2.pose = target
        loc2.nav_type = Waypoint.NAV_STATION
        loc2.station_duration = -1
        waypoint_msg.waypoints=[loc2]
        self.waypoint_pub.publish(waypoint_msg)

        start = rospy.Time.now().secs
        rate = rospy.Rate(20)
        expired = False
        while (not self.inRange(target,dist_thresh = dist_thresh, ang_thresh = ang_thresh)) and wait and not expired:
            #print(target,self.current_pose)
            rate.sleep()
            if timeout != 0 and (rospy.Time.now().secs-start) > timeout:
                rospy.loginfo("Timeing out: %f", rospy.Time.now().secs-start)
                expired = True

        #Republish a waypoint in its own position if wanted to wait, else, it will exit the function but continue on its trajectory.
        if wait:
            waypoint_msg = WaypointRoute()
            waypoint_msg.speed = 2
            loc0 = Waypoint()
            loc0.pose = self.current_pose
            loc0.nav_type = Waypoint.NAV_STATION
            loc0.station_duration = -1
            waypoint_msg.waypoints=[loc0]
            self.waypoint_pub.publish(waypoint_msg)


        rospy.loginfo("Arrived at target")
        return


    def inRange(self, target, dist_thresh = 1.2, ang_thresh = 0.4):

        dist = math.sqrt((self.current_pose.position.x-target.position.x)**2 + (self.current_pose.position.y-target.position.y)**2)
        #rospy.loginfo("Range: %f", dist)
        angle = abs(quatToEuler(self.current_pose.orientation)[2] - quatToEuler(target.orientation)[2])

        if (dist<=dist_thresh) and (angle <= ang_thresh):
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
        for i in self.unused_objects:
            if i is None:
                self.unused_objects.remove(i)

        for object in self.unused_objects[:]:
            #Check if object in in the used object list.
            id = object.frame_id

            for i in self.used_objects:
                if i is not None:
                    if i.frame_id == id:
                        #If there is a match, remove the obect
                        self.unused_objects.remove(object)
                        break
        return


    def updateUnused(self):
        for object in self.unused_objects[:]:
            #Check if object in in the used object list.
            id = object.frame_id
            if object is not None:
                for i in self.used_objects:
                    #print("Checking unused: %s with used %s",id,i.frame_id)
                    if i is not None:
                        if i.frame_id == id:
                            #If there is a match, remove the obect
                            rospy.loginfo("Found used object %s",object.frame_id)
                            try:
                                self.unused_objects.remove(object)
                                break
                            except:
                                continue
                    else:
                        self.used_objects.remove(i)

        return

    def publishMarker(self, pose,id = 0):

        if pose is None:
            return
        marker = Marker()
        marker.header.frame_id = "map"
        marker.header.stamp = rospy.Time.now()
        marker.ns = "nav_task"
        marker.id = id
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
    #
    # def findObject(self,object_list,frame_id):
    #     if object_list is None:
    #         rospy.logwarn("Find Closest passed empty list")
    #         return None
    #     for object in object_list:
    #         #Get distance of object
    #         if object.best_guess in accepted_objects and object.confidences[0]>conf_thresh:
    #



    def exploreFor(self,type="object",conf_thresh = 0.3):
        """ Try find an object by navigating around"""
        rospy.loginfo("Looking for %s", type)
        current_target_object = Object()
        current_target_object.frame_id = ""
        target = None
        random_target = None
        object_search_list_used = []
        object = None
        count = 0
        while object is None:
            object_search_list = self.unused_objects[:]
            for object in object_search_list[:]:
                for i in object_search_list_used:
                    if i.frame_id == object.frame_id:
                        object_search_list.remove(object)
                        break
            #Execute explore for object
            object = self.findClosest(object_search_list, type=type, conf_thresh = conf_thresh)
            if object is None:
                #rospy.loginfo("Cant find, looking for low confidnece")
                guess = self.findClosest(object_search_list, type=type, conf_thresh = 0)
                if guess is None:
                    #rospy.loginfo("Cant find, looking for Any Object")
                    guess = self.findClosest(object_search_list, type="object", conf_thresh = 0)
                    if guess is None:
                        rospy.logwarn("No Objects Found... at all")
                        #Go to a random posiiont:
                        count = count +1

                        if (random_target is None or self.inRange(random_target,dist_thresh = 6, ang_thresh = 1)) and count >10:
                            rospy.logwarn("Cannot find any new things to investigate, lets try a random position")
                            random_target = Pose()
                            random_target.position.x = 0
                            random_target.position.y = 50
                            random_target.orientation=self.current_pose.orientation
                            self.navigateTo(random_target, wait=False)
                            count = 0
                        rospy.sleep(0.5)
                        continue


                count = 0
                if guess is not None and current_target_object.frame_id != guess.frame_id:
                    rospy.loginfo("Getting a new location to try ")
                    target = Pose()
                    target.position = guess.pose.position
                    target.orientation = self.current_pose.orientation
                    target = self.translatePose(target,0,-15,0)
                    current_target_object = guess
                    self.navigateTo(target,wait=False)

                if target is not None and current_target_object is not None and (self.inRange(target) or (len(current_target_object.confidences) != 0 and current_target_object.confidences[0]>0.8)):
                    #If it is in at the nav location or the classificaiton is very high
                        ##Remove the item in object search list:
                    rospy.loginfo("Removing object because its not what we are looking for")
                    object_search_list_used.append(current_target_object)

                #Update the object search list




                rospy.sleep(0.5)

            else:
                rospy.loginfo("Found object %s whilst exploring",type)
                break


        return object


    def findClosest(self,object_list, type="object",frame="base_link", conf_thresh = 0,ignore_land = False):

        if object_list is None:
            rospy.logwarn("Find Closest passed empty list")
            return None
        tf_listener = self.tf_listener
        closest = None
        min_dist = None
        if type =="object":
            accepted_objects = ["dock", "buoy", "scan_buoy", "yellow_totem", "black_totem", "blue_totem", "green_totem", "red_totem", "polyform_a3", "polyform_a5", "polyform_a7", "surmark46104", "surmark950400", "surmark950410"]
        elif type == "buoy":
            accepted_objects = ["buoy", "scan_buoy", "yellow_totem", "black_totem", "blue_totem", "green_totem", "red_totem", "polyform_a3", "polyform_a5", "polyform_a7", "surmark46104", "surmark950400", "surmark950410"]
        elif type == "totem":
            accepted_objects = ["yellow_totem", "black_totem", "blue_totem", "green_totem", "red_totem"]
        elif type == "polyform":
            accepted_objects= ["polyform_a3", "polyform_a5", "polyform_a7"]
        elif type == "nav":
            accepted_objects = ["surmark46104", "surmark950400","surmark950410","blue_totem", "polyform_a7", "buoy"]
        elif type == "white":
            accepted_objects = ["surmark46104"]
        elif type == "green":
            accepted_objects = ["surmark950400","blue_totem"]
        elif type == "red":
            accepted_objects = ["surmark950410"]
        else:
            accepted_objects = [type]

        for object in object_list:
            #Get distance of object
            if object.best_guess in accepted_objects and object.confidences[0]>conf_thresh:

                try:
                    (trans,rot) = tf_listener.lookupTransform(frame,object.frame_id, rospy.Time(0))
                except Exception as e:
                    rospy.logwarn("Transform Lookup Error")
                    print(e)
                    continue

                if object.pose.position.y > (-1.908*object.pose.position.x + 435) and not ignore_land:
                    rospy.loginfo("Found object in land: ignoring")
                    continue

                dist = math.sqrt(trans[0]**2 +trans[1]**2)
                if min_dist is None or dist<min_dist:
                    closest = object
                    min_dist = dist

        if closest is None:
            rospy.loginfo("object type :%s not found", type)
        return closest


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
