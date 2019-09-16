#!/usr/bin/env python
import rospy
import tf
import scipy.cluster.hierarchy as hcluster
import numpy
import math
from nav_msgs.srv import GetMap
from nav_msgs.msg import OccupancyGrid
from vrx_msgs.msg import ObjectArray, Object
from geometry_msgs.msg import Pose, Vector3, Quaternion


THRESHOLD = rospy.get_param('threshold', 40); #Min value of a cell before it is counted
DIST_THRESH = rospy.get_param('distance_threshold',3); #Distance between clusters before it is condidered seperate
EXPIRY_TIME = rospy.get_param('expiry_time', 3) #Time to before cleaning up missing objects


class Obstacle():
    """Obstacle Class containing information and functions for different detected Obstacles"""
    def __init__(self,tf_broadcaster,tf_listener,object_server,frame_id):
        self.x = 0
        self.y = 0
        self.time = rospy.Time.now()
        self.rot = tf.transformations.quaternion_from_euler(0,0,0)
        self.object = Object()
        self.points = None
        self.radius = None
        self.tf_broadcaster = tf_broadcaster
        self.tf_listener = tf_listener
        #self.image_server = image_server
        self.best_guess_conf = 0
        self.object_server = object_server
        self.parent_frame = "map"
        self.object.pose.position.x = self.x
        self.object.pose.position.y = self.y
        self.object.pose.orientation.w = 1
        self.object.frame_id = frame_id;


    def classify(self):
        """Here is where you request the cameras to classify the buoy"""
        if self.radius < 1.6:
            self.object.types = ["buoy"]
            self.object.best_guess = "buoy"
            self.object.confidences = [1]
        elif self.radius < 15:
            self.object.types = ["dock"]
            self.object.best_guess = "dock"
            self.object.confidences = [1]
        else:
            self.object.types = ["land"]
            self.object.best_guess = "land"
            self.object.confidences = [1]



    def broadcast(self):
        """Broadcast the object via tf"""
        #self.object.pose.orientation=self.rot

        self.tf_broadcaster.sendTransform(
        (self.x,self.y,0),
        self.rot,
        rospy.Time.now(),
        self.object.frame_id,
        self.parent_frame
        )



class ObjectServer():
    def __init__(self):
        self.tf_broadcaster = tf.TransformBroadcaster()
        self.tf_listener = tf.TransformListener()
        self.pub = rospy.Publisher("objects", ObjectArray, queue_size="10")
        self.objects = []
        self.map = OccupancyGrid()
        self.cumulative_count=0


    def callback(self,my_map):
        """Callback when a map is called."""
        #print("Recieved Map")
        self.map = my_map

    def process_map(self):
        #print("Processing map");
        my_map = self.map
        points_x = []
        points_y = []
        my_data = []
        info = my_map.info
        r = 0
        c = 0
        count  = 0
        #print(my_map)
        #Put map into a list of points.
        for i in my_map.data:
            if i > THRESHOLD: #If the value of the cell is > THRESHOLD append
                points_x.append(r*info.resolution)
                points_y.append(c*info.resolution)
                my_data.append((r*info.resolution,c*info.resolution))
            r = r+1
            if r == info.width:
                r = 0
                c = c +1
        if len(my_map.data)==0:
            print("Empty Map")
            return
        #Apply a distance threshold Cluster on the objects.
        #print(my_data,thresh)
        try:
            clust = hcluster.fclusterdata(my_data, DIST_THRESH, criterion="distance")
        except Exception:
            print(my_data,thresh)
        clusters = {}
        count = 0
        for point in my_data:
            cluster_num = clust[count]
            if cluster_num not in clusters:
                clusters[cluster_num] = [point]
            else:
                clusters[cluster_num].append(point)
            count = count+1

        #Iterate through the different colusters get the average centre point distance and size
        for cluster in clusters:
            sum_x = 0
            sum_y = 0
            for point in clusters[cluster]:
                sum_x = sum_x + point[0]
                sum_y = sum_y + point[1]
            avg = (sum_x/len(clusters[cluster]), sum_y/len(clusters[cluster]))
            max_dist = 0
            for point in clusters[cluster]:
                dist = math.sqrt((avg[0] - point[0])**2 + (avg[1] - point[1])**2)
                if dist>max_dist:
                    max_dist = dist

            #Get the Distance of the x and y axis
            x = avg[0] + info.origin.position.x
            y = avg[1] + info.origin.position.y

            #If the object is close to an already found object. Consider it the same object.
            updated = False
            current_frames = []
            name = ""
            for my_obj in self.objects:
                frame_id = my_obj.object.frame_id
                current_frames.append(frame_id)
                thresh_dist = 1
                dist = math.sqrt((my_obj.x-x)**2 + (my_obj.y-y)**2)
                if (dist<thresh_dist):
                    my_obj.x = x
                    my_obj.y = y
                    my_obj.radius = max_dist
                    my_obj.points = clusters[cluster]
                    updated = True
                    name = my_obj.object.frame_id
                    my_obj.time = rospy.Time.now()
                    break
            #If it is not close to any other objects then add it as a new object.
            if updated == False:
                #print("Adding new object", self.cumulative_count)
                frame_id = str(self.cumulative_count)
                self.cumulative_count=self.cumulative_count+1
                #print(max_dist,x,y,frame_id,frame_id)
                self.add_object(clusters[cluster],max_dist,x,y,frame_id)
                #Append threw new object to the servers object list.
    def add_object(self,points,rad,x,y,frame_id):
        my_obj = Obstacle(self.tf_broadcaster, self.tf_listener ,self,frame_id)
        my_obj.x = x
        my_obj.y = y
        my_obj.radius = rad
        my_obj.points = points
        msg_obj = Object()
        #TODO Check if object frame number is being used.
        msg_obj.frame_id = frame_id
        my_obj.object = msg_obj
        self.objects.append(my_obj)


    def classify_objects(self):
        """Classify the objects found so far using appropiate cameras."""
        for i in self.objects:
            i.classify()
        #rospy.loginfo("Classifyed clusters")

    def broadcast_objects(self):
        """Broadcast the objects found"""
        objectlist = ObjectArray()
        for i in self.objects:
            i.broadcast()
            objectlist.objects.append(i.object)
            #print(i.object)
        self.pub.publish(objectlist)

    def cleanup(self):
        """Method to clean up any objects that are old"""
        expire_time = EXPIRY_TIME
        #print("Cleaning")
        for i in self.objects:
            time_diff = rospy.Time.now().secs - i.time.secs
            #print(i.object.frame_id, time_diff)
            if time_diff > expire_time:
                rospy.logdebug("Removing expired Object")
                self.objects.remove(i)



if __name__ == "__main__":
    rospy.init_node("object_server")
    object_server = ObjectServer()
    rate = rospy.Rate(5)
    sub = rospy.Subscriber("map",OccupancyGrid,object_server.callback)
    rospy.sleep(1)
    while not rospy.is_shutdown():
        object_server.process_map();
        object_server.cleanup()
        object_server.classify_objects()
        object_server.broadcast_objects()
        rate.sleep()
