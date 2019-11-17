#!/usr/bin/env python

from vrx_msgs.msg import ObjectArray, Object
from geographic_msgs.msg import GeoPoseStamped
import rospy


if __name__ == "__main__":
    rospy.init_node("perception_publisher")




class Node():
    def __init__(self):
        self.current_objects = {}


    def object_callback(self,msg):

        #Go through the list of objects
        for object in msg.objects:
            #print("PERCEPTION",object)
            if object.best_guess != "buoy" and object.confidences[0] > 0.5:
                print(object.best_guess)
                print(self.current_objects)
                if object.frame_id not in self.current_objects:
                    #Publish object
                    #print("NNEEEEEEEWWWWWWWWW")
                    #print(object)
                    self.publish(object)
                    self.current_objects[object.frame_id] = object.best_guess
                    print(object)
                elif object.best_guess != self.current_objects[object.frame_id]:
                    self.publish(object)
                    self.current_objects[object.frame_id] = object.best_guess
                    print(object)
            #is it new
                #if so, inistanty publish
                #Add it to current objects

            #if isnt new
            else:
                pass


                #Ignore

        #if current objects wasnt seend,
            #remove current object
    def publish(self,object):
        #print("perception_PUBLISHER HAS OBJECT")
        message = GeoPoseStamped()
        message.header.frame_id = object.best_guess
        self.pub.publish(message)
        print("PUBLISHED OBJECT")

if __name__ == "__main__":
    print("PERCEPTION STARTED")
    node = Node()
    sub = rospy.Subscriber("wamv/objects",ObjectArray,node.object_callback)
    node.pub = rospy.Publisher("results", GeoPoseStamped, queue_size="10")

    rospy.spin()
