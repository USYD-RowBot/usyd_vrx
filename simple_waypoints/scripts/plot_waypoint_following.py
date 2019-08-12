#!/usr/bin/python
import sys
import rospy
import rosbag
from nav_msgs.msg import Path, Odometry
import argparse
import matplotlib.pyplot as plt
import numpy as np




def get_args():
    parser = argparse.ArgumentParser(
        description="Plots the waypoints and the path followed")
    parser.add_argument(
        "bag_file", help="Path to the bag file")
    parser.add_argument(
        "--waypoints_topic", help="Waypoints topic")
    parser.add_argument(
        "--odometry_topic", help="Odometry topic")
    parser.add_argument("--plot_speed", action="store_true", help="Whether to plot speed")
    args = parser.parse_args()
    return args


def plot_waypoint_following(args):

    bag = rosbag.Bag(args.bag_file)
    wp_list = []
    odom_list = []
    speed_list = []
    if args.plot_speed:
        from rowbot_msgs.msg import Course

    for (topic, msg, t) in bag.read_messages():
        if topic == args.waypoints_topic:
            for ps in msg.poses:
                wp_list.append(np.array([ps.pose.position.x,ps.pose.position.y]))
        elif topic == args.odometry_topic:
            odom_list.append(np.array([msg.pose.pose.position.x,msg.pose.pose.position.y]))
        elif topic == "/cmd_course" and args.plot_speed:
            speed_list.append(msg.speed)
    plt.figure()
    if len(wp_list) != 0:
        wp_array = np.asarray(wp_list)
        plt.plot(wp_array[:,0], wp_array[:,1], label='Waypoints')
    if len(odom_list) != 0:
        odom_array = np.asarray(odom_list)
        plt.plot(odom_array[:,0], odom_array[:,1], label='Odometry')
    plt.legend()

    if args.plot_speed:
        plt.figure()
        plt.plot(range(len(speed_list)), speed_list)
        plt.ylim([min(speed_list)-0.5, max(speed_list)+0.5])
    plt.show()

if __name__ == '__main__':
    plot_waypoint_following(get_args())
