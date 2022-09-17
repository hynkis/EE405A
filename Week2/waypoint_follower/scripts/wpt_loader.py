#!/usr/bin/env python
import numpy as np
import os
import pandas as pd
from datetime import datetime

import roslib
import rospy
import rospkg
import math
import tf
from tf.transformations import euler_from_quaternion
from visualization_msgs.msg import MarkerArray, Marker
from geometry_msgs.msg import Point

rospack = rospkg.RosPack()
WPT_CSV_PATH = rospack.get_path('waypoint_follower') + "/wpt_data/wpt_data_dense.csv"

# Load wpt file
csv_data = pd.read_csv(WPT_CSV_PATH, sep=',', header=None)
wpts_x = csv_data.values[:,0]
wpts_y = csv_data.values[:,1]
wpts_length = len(wpts_x)

# ROS init
rospy.init_node('wpt_loader')
marker_pub = rospy.Publisher('waypoint_markers', Marker, queue_size=10)
listener = tf.TransformListener()
rate = rospy.Rate(1.0)

while not rospy.is_shutdown():
    # Publish wpt as MarkerArray
    msg_wpts = Marker()
    msg_wpts.header.frame_id= "/odom"
    msg_wpts.header.stamp= rospy.Time.now()
    msg_wpts.ns= "spheres"
    msg_wpts.action= Marker.ADD
    msg_wpts.pose.orientation.w= 1.0

    msg_wpts.id = 0
    msg_wpts.type = Marker.SPHERE_LIST

    # POINTS markers use x and y scale for width/height respectively
    msg_wpts.scale.x = 0.1
    msg_wpts.scale.y = 0.1
    msg_wpts.scale.z = 0.1

    # Points are green
    msg_wpts.color.a = 1.0      # Don't forget to set the alpha!
    msg_wpts.color.r = 0/255.
    msg_wpts.color.g = 255/255.
    msg_wpts.color.b = 255/255.

    msg_point_list = []

    for i in range(wpts_length):
        msg_point = Point()
        msg_point.x = wpts_x[i]
        msg_point.y = wpts_y[i]
        msg_point_list.append(msg_point)

    msg_wpts.points = msg_point_list
    marker_pub.publish(msg_wpts)

    rate.sleep()
