#!/usr/bin/env python
import numpy as np
import os
import math
import pandas as pd
from datetime import datetime

import roslib
import rospy
import rospkg

from nav_msgs.msg import Odometry

import tf
from tf.transformations import euler_from_quaternion

# Params
WPTS_GAP = 0.1 # [m]

# Variables
last_ego_pose = []

# Waypoint recorder init
now = datetime.now()
filename = 'wpt_data/' + str(now.year) + "_" + str(now.month) + "_" + str(now.day) + "_" + str(now.hour) + "_" + str(now.minute) + ".csv"
pkg_path = rospkg.RosPack().get_path('waypoint_recorder')
# WPT_CSV_PATH = pkg_path + filename
WPT_CSV_PATH = os.path.join(pkg_path, filename)
print("WPT_CSV_PATH :", WPT_CSV_PATH)

if os.path.isfile(WPT_CSV_PATH):
    wpt_csv_data = pd.read_csv(WPT_CSV_PATH, sep=',', header=None)
else:
    wpt_csv_data = np.array([[]])

def calc_dist(tx, ty, ix, iy):
    return math.sqrt( (tx-ix)**2 + (ty-iy)**2 )

def callback_odom(msg):
    """
    Subscribe Odometry message
    ref: http://docs.ros.org/en/melodic/api/nav_msgs/html/msg/Odometry.html
    """
    global wpt_csv_data, last_ego_pose
    ego_x = msg.pose.pose.position.x
    ego_y = msg.pose.pose.position.y
    # get euler from quaternion
    q = msg.pose.pose.orientation
    q_list = [q.x, q.y, q.z, q.w]
    _, _, ego_yaw = euler_from_quaternion(q_list)

    print("ego_x: {}, ego_y: {}, ego_yaw: {}".format(round(ego_x, 3), round(ego_y, 3), round(np.rad2deg(ego_yaw), 3)))

    # Waypoint recorder
    wpt_payload = np.array([[ego_x, ego_y]])

    if wpt_csv_data.shape[1] == 0:
        # if csv data is empty
        wpt_csv_data = np.append(wpt_csv_data, wpt_payload, axis=1)
        last_ego_pose = [ego_x, ego_y] # update data
    else:
        # if csv data is not empty,
        # Append wpt data whenever agent has driven 0.1 m from before
        dist_driven = calc_dist(last_ego_pose[0], last_ego_pose[1], ego_x, ego_y)
        print("dist_driven :", dist_driven)
        if dist_driven < WPTS_GAP:
            pass
        else:
            wpt_csv_data = np.append(wpt_csv_data, wpt_payload, axis=0)
            # update data
            last_ego_pose = [ego_x, ego_y]

    # Save as csv
    dataframe = pd.DataFrame(wpt_csv_data)
    dataframe.to_csv(WPT_CSV_PATH, header=False, index=False)


# ROS init
rospy.init_node('wpt_recorder')
listener = tf.TransformListener()
rate = rospy.Rate(100.0)

# Subscriber
rospy.Subscriber('/car_1/ground_truth', Odometry, callback_odom)


while not rospy.is_shutdown():
    rate.sleep()


