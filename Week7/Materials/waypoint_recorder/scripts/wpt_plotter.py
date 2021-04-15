#!/usr/bin/env python
import numpy as np
import os
import pandas as pd
from scipy import interpolate

import matplotlib.pyplot as plt
from datetime import datetime
import time

import roslib
import rospy
import rospkg
import math

FILENAME = "2021_4_13_21_45.csv"
pkg_path = rospkg.RosPack().get_path('waypoint_recorder') + '/wpt_data'
WPT_CSV_LOAD_PATH = os.path.join(pkg_path, FILENAME)

# ----- Load wpt file
csv_data = pd.read_csv(WPT_CSV_LOAD_PATH, sep=',', header=None)
wpts_data = csv_data.values
wpts_x = wpts_data[:,0]
wpts_y = wpts_data[:,1]
wpts_i = wpts_data[:,2]
wpts_length = len(wpts_x)
print("wpts_length :", wpts_length)

# Plot waypoints
plt.figure()
plt.plot(wpts_x, wpts_y, '.b')
plt.xlabel('X')
plt.ylabel('Y')
plt.title('Recorded Waypoint Trajectory')
plt.grid()
plt.axis('equal')

plt.show()
