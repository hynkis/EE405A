# Record waypoint trajectory in simulator
Package for recording waypoint trajectory in the gazebo simulator.

## Requirements 
1. For python packages
```
pip2 install numpy --user
pip2 install pandas --user
pip2 install roslib --user
pip2 install matplotlib --user
```
2. For ROS Melodic messages
```
sudo apt-get install ros-melodic-geometry-msgs
sudo apt-get install ros-melodic-visualization-msgs
sudo apt-get install ros-melodic-nav-msgs
```
3. For Gazebo
```
sudo apt-get install ros-melodic-gazebo-ros-pkgs
sudo apt-get install ros-melodic-gazebo-ros-control
```

## Record waypoint trajectory (waypoint_recorder/recorder.py)
Subscribe the current position of ego vehicle and save the position every specific distance.
Simply run package and save waypoint in real-time.
For stop recording, turn off this package using `ctrl + c`

You can change these parameters
- WPT_CSV_PATH : the path for saving your waypoint trajectory
- WPTS_GAP     : the gap distance between each waypoint (Waypoint trajectory resolution)

```
rosrun waypoint_recorder recorder.py
```

For manual control using your keyboard inputs, run the keyboard node in the gazebo simulator package:
[https://github.com/Leedk3/EE405_a_eurecar_f1_tenth_project/blob/master/f1tenth-sim/scripts/keyboard_teleop.py](https://github.com/Leedk3/EE405_a_eurecar_f1_tenth_project/blob/master/f1tenth-sim/scripts/keyboard_teleop.py)

```
rosrun f1tenth-sim keyboard_teleop.py car_1
```

Click the terminal where the keyboard_teleop node is running, and press your keyboards to control your vehicle.

- w a d s : forward, left, right, backward
- space_bar : brack 

After recording the trajectory, you can plot the recorded data using Matplotlib.

You may need to edit and smooth your recorded trajectory by your hand, spline interpolation, etc.

Change the path for saved waypoint trajectory.
- FILENAME : the name of the saved waypoint trajectory in the directory 'waypoint_recorder/wpt_data'

```
roscd waypoint_recorder
cd scripts
python2 wpt_plotter.py
```