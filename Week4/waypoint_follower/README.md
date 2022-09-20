# waypoint_follower

A reference code for vehicle controller.

## Download the Code

Copy & paste this package at your catkin workspace (~/catkin_ws).

Run the following command to install all ROS dependencies for the `src/` directory.
```
cd ~/catkin_ws
rosdep install --from-paths src --ignore-src -r -y
```

## Install Python Dependencies

```
pip2 install numpy pandas --user
```
Note, ROS (melodic) runs with `python 2.x`

## Running the waypoint visualizer

In another terminal, run the node.
```
rosrun waypoint_follower wpt_loader.py
```

## Running the Controller

Open a terminal and run the following node.
```
rosrun waypoint_follower controller.py
```