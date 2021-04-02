# waypoint_followwer

A reference code for vehicle controller.

## Install Python Dependencies

```
pip2 install numpy pandas --user
```
Note, ROS runs with `python 2.x`

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

- Find 'TODO' in the waypoint_follower/controller.py for your assignment.
- Change the code freely to fit your needs.