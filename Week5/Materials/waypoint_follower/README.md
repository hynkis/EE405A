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

## For the assignment

- Find 'TODO' in the waypoint_follower/script/controller.py for your assignment.
- Design your proper steering/speed controller by tuning controller gains.
- Implement a PI controller for steering angle control.
- Change the code freely to fit your needs.
