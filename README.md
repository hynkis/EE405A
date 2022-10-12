# EE405A 2022
KAIST EE405A Electronics Design Lab.

# Week 3 - Ubuntu Installation & ROS (1)
[Lecture Note Week3](https://www.dropbox.com/s/e9p2nmgp4t0f6lj/%5BEE405%5D%20Robotics%20Operating%20System%20%28ROS%29_1.pdf?dl=0)
- Brief tips on installing Ubuntu (Linux-based OS)
- Understand the Robotics Operating System (ROS) (1)
- Install & Setup ROS
- Run ROS tutorial
- Learn ROS programming


# Week 4 - ROS (2) & Vehicle Control
[Lecture Note Week4](https://www.dropbox.com/s/05o76sm8lu2nwb5/%5BEE405A%5D%20Vehicle_Control.pdf?dl=0)
- Understand the Robotics Operating System (ROS) (2)
    - ROS tools: rviz, rosbag
    - ROS message types

- Learn how to design the vehicle controller
    - Vehicle kinematics model
    - Longitudinal controller using PID control
    - Geometry for lateral vehicle control
    - Lateral controller based on Pure Pursuit & Stanley Method

# Week 5 - Hardware & Software Configuration
- [Hardware configuration](https://www.dropbox.com/s/sju9q2fn8crvdl6/%5BEE405A%202022%5D%20Hardware_Configuration_for_RC_Car_Platform.pdf?dl=0)
    - Hardware architecture
    - Electronics
    - Chassis

- [Software configuration](https://www.dropbox.com/s/qeh6crj0ytsh6wf/%5BEE405A%202022%5D%20Software_Configuration_for_RC_Car_Platform.pdf?dl=0)
    - Software architecture
    - Setting up Jetson NX (Official demo version)
    - Install libraries (ROS, OpenCV, ...)
    - Install sensor interfaces
        - Camera (Intel Realsense D435i)
        - 2D LiDAR (Hokuyo UST-20LX)
        - IMU (myAHRS+)
    - Install control interface
        - Arduino ROS

- [Setting up Jetson NX (eMMC version)](https://www.dropbox.com/s/fsir0et88jfrp4h/%5BEE405A%5D%20Jetson_NX_new_install_tutorial.pdf?dl=0)

# Week 6 - Localization & Mapping
[Lecture Note Week6](https://www.dropbox.com/s/yi5amtnppyxztj5/%5BEE405A%5D%20Mapping%26Localization.pdf?dl=0)
- SLAM basics
    - 2-D, 3-D SLAMs supported by ROS.
    - Occupancy grid map messages to save the 2-D map
- Scan matching algorithm
    - How does the scan matching algorithm work?
- Implement 2D Localization based on Particle filter
- Implement 2D Localization based on PCL library
    - Example code
    - Real-world test
- Parametric study

