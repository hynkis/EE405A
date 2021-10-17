# Assignment for Week3
## 1. Create a ROS package.
## 2. Write a ros node (refer ‘data_pub.py’).
* Publish two data as follows:
1. Data 1
    - topic name is "/data1"
    - message type is std_msgs/Float32 (You can set the data value arbitrary.)
    - rate is 100 Hz
2. Data 2
    - topic name is "/data2"
    - message type is std_msgs/Float32MultiArray (You can set the data values arbitrary.)
    - rate is 50 Hz

## 3. Write a ros node (refer 'data_sub_and_pub.py’).
* Subscribe two data as follows:
1. Data 1
    - topic name is "/data1"
    - message type is std_msgs/Float32
2. Data 2
    - topic name is "/data2"
    - message type is std_msgs/Float32MultiArray

* Publish two data as follows:
1. Data 3
    - publish the sum of the message data array whose topic name is "/data2"
    - publish data in 100 HZ
    - topic name is "/data3"
    - message type is std_msgs/Float32

2. Data 4
    - publish the sum of the message data array whose topic name is "/data2"
    - publish data whenever it gets the message topic "/data2"
    - topic name is "/data4"
    - message type is std_msgs/Float32


## 4. Run both nodes using ‘roslaunch’.
* Create a .launch script to run the above two nodes at the same time.