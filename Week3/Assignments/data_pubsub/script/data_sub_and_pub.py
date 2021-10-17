#!/usr/bin/env python
# license removed for brevity
"""
TODO Subscribe two data and publish two data as follows
@ Data 1 to subscribe
- message topic name : "/data1"
- message type : Float32

@ Data 2 to subscribe
- message topic name : "/data2"
- message type : Float32MultiArray

@ Data 3 to publish
: Publish the sum of the message data array whose topic name is "/data2"
: Publish the message in 100 HZ
- message topic name : "/data3"
- message type : Float32

@ Data 4 to publish
: Publish the sum of the message data array whose topic name is "/data2"
: Publish the message whenever it gets the message ("/data2")
- message topic name : "/data4"
- message type : Float32

"""

import rospy

# TODO: import libraries if you need

class DataSubAndPub():
    def __init__(self):
        # Init ros node
        rospy.init_node('data_sub_and_pub', anonymous=True)

        # TODO: fill below if you need

    def callback_data1(self, msg):
        # TODO: get data from the topic message ("/data1")
        pass

    def callback_data2(self, msg):
        # TODO: get data from the topic message ("/data2")
        pass

    # TODO: fill below if you need

def main():
    # Create a class instance
    data_sub_and_pub = DataSubAndPub()

    # TODO: fill below if you need

    # Main loop
    while not rospy.is_shutdown():
        # TODO: fill below if you need
        pass

if __name__ == '__main__':
    main()
