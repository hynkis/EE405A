#!/usr/bin/env python
# license removed for brevity
"""
TODO Publish two data as follows:
@ Data 1 to publish
- message topic name : "/data1"
- message type : Float32
- message rate : 100 Hz

@ Data 2 to publish
- message topic name : "/data2"
- message type : Float32MultiArray
- message rate : 50 Hz

"""

import rospy

# TODO: import libraries if you need

class DataPublisher():
    def __init__(self):
        # Init ros node
        rospy.init_node('data_pub', anonymous=True)

        # TODO: fill below if you need


def main():
    # Create a class instance
    data_pub = DataPublisher()

    # TODO: fill below if you need

    # Main loop
    while not rospy.is_shutdown():
        # TODO: fill below if you need
        pass

if __name__ == '__main__':
    main()
