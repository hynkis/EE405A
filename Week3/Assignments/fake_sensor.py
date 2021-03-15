#!/usr/bin/env python
# license removed for brevity
import rospy
# TODO: Import message types that you need (Float32)
from std_msgs.msg import String

class FakeSensor():
    def __init__(self):
        # Init ros node
        rospy.init_node('fake_sensor', anonymous=True)

        # TODO: Define publisher

        # Define ros node rate
        self.rate = rospy.Rate(30) # 5hz

def main():
    # Create a class instance
    fake_sensor = FakeSensor()

    # Main loop
    while not rospy.is_shutdown():
        # TODO: Compute a fake sensor value in Float32

        # TODO: Publish the fake sensor message as a Float32 message

        # Rate control
        fake_sensor.rate.sleep()

if __name__ == '__main__':
    main()
