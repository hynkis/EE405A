#!/usr/bin/env python
# license removed for brevity
import rospy
# TODO: Import message types that you need (Float32)
from std_msgs.msg import String

class StateProcessor():
    def __init__(self):
        # Init ros node
        rospy.init_node('processor', anonymous=True)

        # TODO: Define publisher and subscriber
        
        # Define ros node rate
        self.rate = rospy.Rate(100) # 100hz

    def callback_fake_sensor(self, msg):
        # TODO: Parse the data in the message
        msg_data = msg.data

        # TODO: Set an arbitrary output using the subscribed data in Float32
        #       and Publish the output as a Float32 message.
        #       (You can set the processed data arbitrary.)


def main():
    # Create a class instance
    state_processor = StateProcessor()

    # Main loop
    while not rospy.is_shutdown():
        # Rate control
        state_processor.rate.sleep()

if __name__ == '__main__':
    main()
