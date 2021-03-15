#!/usr/bin/env python
# license removed for brevity
import rospy
from std_msgs.msg import String

class ROS_pub_sub():
    def __init__(self):
        # Init ros node
        rospy.init_node('talker_listener', anonymous=True)

        # Define publisher and subscriber
        self.sub_chatter_1 = rospy.Subscriber('/chatter', String, self.callback_chatter)
        self.pub_chatter_2 = rospy.Publisher('/chatter_2', String, queue_size=10)
        self.pub_processed = rospy.Publisher('/chatter_processed', String, queue_size=10)

        # Define ros node rate
        self.rate = rospy.Rate(5) # 5hz

    def callback_chatter(self, msg):
        # Parse the string data in the message
        chat_data = msg.data
        rospy.loginfo("I heard %s", chat_data)

        # Process
        processed_chat_data = chat_data + "_processed"

        # Publish a processed message
        msg_processed = String()
        msg_processed.data = processed_chat_data
        self.pub_processed.publish(msg_processed)

def main():
    # Create a class instance
    pub_sub_node = ROS_pub_sub()

    # Main loop
    while not rospy.is_shutdown():
        # Publish a message, chatter_2
        msg_chater_2_data = "hello_world v2"
        msg_chater_2 = String()
        msg_chater_2.data = msg_chater_2_data
        pub_sub_node.pub_chatter_2.publish(msg_chater_2)
        rospy.loginfo("I sent %s", msg_chater_2_data)

        # Rate control
        pub_sub_node.rate.sleep()

if __name__ == '__main__':
    main()