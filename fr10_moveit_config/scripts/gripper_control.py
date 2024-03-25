#!/usr/bin/env python3
import rospy
from std_msgs.msg import Float64
import time

def publish_commands():
    # Initialize the ROS Node
    rospy.init_node('rh_p12_rn_command_publisher', anonymous=True)

    # Create a publisher object
    pub = rospy.Publisher('/rh_p12_rn_position/command', Float64, queue_size=10)

    # Wait for the publisher to establish connection to subscribers
    rospy.sleep(1)

    # Create and publish the first message
    msg = Float64()
    msg.data = 0.68
    rospy.loginfo(f"Publishing: {msg.data}")
    pub.publish(msg)

    # Wait a bit between messages
    time.sleep(1)

    # Create and publish the second message
    msg.data = 0.0
    rospy.loginfo(f"Publishing: {msg.data}")
    pub.publish(msg)

if __name__ == '__main__':
    try:
        publish_commands()
    except rospy.ROSInterruptException:
        pass
