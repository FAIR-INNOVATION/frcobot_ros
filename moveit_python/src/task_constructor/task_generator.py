#!/usr/bin/env python3

import rospy
import time
import os
# For GAZEBO -> Link States:
from gazebo_msgs.srv import GetEntityState
from gazebo_msgs.msg import EntityState
from geometry_msgs.msg import Pose
from geometry_msgs.msg import Point
from geometry_msgs.msg import Quaternion
# Assuming `Feedbackinfo` is a custom service, its import path might need adjustment
from platform_interfaces.srv import Feedbackinfo

# For /joint_states:
from sensor_msgs.msg import JointState
k = 180.0/3.14159265359

######################################################################################################
class saveService():
    def __init__(self):
        pass
    
    def save_param(self, folder_link, folder_name, result):
        save_path = os.path.join(folder_link, folder_name)
        with open(save_path, 'a') as f:
            f.write(result)

# Create NODE + SERVICE CLIENT for Gz Plugin (GET STATE) service:
class serviceClientGET():
    def __init__(self):
        rospy.init_node('GraspGET_node', anonymous=True)
        self.get_entity_state_service = rospy.ServiceProxy("ros2_grasp/get_entity_state", GetEntityState)
        self.feedbackinfo_service = rospy.ServiceProxy('feedbackinfo', Feedbackinfo)
    
    def GET(self, name, ref):
        req = GetEntityStateRequest(name=name, reference_frame=ref)
        return self.get_entity_state_service(req)

######################################################################################################

# Create NODE + SUBSCRIBER for /joint_states:
class CreateSubscriber():
    def __init__(self):
        rospy.init_node("r3m_SUBSCRIBER", anonymous=True)
        self.subscriber = rospy.Subscriber("joint_states", JointState, self.listener_callback)
        self.get_entity_state_service = rospy.ServiceProxy("ros2_grasp/get_entity_state", GetEntityState)
        self.feedbackinfo_service = rospy.ServiceProxy('feedbackinfo', Feedbackinfo)

    def listener_callback(self, data):
        # Process joint states data
        pass

######################################################################################################

def main():
    print(" --- Cranfield University --- ")
    print("        (c) IFRA Group        ")
    print("promavto_execution --> GET ROBOT STATE")
    print("Python script -> RobotState.py")

    # Argument parsing and other setup code goes here

    rospy.spin()

if __name__ == "__main__":
    main()