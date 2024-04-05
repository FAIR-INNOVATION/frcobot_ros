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
import json
# For /joint_states:
from sensor_msgs.msg import JointState
k = 180.0/3.14159265359
import sys

######################################################################################################
class TaskGenerator():
    def __init__(self, mode):
        rospy.init_node('task_geberator_node', anonymous=True)

        
        if mode=="get_joints_position":
                self.get_joints_position()
        elif mode=="get_end_coordinate":
                self.get_end_coordinate()
        elif mode=="do_gripper":
            pass
        elif mode=="gripper_attach":
            pass
        elif mode=="gripper_detach":
            pass
        elif mode=="choose_pipeline":
            pass
        elif mode=="choose_follow_mode":
            pass
        elif mode=="clear_scene":
            pass
        elif mode=="check_json_file":
            pass
            
        self.get_entity_state_service = rospy.ServiceProxy("ros2_grasp/get_entity_state", GetEntityState)
        self.feedbackinfo_service = rospy.ServiceProxy('feedbackinfo', Feedbackinfo)
        self.get_entity_state_service = rospy.ServiceProxy("ros2_grasp/get_entity_state", GetEntityState)

    def get_joints_position(self):
        self.subscriber = rospy.Subscriber("joint_states", JointState, self.listener_callback)
        # Process joint states data
        pass

    def get_end_coordinate(self, folder_link, folder_name, result):
        # get tf transform
        save_path = os.path.join(folder_link, folder_name)
        with open(save_path, 'a') as f:
            f.write(result)

    def do_gripper(self, folder_link, folder_name, result):
        self.subscriber = rospy.Subscriber("gripper topic", JointState, self.listener_callback)
        save_path = os.path.join(folder_link, folder_name)
        with open(save_path, 'a') as f:
            f.write(result)

    def gripper_attach(self, folder_link, folder_name, result):
        self.subscriber = rospy.Subscriber("gripper topic", JointState, self.listener_callback)
        save_path = os.path.join(folder_link, folder_name)
        with open(save_path, 'a') as f:
            f.write(result)

    def gripper_detach(self, folder_link, folder_name, result):
        self.subscriber = rospy.Subscriber("gripper topic", JointState, self.listener_callback)
        save_path = os.path.join(folder_link, folder_name)
        with open(save_path, 'a') as f:
            f.write(result)

    def choose_pipeline(self, folder_link, folder_name, result):
        save_path = os.path.join(folder_link, folder_name)
        with open(save_path, 'a') as f:
            f.write(result)

    def choose_follow_mode(self, folder_link, folder_name, result):
        save_path = os.path.join(folder_link, folder_name)
        with open(save_path, 'a') as f:
            f.write(result)

    def clear_scene(self, folder_link, folder_name, result):
        save_path = os.path.join(folder_link, folder_name)
        with open(save_path, 'a') as f:
            f.write(result)

    def check_json_file(self, folder_link, folder_name, result):
        try:
            with open(file_path, 'r') as file:
                json.load(file)
            print(f"{file_path} is valid JSON.")
        except ValueError as e:
            print(f"{file_path} is not valid JSON: {e}")

        # Directory containing JSON files
        directory = '/path/to/your/json/files'

        # List to hold invalid JSON files
        invalid_files = []

        # Iterate over files in the directory
        for filename in os.listdir(directory):
            if filename.endswith('.json'):
                file_path = os.path.join(directory, filename)
                parse_json_file(file_path)


if __name__ == "__main__":
    if len(sys.argv) < 2:
        print("usage: my_node.py arg1")
    else:
        TaskGenerator(sys.argv[1])
        rospy.spin()

# 1 print joints position
# 2 print link* global coordinate
# 3 gripper on/off
# 4 simulated grasp attach/release
# 5 change planner solution pilz, ompl
# 6 clear scene
# 8 perseption follower mode
# 9 check json file