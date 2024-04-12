#!/usr/bin/env python3

import rospy
import time
import os
from std_msgs.msg import Float64
# from gazebo_msgs.srv import GetEntityState
# from gazebo_msgs.msg import EntityState
from geometry_msgs.msg import Pose
from geometry_msgs.msg import Point
from geometry_msgs.msg import Quaternion
import json
from sensor_msgs.msg import JointState
import sys
import json
from tf import TransformListener
######################################################################################################
class TaskGenerator():
    def __init__(self, robot_name, mode, *argv):
        rospy.init_node('task_geberator_node', anonymous=True)

        print(f"task_geberator_node: | robot:{robot_name} | mode:{mode} |")
        # Check if there are any arguments passed
        if len(sys.argv) > 1:
            # Iterate over all arguments starting from the second one (index 1)
            for i in range(3, len(sys.argv)):
                print(f"Argument #{i} is {sys.argv[i]}")
        else:
            print("Task_geberator_node: No additional command-line arguments provided.")

        self.robot = robot_name
        self.home_dir = os.path.expanduser('~')
        self.mode = mode
        self.joint_data1 = {}
        self.joint_data2 = {}
        self.joint_data3 = {}
        self.name = None
        self.position = None
        self.rate = rospy.Rate(10)

        mode_actions = {
            "get_joints_position": self.get_joints_position,
            "get_end_coordinate": self.get_end_coordinate,
            "spawn_object": self.spawn_object,
            "attach_object": self.attach_object,
            "detach_object": self.detach_object,
            "gripper_open": self.gripper_open,
            "gripper_close": self.gripper_close,
            "choose_pipeline": self.choose_pipeline,
            "choose_follow_mode": self.choose_follow_mode,
            "clear_scene": self.clear_scene,
            "check_json_file": self.check_json_file
        }

        action = mode_actions.get(mode)
        if action:
            action()
        else:
            print("Mode name error")
            print("Available modes:", list(mode_actions.keys()))
            sys.exit()

    def get_joints_position(self):
        self.subscriber = rospy.Subscriber("/joint_states", JointState, self.get_joints_position_callback)
        while not rospy.is_shutdown():
            if ((self.name is not None) and (self.position is not None)):
                for name, position in zip(self.name, self.position):
                    self.joint_data1[name] = position
                self.joint_data2[self.mode] = self.joint_data1
                self.joint_data3[time.time()] = self.joint_data2

                save_path = f'{self.home_dir}/catkin_ws/src/frcobot_ros/moveit_python/tasks/{self.robot}/test.json'

                try:
                    with open(save_path, 'r') as file:
                        data = json.load(file)
                except FileNotFoundError:
                    data = []
                data.append(self.joint_data3)

                with open(save_path, 'w') as file:
                    json.dump(data, file, indent=1)

                print("get_joints_position finished")
                break

            self.rate.sleep()
        sys.exit()

    def get_joints_position_callback(self, msg):
        self.name = msg.name
        self.position = msg.position

    def get_end_coordinate(self):
        listener = TransformListener()
        listener.waitForTransform("/map", "/base_link", rospy.Time(), rospy.Duration(4.0))
        position, quaternion = listener.lookupTransform("/map", f"/{self.argv}", rospy.Time())
        self.joint_data[self.argv] = {position, quaternion}
        with open(f'{self.home_dir}/catkin_ws/src/frcobot_ros/moveit_python/tasks/{self.robot}/test.json', "+w")  as f:
            json.dump(self.joint_data, f)
        print("get_end_coordinate finished")

    def spawn_object(self):
        # self.joint_data["spawn_object"] = {box, location}
        with open(f'{self.home_dir}/catkin_ws/src/frcobot_ros/moveit_python/tasks/{self.robot}/test.json', "+w")  as f:
            json.dump(self.joint_data, f)

    def attach_object(self):
        # self.joint_data["attach_object"] = {object_name, attach_link}
        with open(f'{self.home_dir}/catkin_ws/src/frcobot_ros/moveit_python/tasks/{self.robot}/test.json', "+w")  as f:
            json.dump(self.joint_data, f)

    def detach_object(self):
        # self.joint_data["detach_object"] = {object_name, attach_link}
        with open(f'{self.home_dir}/catkin_ws/src/frcobot_ros/moveit_python/tasks/{self.robot}/test.json', "+w")  as f:
            json.dump(self.joint_data, f)

    def clear_scene(self):
        # all or specific joint
        # clear all scene
        pass

    def gripper_open(self):
        pub = rospy.Publisher('/rh_p12_rn_position/command', Float64, queue_size=10)
        msg = Float64()
        msg.data = 0.0
        rospy.loginfo(f"Publishing: {msg.data}")
        pub.publish(msg)
        self.joint_data["gripper_open"] = msg.data
        with open(f'{self.home_dir}/catkin_ws/src/frcobot_ros/moveit_python/tasks/{self.robot}/test.json', "+w")  as f:
            json.dump(self.joint_data, f)

    def gripper_close(self):
        pub = rospy.Publisher('/rh_p12_rn_position/command', Float64, queue_size=10)
        msg = Float64()
        msg.data = 0.68
        rospy.loginfo(f"Publishing: {msg.data}")
        pub.publish(msg)
        self.joint_data["gripper_close"] = msg.data
        with open(f'{self.home_dir}/catkin_ws/src/frcobot_ros/moveit_python/tasks/{self.robot}/test.json', "+w")  as f:
            json.dump(self.joint_data, f)


    def choose_pipeline(self):
        self.joint_data["choose_pipeline"] = pipeline_name
        with open(f'{self.home_dir}/catkin_ws/src/frcobot_ros/moveit_python/tasks/{self.robot}/test.json', "+w")  as f:
            json.dump(self.joint_data, f)

    def choose_follow_mode(self, folder_link, folder_name, result):
        self.joint_data["choose_follow_mode"] = true or false
        with open(f'{self.home_dir}/catkin_ws/src/frcobot_ros/moveit_python/tasks/{self.robot}/test.json', "+w")  as f:
            json.dump(self.joint_data, f)

    def check_json_file(self):
        directory = f'{self.home_dir}/catkin_ws/src/frcobot_ros/moveit_python/tasks/{self.robot}/test.json'
        invalid_json_files = []
        valid_json_files = []
        
        for filename in os.listdir(directory):
            if filename.endswith('.json'): # Ensure we're only checking JSON files
                file_path = os.path.join(directory, filename)
                try:
                    with open(file_path, 'r') as file:
                        json.load(file) # Attempt to parse the file as JSON
                    valid_json_files.append(filename)
                except ValueError as e:
                    print(f"Invalid JSON file: {filename} - Error: {e}")
                    invalid_json_files.append(filename)
        
        print(f"Valid JSON files: {valid_json_files}")
        print(f"Invalid JSON files: {invalid_json_files}")

def print_error():
    print("Error usage: rosrun moveit_python task_generator.py arg1 arg1 arg3")
    print("Available arg1: fr3, fr10")
    print("Available arg2: ")
    print("Available arg3: for specific functions")
    sys.exit()

if __name__ == "__main__":
    if len(sys.argv) < 3: # Assuming TaskGenerator requires three arguments plus the script name
        print("Error usage: rosrun moveit_python task_generator.py arg1 arg2 arg3 arg4 etc..")
        sys.exit()
    else:
        _, robot, mode, *arguments = sys.argv
        if not robot in ["fr3", "fr10"]:
            print("Error usage: arg1: [fr3, fr10]")
            sys.exit()
        if not isinstance(mode, str):
            print("Error usage: arg2 is not a string")
            sys.exit()
        TaskGenerator(robot, mode, *arguments)
        rospy.spin()


# 1 print joints position
# 2 print link* global coordinate
# 3 gripper on/off
# 4 simulated grasp attach/release
# 5 change planner solution pilz, ompl
# 6 clear scene
# 8 perseption follower mode
# 9 check json file