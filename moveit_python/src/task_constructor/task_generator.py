#!/usr/bin/env python3

import rospy
import time
import os
from std_msgs.msg import Float64
import json
from sensor_msgs.msg import JointState
import sys
import json
from tf import TransformListener
from moveit_python import PlanningSceneInterface

######################################################################################################
class TaskGenerator():
    def __init__(self, robot_name, mode, *argv):
        rospy.init_node('task_geberator_node', anonymous=True)

        print(f"task_geberator_node: | robot:{robot_name} | mode:{mode} |")
        self.arguments = sys.argv
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
            "check_json_file": self.check_json_file,
            "detele_json_sim_content": self.detele_json_sim_content,
        }

        action = mode_actions.get(mode)
        if action:
            action()
        else:
            print("Mode name error")
            print("Available modes:", list(mode_actions.keys()))
            sys.exit()

    def save_json(self, joint_data_local):
        save_path = f'{self.home_dir}/catkin_ws/src/frcobot_ros/moveit_python/tasks/{self.robot}/test.json'
        try:
            with open(save_path, 'r') as file:
                data = json.load(file)
        except FileNotFoundError:
            data = []
        except json.JSONDecodeError:
            # Attempt to repair or bypass corrupted JSON data
            print(f"File are corrupted: {save_path}")
            sys.exit()

        data.append(joint_data_local)
        with open(save_path, 'w') as file:
            json.dump(data, file, indent=1)

    def get_joints_position(self):
        self.subscriber = rospy.Subscriber("/joint_states", JointState, self.get_joints_position_callback)
        while not rospy.is_shutdown():
            if ((self.name is not None) and (self.position is not None)):
                position_list = {}
                for name, position in zip(self.name, self.position):
                    position_list[name] = position
                self.joint_data1["positions"] = position_list
                self.joint_data2[self.mode] = self.joint_data1
                self.joint_data3[time.time()] = self.joint_data2

                self.save_json(self.joint_data3)
                print("get_joints_position finished")
                break
            self.rate.sleep()
        sys.exit()

    def get_joints_position_callback(self, msg):
        self.name = msg.name
        self.position = msg.position

    def get_end_coordinate(self):
        if len(self.arguments) == 4:
            # Iterate over all arguments starting from the second one (index 1)
            target = str(self.arguments[len(sys.argv)-1])
            print(f"Argument is {self.arguments[len(sys.argv)-1]}")

            listener = TransformListener()
            listener.waitForTransform("/world", f"/{target}", rospy.Time(), rospy.Duration(5.0))
            position, quaternion = listener.lookupTransform("/world", f"/{target}", rospy.Time())
            self.joint_data1[target] = {'position': position, 'quaternion': quaternion}
            self.joint_data2[self.mode] = self.joint_data1
            self.joint_data3[time.time()] = self.joint_data2
            self.save_json(self.joint_data3)
            print("get_end_coordinate finished")
            sys.exit()
        else:
            print("Arguments error")
            print("Example: rosrun moveit_python task_generator.py fr10 get_end_coordinate rh_p12_rn_tf_end")
            sys.exit()
    
    def spawn_object(self):
        if len(self.arguments) == 7:
            xyz = [0,0,0]
            for i in range(4,len(self.arguments)):
                print(self.arguments[i])
                xyz[i-4]=self.arguments[i]
            scene = PlanningSceneInterface("/base_link")
            scene.addBox(self.arguments[3], 0.05, 0.05, 0.05, xyz[0], xyz[1], xyz[2], use_service=True)
            self.joint_data1[self.arguments[3]] = {'x':xyz[0],'y':xyz[1],'z':xyz[2]}
            self.joint_data2[self.mode] = self.joint_data1
            self.joint_data3[time.time()] = self.joint_data2
            self.save_json(self.joint_data3)
            print("get_end_coordinate finished")
            sys.exit()
        else:
            print("Arguments error")
            print("Example: rosrun moveit_python task_generator.py fr10 spawn_object hello_box 0 0.5 0.2")
            sys.exit()

    def attach_object(self):
        if len(self.arguments) == 5:
            scene = PlanningSceneInterface("/base_link")
            scene.attachBox(self.arguments[3], 0.05, 0.05, 0.05, 0, 0, 0, "rh_p12_rn_tf_end")
            self.joint_data1[self.arguments[3]] = self.arguments[4]
            self.joint_data2[self.mode] = self.joint_data1
            self.joint_data3[time.time()] = self.joint_data2
            self.save_json(self.joint_data3)
            print("get_end_coordinate finished")
            sys.exit()
        else:
            print("Arguments error")
            print("Example: rosrun moveit_python task_generator.py fr10 attach_object hello_box rh_p12_rn_tf_end")
            sys.exit()

    def detach_object(self):
        if len(self.arguments) == 5:
            # scene = PlanningSceneInterface("/base_link")
            # scene.removeAttachedObject(self.arguments[3])
            # get coordinate fo the end
            # scene.addBox(self.arguments[3], 0.05, 0.05, 0.05, x, y, z, use_service=True)
            self.joint_data1[self.arguments[3]] = self.arguments[4]
            self.joint_data2[self.mode] = self.joint_data1
            self.joint_data3[time.time()] = self.joint_data2
            self.save_json(self.joint_data3)
            print("get_end_coordinate finished")
            sys.exit()
        else:
            print("Arguments error")
            print("Example: rosrun moveit_python task_generator.py fr10 detach_object hello_box rh_p12_rn_tf_end")
            sys.exit()

    def clear_scene(self):
        scene = PlanningSceneInterface("/base_link")
        scene.clear()

    def gripper_open(self):
        pub = rospy.Publisher('/rh_p12_rn_position/command', Float64, queue_size=10)
        sub = rospy.Subscriber('/rh_p12_rn_position/state', Float64, self.gripper_callback)
        msg = Float64()
        msg.data = 0.0
        while not rospy.is_shutdown():
            rospy.loginfo(f"Publishing: {msg.data}")
            pub.publish(msg)
            if self.position == msg.data:
                break

        self.joint_data1[self.arguments[2]] = 0
        self.joint_data2[self.mode] = self.joint_data1
        self.joint_data3[time.time()] = self.joint_data2
        self.save_json(self.joint_data3)
        print("gripper_open finished")
        sys.exit()

    def gripper_close(self):
        pub = rospy.Publisher('/rh_p12_rn_position/command', Float64, queue_size=10)
        sub = rospy.Subscriber('/rh_p12_rn_position/state', Float64, self.gripper_callback)
        msg = Float64()
        msg.data = 0.68
        while not rospy.is_shutdown():
            rospy.loginfo(f"Publishing: {msg.data}")
            pub.publish(msg)
            if self.position == msg.data:
                break

        self.joint_data1[self.arguments[2]] = 0.68
        self.joint_data2[self.mode] = self.joint_data1
        self.joint_data3[time.time()] = self.joint_data2
        self.save_json(self.joint_data3)
        print("gripper_close finished")
        sys.exit()
        
    def gripper_callback(self, msg):
        self.position = round(msg.command,2)

    def choose_pipeline(self):
        if (len(self.arguments) == 5) and (not self.arguments[3] in ["OMPL", "PILZ"]):
            self.joint_data1[self.arguments[3]] = self.arguments[4]
            self.joint_data2[self.mode] = self.joint_data1
            self.joint_data3[time.time()] = self.joint_data2
            self.save_json(self.joint_data3)
            print("choose_pipeline finished")
            sys.exit()
        else:
            print("Arguments error")
            print("Example: rosrun moveit_python task_generator.py fr10 choose_pipeline OMPL RRTConnect")
            print("Example: rosrun moveit_python task_generator.py fr10 choose_pipeline PILZ LIN")
            sys.exit()
        
    def choose_follow_mode(self):
        self.joint_data2[self.mode] = 1
        self.joint_data3[time.time()] = self.joint_data2
        self.save_json(self.joint_data3)
        print("choose_follow_mode finished")
        sys.exit()

    def check_json_file(self):
        if (len(self.arguments) == 4):
            directory = f'{self.home_dir}/catkin_ws/src/frcobot_ros/moveit_python/tasks/{self.robot}/{self.arguments[3]}.json'
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
            print("check_json_file finished")
            sys.exit()
        else:
            print("Arguments error")
            print("Example: rosrun moveit_python task_generator.py fr10 check_json_file test")
            sys.exit()

    def detele_json_sim_content(self):
        directory = f'{self.home_dir}/catkin_ws/src/frcobot_ros/moveit_python/tasks/{self.robot}/{self.arguments[3]}.json'
        with open(directory, 'r') as file:
            data = json.load(file) # Attempt to parse the file as JSON
        print(data)
        
if __name__ == "__main__":
    if len(sys.argv) < 3: # Assuming TaskGenerator requires three arguments plus the script name
        print("Error usage: rosrun moveit_python task_generator.py arg1 arg2 arg3 arg4 etc..")
        sys.exit()
    else:
        _, robot, mode, *arguments = sys.argv
        if robot == "help":
            print("Usage example:")
            print("rosrun moveit_python task_generator.py fr10 get_joints_position")
            print("rosrun moveit_python task_generator.py fr10 get_end_coordinate rh_p12_rn_tf_end")
            print("rosrun moveit_python task_generator.py fr10 spawn_object hello_box 0 0.5 0.2")
            print("rosrun moveit_python task_generator.py fr10 clear_scene")
            print("rosrun moveit_python task_generator.py fr10 gripper_open")
            print("rosrun moveit_python task_generator.py fr10 gripper_close")
            print("rosrun moveit_python task_generator.py fr10 choose_pipeline OMPL RRTConnect")
            print("rosrun moveit_python task_generator.py fr10 choose_pipeline PILZ LIN")
            print("rosrun moveit_python task_generator.py fr10 choose_follow_mode")
            print("rosrun moveit_python task_generator.py fr10 check_json_file test")
            print("rosrun moveit_python task_generator.py fr10 detele_json_sim_content test")
            sys.exit()
        if not robot in ["fr3", "fr10"]:
            print("Error usage: arg1: [fr3, fr10]")
            sys.exit()
        if not isinstance(mode, str):
            print("Error usage: arg2 is not a string")
            sys.exit()
        TaskGenerator(robot, mode, *arguments)
        rospy.spin()
