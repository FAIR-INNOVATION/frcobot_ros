#!/usr/bin/env python3

import rospy
import time
import os
from std_msgs.msg import Float64
from control_msgs.msg import JointControllerState
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
            "joints_position": self.joints_position,
            "end_coordinate": self.end_coordinate,
            "spawn_object": self.spawn_object,
            "attach_object": self.attach_object,
            "detach_object": self.detach_object,
            "remove_object": self.remove_object,
            "gripper_open": self.gripper_open,
            "gripper_close": self.gripper_close,
            "choose_pipeline": self.choose_pipeline,
            "choose_follow_mode": self.choose_follow_mode,
            "clear_scene": self.clear_scene,
            "check_json_files": self.check_json_files,
            "detele_json_sim_content": self.detele_json_sim_content,
        }

        action = mode_actions.get(mode)
        if action:
            action()
        else:
            print("Mode name error")
            print("Available modes:", list(mode_actions.keys()))
            sys.exit()

    def load_json(self, load_path=None):
        if load_path == None:
            load_path = f'{self.home_dir}/catkin_ws/src/frcobot_ros/moveit_python/tasks/{self.robot}/test.json'
        try:
            with open(load_path, 'r') as file:
                data = json.load(file)
        except FileNotFoundError:
            data = []
        except json.JSONDecodeError:
            # Attempt to repair or bypass corrupted JSON data
            print(f"File are corrupted: {load_path}")
            sys.exit()
        return data

    def save_json(self, data_local, save_path=None):
        if save_path == None:
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

        data.append(data_local)
        with open(save_path, 'w') as file:
            json.dump(data, file, indent=1)

    def joints_position(self):
        # if 5 param then this esle that
        if (len(self.arguments) == 3) or (len(self.arguments) == 9):
            self.subscriber = rospy.Subscriber("/joint_states", JointState, self.joints_position_callback)
            while not rospy.is_shutdown():
                if ((self.name is not None) and (self.position is not None)):
                    break
                self.rate.sleep()

            position_list = {}
            if len(self.arguments) == 3:
                for name, position in zip(self.name, self.position):
                    position_list[name] = position
            elif len(self.arguments) == 9:
                j1_to_j6 = []
                for i in range(3,len(self.arguments)):
                    j1_to_j6.append(float(self.arguments[i]))
                print(f"Joints position: {j1_to_j6}")

            self.joint_data1["positions"] = position_list
            self.joint_data2[self.mode] = self.joint_data1
            self.joint_data3[time.time()] = self.joint_data2

            self.save_json(self.joint_data3)
            print("joints_position finished")
            sys.exit()
        else:
            print("Arguments error")
            print("Example of getting current position: rosrun moveit_python task_generator.py fr10 joints_position")
            print("Example of getting desired [j1...j6]: rosrun moveit_python task_generator.py fr10 joints_position 0 0 0 0 0 0")
            sys.exit()

    def joints_position_callback(self, msg):
        self.name = msg.name
        self.position = msg.position

    def end_coordinate(self):
        if len(self.arguments) == 4:
            # Iterate over all arguments starting from the second one (index 1)
            target = str(self.arguments[len(sys.argv)-1])
            print(f"Argument is {target}")
            
            scene = PlanningSceneInterface("base_link")

            self.subscriber = rospy.Subscriber("/joint_states", JointState, self.joints_position_callback)
            link_trigger = False
            obj_trigger = False
            while not rospy.is_shutdown():
                if (self.name is not None):
                    if target in self.name:
                        link_trigger = True
                    break
                self.rate.sleep()

            scene = PlanningSceneInterface("base_link")
            if len(scene.getKnownCollisionObjects() + scene.getKnownAttachedObjects()) != 0:
                for name in scene.getKnownCollisionObjects():
                    print(f"getKnownCollisionObjects: {name}")
                    if target in name:
                        obj_trigger = True
                for name in scene.getKnownAttachedObjects():
                    print(f"getKnownAttachedObjects: {name}")
                    if target in name:
                        print(f"Error: {target} is attached. Use the link name instead.")
                        sys.exit()

            if link_trigger == True:
                listener = TransformListener()
                listener.waitForTransform("/world", f"/{target}", rospy.Time(), rospy.Duration(5.0))
                position, quaternion = listener.lookupTransform("/world", f"/{target}", rospy.Time())
                self.joint_data1[target] = {'position': position, 'quaternion': quaternion}
                self.joint_data2[self.mode] = self.joint_data1
                self.joint_data3[time.time()] = self.joint_data2
                self.save_json(self.joint_data3)
                print("end_coordinate finished")
            elif obj_trigger == True:
                data = self.load_json()
                last_spawn_object = None
                for entry in data:
                    for key, value in entry.items():
                        if ('spawn_object' in value) and (target in value['spawn_object']):
                            last_spawn_object = value['spawn_object'][target]

                # Check if a spawn_object was found and print its coordinates
                if last_spawn_object:
                    for obj_name, obj_data in last_spawn_object.items():
                        print(f"Last spawn_object coordinates:{obj_name} x={obj_data['x']}, y={obj_data['y']}, z={obj_data['z']}")
                        self.joint_data1[target] = {'position': [obj_data['x'],obj_data['y'],obj_data['z']], 'quaternion': [0,0,0,1]}
                        self.joint_data2[self.mode] = self.joint_data1
                        self.joint_data3[time.time()] = self.joint_data2
                        self.save_json(self.joint_data3)
                        print("end_coordinate finished")
                else:
                    print(f"No spawn_object with the name {target} found in the JSON data.")
            else:
                print(f"There's no name '{target}' founded.")
        else:
            print("Arguments error")
            print("Example: rosrun moveit_python task_generator.py fr10 end_coordinate rh_p12_rn_tf_end")
        sys.exit()
    
    def spawn_object(self):
        if len(self.arguments) == 7:
            xyz = [0,0,0]
            for i in range(4,len(self.arguments)):
                print(self.arguments[i])
                xyz[i-4]=float(self.arguments[i])
            scene = PlanningSceneInterface("/base_link")
            scene.addBox(self.arguments[3], 0.05, 0.05, 0.05, xyz[0], xyz[1], xyz[2], use_service=True)
            self.joint_data1[self.arguments[3]] = {'x':xyz[0],'y':xyz[1],'z':xyz[2]}
            self.joint_data2[self.mode] = self.joint_data1
            self.joint_data3[time.time()] = self.joint_data2
            self.save_json(self.joint_data3)
            print("end_coordinate finished")
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
            print("attach_object finished")
            sys.exit()
        else:
            print("Arguments error")
            print("Example: rosrun moveit_python task_generator.py fr10 attach_object hello_box rh_p12_rn_tf_end")
            sys.exit()

    def detach_object(self):
        if len(self.arguments) == 5:
            scene = PlanningSceneInterface("/base_link")
            scene.removeAttachedObject(self.arguments[3])

            target = self.arguments[4]
            print(f"Argument is {target}")
            listener = TransformListener()
            listener.waitForTransform("/world", f"/{target}", rospy.Time(), rospy.Duration(5.0))
            position, quaternion = listener.lookupTransform("/world", f"/{target}", rospy.Time())
            time.sleep(5)
            scene.addBox(self.arguments[3], 0.05, 0.05, 0.05, position[0], position[1], position[2], use_service=True)
            self.joint_data1[self.arguments[3]] = self.arguments[4]
            self.joint_data2[self.mode] = self.joint_data1
            self.joint_data3[time.time()] = self.joint_data2
            self.save_json(self.joint_data3)
            print("detach_object finished")
            sys.exit()
        else:
            print("Arguments error")
            print("Example: rosrun moveit_python task_generator.py fr10 detach_object hello_box rh_p12_rn_tf_end")
            sys.exit()
            
    def remove_object(self):
        if len(self.arguments) == 4:
            scene = PlanningSceneInterface("/base_link")
            scene.removeAttachedObject(self.arguments[3])
            print("remove_object finished")
            sys.exit()
        else:
            print("Arguments error")
            print("Example: rosrun moveit_python task_generator.py fr10 remove_object hello_box")
            sys.exit()
            

    def clear_scene(self):
        scene = PlanningSceneInterface("/base_link")
        scene.clear()
        self.joint_data1[self.mode] = 1
        self.joint_data2[self.mode] = self.joint_data1
        self.joint_data3[time.time()] = self.joint_data2
        self.save_json(self.joint_data3)
        print("choose_follow_mode finished")
        sys.exit()

    def gripper_open(self):
        pub = rospy.Publisher('/rh_p12_rn_position/command', Float64, queue_size=10)
        sub = rospy.Subscriber('/rh_p12_rn_position/state', JointControllerState, self.gripper_callback)
        msg = Float64()
        msg.data = 0.0
        rate = rospy.Rate(10)
        while not rospy.is_shutdown():
            rospy.loginfo(f"Publishing: {msg.data}")
            pub.publish(msg)
            print(f"Feedback: {self.position}")
            if self.position == msg.data:
                break
            self.rate.sleep()

        self.joint_data1[self.arguments[2]] = 0
        self.joint_data2[self.mode] = self.joint_data1
        self.joint_data3[time.time()] = self.joint_data2
        self.save_json(self.joint_data3)
        print("gripper_open finished")
        sys.exit()

    def gripper_close(self):
        pub = rospy.Publisher('/rh_p12_rn_position/command', Float64, queue_size=10)
        sub = rospy.Subscriber('/rh_p12_rn_position/state', JointControllerState, self.gripper_callback)
        msg = Float64()
        msg.data = 0.68
        while not rospy.is_shutdown():
            rospy.loginfo(f"Publishing: {msg.data}")
            pub.publish(msg)
            print(f"Feedback: {self.position}")
            if self.position == msg.data:
                break
            self.rate.sleep()

        self.joint_data1[self.arguments[2]] = 0.68
        self.joint_data2[self.mode] = self.joint_data1
        self.joint_data3[time.time()] = self.joint_data2
        self.save_json(self.joint_data3)
        print("gripper_close finished")
        sys.exit()
        
    def gripper_callback(self, msg):
        self.position = round(msg.process_value,2)

    def choose_pipeline(self):
        if (len(self.arguments) == 5) and (self.arguments[3] in ["OMPL", "PILZ"]):
            self.joint_data1[self.arguments[3]] = self.arguments[4]
            self.joint_data2[self.mode] = self.joint_data1
            self.joint_data3[time.time()] = self.joint_data2
            self.save_json(self.joint_data3)
            print("choose_pipeline finished")
            sys.exit()
        else:
            print(self.arguments[3] in ["OMPL", "PILZ"])
            print("Arguments error")
            print("Example: rosrun moveit_python task_generator.py fr10 choose_pipeline OMPL RRTConnect")
            print("Example: rosrun moveit_python task_generator.py fr10 choose_pipeline PILZ LIN")
            sys.exit()
        
    def choose_follow_mode(self):
        self.joint_data1[self.mode] = 1
        self.joint_data2[self.mode] = self.joint_data1
        self.joint_data3[time.time()] = self.joint_data2
        self.save_json(self.joint_data3)
        print("choose_follow_mode finished")
        sys.exit()

    def check_json_files(self):
        if (len(self.arguments) == 3):
            # directory = f'{self.home_dir}/catkin_ws/src/frcobot_ros/moveit_python/tasks/{self.robot}/{self.arguments[3]}'
            directory = f'{self.home_dir}/catkin_ws/src/frcobot_ros/moveit_python/tasks/{self.robot}'
            print("#"*80)
            print(f"Directory: {directory}")
            print("#"*80)
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
            print("#"*80)
            print(f"Valid JSON files: {valid_json_files}")
            print("#"*80)
            print(f"Invalid JSON files: {invalid_json_files}")
            print("#"*80)
            print("WARNING: This module can only briefly check your json file. Be sure to check everything manually.")
            print("#"*80)
            print("check_json_files finished")
            sys.exit()
        else:
            print("Arguments error")
            print("Example: rosrun moveit_python task_generator.py fr10 check_json_files")
            sys.exit()

    def detele_json_sim_content(self):
        directory = f'{self.home_dir}/catkin_ws/src/frcobot_ros/moveit_python/tasks/{self.robot}/{self.arguments[3]}'
        directory_mod = f'{self.home_dir}/catkin_ws/src/frcobot_ros/moveit_python/tasks/{self.robot}/mod_{self.arguments[3]}'
        
        data = self.load_json(load_path=directory)

        # Recursive function to remove unwanted keys
        def remove_unwanted_keys(data):
            if isinstance(data, dict):
                return {k: remove_unwanted_keys(v) for k, v in data.items() if k not in ["detach_object", "clear_scene", "attach_object", "spawn_object"]}
            elif isinstance(data, list):
                return [remove_unwanted_keys(item) for item in data]
            else:
                return data
        
        # Apply the function to remove unwanted keys
        cleaned_data = remove_unwanted_keys(data)

        # Filter out empty dictionaries
        filtered_data = [item for item in cleaned_data if any(item.values())]

        self.save_json(filtered_data, save_path=directory_mod)

        print("delete_json_sim_content finished")
        sys.exit()

if __name__ == "__main__":
    if len(sys.argv) < 3: # Assuming TaskGenerator requires three arguments plus the script name
        if (sys.argv[1]).lower() == "help":
            print("Usage example:")
            print("rosrun moveit_python task_generator.py fr10 joints_position")
            print("rosrun moveit_python task_generator.py fr10 joints_position 0 0 0 0 0 0")
            print("rosrun moveit_python task_generator.py fr10 end_coordinate rh_p12_rn_tf_end")
            print("rosrun moveit_python task_generator.py fr10 end_coordinate hello_box")
            print("rosrun moveit_python task_generator.py fr10 spawn_object hello_box 0 0.5 0.2")
            print("rosrun moveit_python task_generator.py fr10 attach_object hello_box rh_p12_rn_tf_end")
            print("rosrun moveit_python task_generator.py fr10 detach_object hello_box rh_p12_rn_tf_end")
            print("rosrun moveit_python task_generator.py fr10 remove_object hello_box")
            print("rosrun moveit_python task_generator.py fr10 clear_scene")
            print("rosrun moveit_python task_generator.py fr10 gripper_open")
            print("rosrun moveit_python task_generator.py fr10 gripper_close")
            print("rosrun moveit_python task_generator.py fr10 choose_pipeline OMPL RRTConnect")
            print("rosrun moveit_python task_generator.py fr10 choose_pipeline PILZ LIN")
            print("rosrun moveit_python task_generator.py fr10 choose_follow_mode")
            print("rosrun moveit_python task_generator.py fr10 check_json_files")
            print("rosrun moveit_python task_generator.py fr10 detele_json_sim_content test.json")
            sys.exit()
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
