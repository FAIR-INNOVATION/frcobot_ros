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
from geometry_msgs.msg import PoseStamped
from moveit_python import MoveGroupInterface
from moveit_python import PlanningSceneInterface
import moveit_commander
import numpy as np
import math

END_COORDINATE = "tf_end"

######################################################################################################
class TaskGenerator():
    def __init__(self, robot_name, mode, *argv):
        rospy.init_node('task_geberator_node', anonymous=True)
        
        if mode not in ["check_json_files", "detele_json_sim_content", "detele_json_temp"]:
            self.bot = moveit_commander.RobotCommander()
            if robot_name == "robot":
                robot_name = self.bot.get_group_names()[0]
                robot_name = robot_name.replace("_arm", "")
            elif not self.bot.get_group_names()[0] == f"{robot_name}_arm":
                print(f"Wrong robot name: {robot_name}")
                sys.exit()

        print(f"task_geberator_node: | robot:{robot_name} | mode:{mode} |")
        self.arguments = sys.argv
        self.robot = robot_name
        self.home_dir = os.path.expanduser('~')
        self.mode = mode
        self.task_executer = True # Set the to False if you want to disable execution to manipulator
        self.joint_data1 = {}
        self.joint_data2 = {}
        self.joint_data3 = {}
        self.name = None
        self.position = None
        self.rate = rospy.Rate(10)

        mode_actions = {
            "get_robot_param": self.get_robot_param,
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
            "detele_json_temp": self.detele_json_temp,
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
                if os.stat(save_path).st_size == 0:
                    data = []
                else:
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

    def get_robot_param(self):
        if (len(self.arguments) == 3):
            print("*"*80)
            print("get_planning_frame:")
            print(self.bot.get_planning_frame())

            print("*"*80)
            print("get_robot_markers:")
            print(self.bot.get_robot_markers())

            print("*"*80)
            print("get_root_link:")
            print(self.bot.get_root_link())

            print("*"*80)
            print("get_active_joint_names:")
            print(self.bot.get_active_joint_names())

            print("*"*80)
            print("get_joint_names:")
            print(self.bot.get_joint_names())

            print("*"*80)
            print("get_link_names:")
            print(self.bot.get_link_names())

            print("*"*80)
            print("get_group_names:")
            print(self.bot.get_group_names())

            print("*"*80)
            print("get_current_state:")
            print(self.bot.get_current_state())

            print("*"*80)
            print("get_current_variable_values:")
            print(self.bot.get_current_variable_values())
        else:
            print("Arguments error")
            print("Example: rosrun moveit_python task_generator.py robot get_robot_param")
        sys.exit()

    def joints_position_callback(self, msg):
        self.name = msg.name
        self.position = msg.position

    def joints_position(self):
        if (len(self.arguments) == 3) or (len(self.arguments) == 9):
            self.subscriber = rospy.Subscriber("/joint_states", JointState, self.joints_position_callback)
            while not rospy.is_shutdown():
                if ((self.name is not None) and (self.position is not None)):
                    break
                self.rate.sleep()

            position_list = {}
            links_number = 0
            if len(self.arguments) == 3:
                print(self.position)
                for name, position in zip(self.name, self.position):
                    links_number +=1
                    if links_number > 6:
                        break
                    position_list[name] = position
            elif len(self.arguments) == 9:
                name_list = []
                task_joints = []
                task_positions = []
                for name, position in zip(self.name, self.position):
                    name_list.append(name)
                for i in range(3,len(self.arguments)):
                    position_list[name_list[i-3]] = float(self.arguments[i])
                    task_joints.append(name_list[i-3])
                    task_positions.append(float(self.arguments[i]))
                if self.task_executer:
                    group_names = self.bot.get_group_names()
                    move_group_interface = MoveGroupInterface(group=group_names[0], frame="world")
                    move_group_interface.moveToJointPosition(joints=task_joints, positions=task_positions, tolerance=0.01, wait=True)
                    time.sleep(5)
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

    def vector_to_quaternion(self, point1, point2):
        vector = np.array(point2)  - np.array(point1)
        vector_norm = vector / np.linalg.norm(vector)
        print(vector_norm)
        if abs(vector[0]) == 1.0:
            z_axis = np.array([1, 0, 0])
        elif abs(vector[1]) == 1.0:
            z_axis = np.array([-abs(vector[1]), 0, 0])
        elif abs(vector[2]) == 1.0:
            z_axis = np.array([0, 0, -abs(vector[1])])
        else:
            z_axis = np.array([-1, 0, 0])
        cross_product = np.cross(vector_norm, z_axis)
        dot_product = np.dot(vector_norm, z_axis)

        w = 1 + dot_product
        x = cross_product[0]
        y = cross_product[1]
        z = cross_product[2]

        norm = math.sqrt(w**2 + x**2 + y**2 + z**2)
        if norm < 1e-6:
            print("Warning: norm is very close to zero. Returning a default quaternion.")
            return [0, 0, 0, 1] # Return a default quaternion
        
        quaternion = np.array([w, x, y, z]) / norm
        return quaternion
    
    def convert2right_vector(self, point1, point2):
        # Calculate the vector from point1 to point2
        vector = np.array(point2)  - np.array(point1)
        vector_list = []
        for i in range(0,3):
            if i == np.argmax(abs(vector)):
                vector_list.append(vector[i]/abs(vector[i]))
            else:
                vector_list.append(0)
        # Return the vector and the nearest normal vector
        return vector_list
    
    def multiply_quat(self, q1, q2):
            w0, x0, y0, z0 = q1
            w1, x1, y1, z1 = q2
            w = w0 * w1 - x0 * x1 - y0 * y1 - z0 * z1
            x = w0 * x1 + x0 * w1 + y0 * z1 - z0 * y1
            y = w0 * y1 - x0 * z1 + y0 * w1 + z0 * x1
            z = w0 * z1 + x0 * y1 - y0 * x1 + z0 * w1
            return [x, y, z, w]

    def rot_modify(self, dx,dy,dz,rx,ry,rz,rw, rotation_mode, rotation_data):
        point1 = (0, 0, 0)
        point2 = (dx, dy, dz)

        if rotation_mode==None:
            x = rx
            y = ry
            z = rz
            w = rw
        elif rotation_data==None:
            print("Error! 'rotation_data' is None")
            sys.exit()
        elif rotation_mode == "right_angle":
            # rotation_data = [[0,0,0],[1,1,1]]
            point1 = (rotation_data[0][0], rotation_data[0][1], rotation_data[0][2])
            point2 = (rotation_data[1][0], rotation_data[1][1], rotation_data[1][2])

            vector = self.convert2right_vector(point1,point2)
            quat = self.vector_to_quaternion(point1,vector)
            if (math.sqrt(dx*dx+dy*dy) < 0.4) and (dz < 0.3):
                if (dx > -0.3) and (dx < 0.3):
                    x = 0.5
                    y = 0.5
                    z = -0.5
                    w = 0.5 
                else:
                    x = 0
                    y = 0.7071078
                    z = 0
                    w = 0.7071078
            else:
                x = quat[1]
                y = quat[2]
                z = quat[3]
                w = quat[0]
        elif rotation_mode == "vector2point":
            # rotation_data = [[0,0,0],[1,0,0]]
            quaternion = self.vector_to_quaternion(rotation_data[0],rotation_data[1])
            x = quaternion[1]
            y = quaternion[2]
            z = quaternion[3]
            w = quaternion[0]
        else:
            print("rotation_mode error")
            sys.exit()
        return [x,y,z,w]
    
    def bot_move(self,bot_group_name, frame ,x,y,z,rx,ry,rz,rw):
        # rotation_mode= "None" or "right_angle" or "vector2point"
        if self.task_executer:
            move_group_interface = MoveGroupInterface(group=bot_group_name, frame="world")
            pick_pose = PoseStamped()
            pick_pose.header.frame_id = "world"
            pick_pose.pose.position.x = x
            pick_pose.pose.position.y = y
            pick_pose.pose.position.z = z
            pick_pose.pose.orientation.x = rx
            pick_pose.pose.orientation.y = ry
            pick_pose.pose.orientation.z = rz
            pick_pose.pose.orientation.w = rw
            result = move_group_interface.moveToPose(pick_pose, gripper_frame=frame, tolerance=0.01, wait=True)
            if result.error_code.val < 1:
                print(f"task_executer.py Error: {result.error_code}")
                sys.exit()

    def end_coordinate(self):
        if len(self.arguments) == 4:
            # Iterate over all arguments starting from the second one (index 1)
            target = str(self.arguments[len(sys.argv)-1])
            print(f"Argument is {target}")
            
            scene = PlanningSceneInterface("base_link")
            bot_link_names = self.bot.get_link_names()
            bot_group_names = self.bot.get_group_names()
            print("Link names:", bot_link_names)

            self.subscriber = rospy.Subscriber("/joint_states", JointState, self.joints_position_callback)
            link_trigger = False
            obj_trigger = False
            while not rospy.is_shutdown():
                if (self.name is not None):
                    if target in bot_link_names:
                        link_trigger = True
                    break
                self.rate.sleep()
            print(self.name)
            scene = PlanningSceneInterface("base_link")
            if (len(scene.getKnownCollisionObjects() + scene.getKnownAttachedObjects()) != 0) and not link_trigger:
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
                pos, quat = listener.lookupTransform("/world", f"/{target}", rospy.Time())
                mod_quad = self.rot_modify(pos[0],pos[1],pos[2],quat[0],quat[1],quat[2],quat[3], rotation_mode="right_angle", rotation_data = [[0,0,0],[pos[0],pos[1],pos[2]]])
                self.bot_move(bot_group_names[0],target,pos[0],pos[1],pos[2],mod_quad[0],mod_quad[1],mod_quad[2],mod_quad[3])
                self.joint_data1[target] = {'position': pos, 'quaternion': quat}
                self.joint_data2[self.mode] = self.joint_data1
                self.joint_data3[time.time()] = self.joint_data2
                self.save_json(self.joint_data3)
                print("end_coordinate finished")
            elif obj_trigger == True:
                data = self.load_json()
                last_obj = None
                for entry in data:
                    for key, value in entry.items():
                        if ('spawn_object' in value) and (target in value['spawn_object']):
                            last_obj = value['spawn_object'][target]
                # Check if a spawn_object was found and print its coordinates
                if last_obj:
                    mod_quad = self.rot_modify(last_obj['x'],last_obj['y'],last_obj['z'],last_obj['rx'],last_obj['ry'],last_obj['rz'],last_obj['rw'], rotation_mode="right_angle", rotation_data = [[0,0,0],[last_obj['x'],last_obj['y'],last_obj['z']]])
                    self.bot_move(bot_group_names[0],END_COORDINATE,last_obj['x'],last_obj['y'],last_obj['z'],mod_quad[0],mod_quad[1],mod_quad[2],mod_quad[3])
                    print(f"Last spawn_object coordinates: {target} x={last_obj['x']}, y={last_obj['y']}, z={last_obj['z']}")
                    self.joint_data1[END_COORDINATE] = {'position': [last_obj['x'],last_obj['y'],last_obj['z']], 'quaternion': [mod_quad[0],mod_quad[1],mod_quad[2],mod_quad[3]]}
                    self.joint_data2[self.mode] = self.joint_data1
                    self.joint_data3[time.time()] = self.joint_data2
                    self.save_json(self.joint_data3)
                    print("end_coordinate finished")
                else:
                    print(f"No spawn_object with the name {target} found in the JSON data.")
            else:
                print(f"There's no name '{target}' founded.")
        elif len(self.arguments) == 11:
            bot_link_names = self.bot.get_link_names()
            bot_group_names = self.bot.get_group_names()
            target = str(self.arguments[3])

            link_trigger = False
            if target in bot_link_names:
                link_trigger = True
            if link_trigger:
                pos = []
                quat = []
                for i in range(4,7):
                    pos.append(float(self.arguments[i]))
                for i in range(7,11):
                    quat.append(float(self.arguments[i]))
                print(f"Argument is {target}")
                self.bot_move(bot_group_names[0],target,pos[0],pos[1],pos[2],quat[0],quat[1],quat[2],quat[3])
                self.joint_data1[END_COORDINATE] = {'position': [pos[0],pos[1],pos[2]], 'quaternion': [quat[0],quat[1],quat[2],quat[3]]}
                self.joint_data2[self.mode] = self.joint_data1
                self.joint_data3[time.time()] = self.joint_data2
                self.save_json(self.joint_data3)
                print("end_coordinate finished")
            else:
                print(f"There's no name '{target}' founded.")
        else:
            print("Arguments error")
            print("Example: rosrun moveit_python task_generator.py fr10 end_coordinate tf_end")
            print("Example: rosrun moveit_python task_generator.py fr10 end_coordinate tf_end 0 0.3 0.2 0 0 0 1")
            print("Example: rosrun moveit_python task_generator.py fr10 end_coordinate hello_box")
        sys.exit()
    
    def spawn_object(self):
        if len(self.arguments) == 11:
            xyz = [0,0,0]
            rot = [0,0,0,1]
            for i in range(4,7):
                xyz[i-4]=float(self.arguments[i])
            for i in range(7,11):
                rot[i-7]=float(self.arguments[i])
            print(f"Spawn object: {self.arguments[3]} {xyz} {rot}")
            if self.task_executer:
                scene = PlanningSceneInterface("/base_link")
                scene.addBox(self.arguments[3], 0.05, 0.05, 0.05, xyz[0], xyz[1], xyz[2], rot[0], rot[1], rot[2], rot[3], use_service=True)
            self.joint_data1[self.arguments[3]] = {'x':xyz[0],'y':xyz[1],'z':xyz[2], 'rx':rot[0],'ry':rot[1],'rz':rot[2], 'rw':rot[3]}
            self.joint_data2[self.mode] = self.joint_data1
            self.joint_data3[time.time()] = self.joint_data2
            self.save_json(self.joint_data3)
            print("end_coordinate finished")
            sys.exit()
        else:
            print("Arguments error")
            print("Example: rosrun moveit_python task_generator.py fr10 spawn_object hello_box 0 0.5 0.2 0 0 0 1")
            sys.exit()

    def attach_object(self):
        if len(self.arguments) == 5:
            if self.task_executer:
                scene = PlanningSceneInterface("/base_link")
                if (len(scene.getKnownCollisionObjects()) == 0):
                    print("Error! No object to attach")
                    sys.exit()
                data = self.load_json()
                last_obj = None
                for entry in data:
                    for key, value in entry.items():
                        if ('spawn_object' in value) and (self.arguments[3] in value['spawn_object']):
                            last_obj = value['spawn_object'][self.arguments[3]]
                if last_obj is None:
                    print(f"There's no {self.arguments[3]} spawned")
                    sys.exit()
                scene.attachBox(self.arguments[3], 0.05, 0.05, 0.05, 0, 0, 0, rx=last_obj['rx'], ry=last_obj['ry'], rz=last_obj['rz'], rw=last_obj['rw'], link_name=END_COORDINATE)
            self.joint_data1[self.arguments[3]] = self.arguments[4]
            self.joint_data2[self.mode] = self.joint_data1
            self.joint_data3[time.time()] = self.joint_data2
            self.save_json(self.joint_data3)
            print("attach_object finished")
            sys.exit()
        else:
            print("Arguments error")
            print("Example: rosrun moveit_python task_generator.py fr10 attach_object hello_box tf_end")
            sys.exit()

    def detach_object(self):
        if len(self.arguments) == 5:
            if self.task_executer:
                scene = PlanningSceneInterface("/base_link")
                if (len(scene.getKnownAttachedObjects()) == 0):
                    print("Error! No attached object")
                    sys.exit()
                data = self.load_json()
                last_obj = None
                for entry in data:
                    for key, value in entry.items():
                        if ('spawn_object' in value) and (self.arguments[3] in value['spawn_object']):
                            last_obj = value['spawn_object'][self.arguments[3]]
                if last_obj is None:
                    print(f"There's no {self.arguments[3]} spawned")
                    sys.exit()
                scene.removeAttachedObject(self.arguments[3])
                target = self.arguments[4]
                print(f"Argument is {target}")
                listener = TransformListener()
                listener.waitForTransform("/world", f"/{target}", rospy.Time(), rospy.Duration(5.0))
                pos, quat = listener.lookupTransform("/world", f"/{target}", rospy.Time())
                quat = self.multiply_quat([quat[3], quat[0], quat[1], quat[2]], [last_obj['rw'], last_obj['rx'], last_obj['ry'], last_obj['rz']])
                time.sleep(5)
                scene.addBox(self.arguments[3], 0.05, 0.05, 0.05, pos[0], pos[1], pos[2], quat[0], quat[1], quat[2], quat[3], use_service=True)
            self.joint_data1[self.arguments[3]] = self.arguments[4]
            self.joint_data2[self.mode] = self.joint_data1
            self.joint_data3[time.time()] = self.joint_data2
            self.save_json(self.joint_data3)
            self.joint_data1 = {}
            self.joint_data2 = {}
            self.joint_data3 = {}
            self.joint_data1[self.arguments[3]] = {'x':pos[0],'y':pos[1],'z':pos[2], 'rx':quat[0],'ry':quat[1],'rz':quat[2], 'rw':quat[3]}
            self.joint_data2["spawn_object"] = self.joint_data1
            self.joint_data3[time.time()] = self.joint_data2
            self.save_json(self.joint_data3)
            print("detach_object finished")
            sys.exit()
        else:
            print("Arguments error")
            print("Example: rosrun moveit_python task_generator.py fr10 detach_object hello_box tf_end")
            sys.exit()
            
    def remove_object(self):
        if len(self.arguments) == 4:
            if self.task_executer:
                scene = PlanningSceneInterface("/base_link")
                scene.removeAttachedObject(self.arguments[3])
            self.joint_data1[self.arguments[3]] = 1
            self.joint_data2[self.mode] = self.joint_data1
            self.joint_data3[time.time()] = self.joint_data2
            self.save_json(self.joint_data3)
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
        print("clear_scene finished")
        sys.exit()

    def gripper_open(self):
        value = 0.0
        if self.task_executer:
            pub = rospy.Publisher('/rh_p12_rn_position/command', Float64, queue_size=10)
            sub = rospy.Subscriber('/rh_p12_rn_position/state', JointControllerState, self.gripper_callback)
            msg = Float64()
            msg.data = value
            last_time = time.time()
            while not rospy.is_shutdown():
                rospy.loginfo(f"Publishing: {msg.data}")
                pub.publish(msg)
                print(f"Feedback: {self.position}")
                if self.position is None:
                    if time.time() - last_time > 0.5:
                        print("No gripper founded")
                        sys.exit()
                if self.position == msg.data:
                    break
                self.rate.sleep()

        self.joint_data1[self.arguments[2]] = value
        self.joint_data2[self.mode] = self.joint_data1
        self.joint_data3[time.time()] = self.joint_data2
        self.save_json(self.joint_data3)
        print("gripper_open finished")
        sys.exit()

    def gripper_close(self):
        value = 0.18 # use other number. I choose this to not collide to the object causing errors
        if self.task_executer:
            pub = rospy.Publisher('/rh_p12_rn_position/command', Float64, queue_size=10)
            sub = rospy.Subscriber('/rh_p12_rn_position/state', JointControllerState, self.gripper_callback)
            msg = Float64()
            msg.data = value
            last_time = time.time()
            while not rospy.is_shutdown():
                rospy.loginfo(f"Publishing: {msg.data}")
                pub.publish(msg)
                print(f"Feedback: {self.position}")
                if self.position is None:
                    if time.time() - last_time > 0.5:
                        print("No gripper founded")
                        sys.exit()
                if self.position == msg.data:
                    break
                self.rate.sleep()

        self.joint_data1[self.arguments[2]] = value
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
            print("Check_json_files finished!")
            sys.exit()
        else:
            print("Arguments error")
            print("Example: rosrun moveit_python task_generator.py fr10 check_json_files")
            sys.exit()

    def detele_json_temp(self):
        print("test.json and mod_test.json will be deleted? y/n")
        answer = input()
        if answer.lower() == "y":
            directory = f'{self.home_dir}/catkin_ws/src/frcobot_ros/moveit_python/tasks/{self.robot}/test.json'
            directory_mod = f'{self.home_dir}/catkin_ws/src/frcobot_ros/moveit_python/tasks/{self.robot}/mod_test.json'
            if os.path.isfile(directory):
                os.remove(directory)
                print(f"removed: {directory}")
            if os.path.isfile(directory_mod):
                os.remove(directory_mod)
                print(f"removed: {directory_mod}")
        else:
            print("Files are not deleted")
        print("detele_json_temp finished")
        sys.exit()

    def detele_json_sim_content(self):
        directory = f'{self.home_dir}/catkin_ws/src/frcobot_ros/moveit_python/tasks/{self.robot}/{self.arguments[3]}'
        if not os.path.isfile(directory):
            print(f"There's no file in the path: {directory}")
            sys.exit()
        directory_mod = f'{self.home_dir}/catkin_ws/src/frcobot_ros/moveit_python/tasks/{self.robot}/mod_{self.arguments[3]}'
        
        data = self.load_json(load_path=directory)

        # Recursive function to remove unwanted keys
        def remove_unwanted_keys(data):
            if isinstance(data, dict):
                return {k: remove_unwanted_keys(v) for k, v in data.items() if k not in ["remove_object", "detach_object", "clear_scene", "attach_object", "spawn_object"]}
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
        if len(sys.argv) == 2:
            if (sys.argv[1]).lower() == "help":
                print("Usage example:")
                print("rosrun moveit_python task_generator.py robot get_robot_param")
                print("rosrun moveit_python task_generator.py fr10 joints_position")
                print("rosrun moveit_python task_generator.py fr10 joints_position 0 0 0 0 0 0")
                print("rosrun moveit_python task_generator.py fr10 end_coordinate tf_end")
                print("rosrun moveit_python task_generator.py fr10 end_coordinate tf_end 0 0.3 0.2 0 0 0 1")
                print("rosrun moveit_python task_generator.py fr10 end_coordinate hello_box")
                print("rosrun moveit_python task_generator.py fr10 spawn_object hello_box 0 0.5 0.2 0 0.707 0 0.707")
                print("rosrun moveit_python task_generator.py fr10 attach_object hello_box tf_end")
                print("rosrun moveit_python task_generator.py fr10 detach_object hello_box tf_end")
                print("rosrun moveit_python task_generator.py fr10 remove_object hello_box")
                print("rosrun moveit_python task_generator.py fr10 clear_scene")
                print("rosrun moveit_python task_generator.py fr10 gripper_open")
                print("rosrun moveit_python task_generator.py fr10 gripper_close")
                print("rosrun moveit_python task_generator.py fr10 choose_pipeline OMPL RRTConnect")
                print("rosrun moveit_python task_generator.py fr10 choose_pipeline PILZ LIN")
                print("rosrun moveit_python task_generator.py fr10 choose_follow_mode")
                print("rosrun moveit_python task_generator.py fr10 check_json_files")
                print("rosrun moveit_python task_generator.py fr10 detele_json_sim_content test.json")
                print("rosrun moveit_python task_generator.py fr10 detele_json_temp")
                sys.exit()
        print("Error usage: rosrun moveit_python task_generator.py arg1 arg2 arg3 arg4 etc..")
        sys.exit()
    else:
        _, robot, mode, *arguments = sys.argv
        if not robot in ["fr3", "fr10", "robot"]:
            print("Error usage: arg1: [fr3, fr10, robot]")
            sys.exit()
        if not isinstance(mode, str):
            print("Error usage: arg2 is not a string")
            sys.exit()
        TaskGenerator(robot, mode, *arguments)
        rospy.spin()
