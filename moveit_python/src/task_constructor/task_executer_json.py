#!/usr/bin/env python3
import rospy
import json
from geometry_msgs.msg import PoseStamped
from moveit_python import MoveGroupInterface, PickPlaceInterface, PlanningSceneInterface
import time
import sys
from std_msgs.msg import Float64
from tf import TransformListener
import numpy as np
import moveit_commander

def task(mode_list, name_list, content_list):
    print("#"*80)
    print(f"Task lenght: {len(mode_list)}\n")
    print(f"Mode list: {mode_list}\n")
    print(f"Name list: {name_list}\n")
    print("#"*80)
    pipeline = None
    for i in range(0, len(mode_list)):
        print("+"*40)
        print(f"TASK {i+1}: {mode_list[i]} {name_list[i]} {content_list[i]}")

        if mode_list[i] == "joints_position":
            joint_list = []
            pos_list = []
            for joint, pos in content_list[i].items():
                joint_list.append(joint)
                pos_list.append(float(pos))
            move_group_interface.moveToJointPosition(joints=joint_list, positions=pos_list, tolerance=0.01, wait=True)
            time.sleep(5)
            print("-"*40)

        elif mode_list[i] == "end_coordinate":
            for loc_type, coordinate in content_list[i].items():
                if loc_type == "position":
                    position = coordinate
                elif loc_type == "quaternion":
                    quaternion = coordinate
                else:
                    print("end_coordinate error")
                    sys.exit()
            
            pick_pose = PoseStamped()
            pick_pose.header.frame_id = "world"
            pick_pose.pose.position.x = position[0]
            pick_pose.pose.position.y = position[1]
            pick_pose.pose.position.z = position[2]
            pick_pose.pose.orientation.x = quaternion[0]
            pick_pose.pose.orientation.y = quaternion[1]
            pick_pose.pose.orientation.z = quaternion[2]
            pick_pose.pose.orientation.w = quaternion[3]
            if pipeline == "pilz_industrial_motion_planner":
                result = move_group_interface.moveToPose(pick_pose, gripper_frame=name_list[i], tolerance=0.01, wait=True, max_velocity_scaling_factor=0.1, max_acceleration_scaling_factor=0.1)
            else:
                result = move_group_interface.moveToPose(pick_pose, gripper_frame=name_list[i], tolerance=0.01, wait=True)

            if result.error_code.val < 1:
                print(f"task_executer.py Error: {result.error_code}")
                sys.exit()
            print("-"*40)

        elif mode_list[i] == "spawn_object":
            axis_list = []
            value_list = []
            for axis, value in content_list[i].items():
                axis_list.append(axis)
                value_list.append(float(value))
            scene.addBox(name_list[i], 0.05, 0.05, 0.05, value_list[0], value_list[1], value_list[2], use_service=True)
            time.sleep(5)
            print("-"*40)

        elif mode_list[i] == "attach_object":
            print(f"'{name_list[i]}' attached to '{content_list[i]}'")
            scene.attachBox(name_list[i], 0.05, 0.05, 0.05, 0, 0, 0, content_list[i])
            time.sleep(5)
            print("-"*40)

        elif mode_list[i] == "detach_object":
            scene.removeAttachedObject(name_list[i])
            target = content_list[i]
            # print(f"Argument is {target}")
            listener = TransformListener()
            listener.waitForTransform("/world", f"/{target}", rospy.Time(), rospy.Duration(5.0))
            position, quaternion = listener.lookupTransform("/world", f"/{target}", rospy.Time())
            print(f"'{name_list[i]}' is detached of the '{content_list[i]}' at the coordinate: {list(np.round(position, 2))}")
            time.sleep(5)
            scene.addBox(name_list[i], 0.05, 0.05, 0.05, position[0], position[1], position[2], use_service=True)
            time.sleep(5)
            print("-"*40)

        elif mode_list[i] == "remove_object":
            scene.removeAttachedObject(name_list[i])

        elif mode_list[i] == "gripper_open":
            msg = Float64()
            msg.data = content_list[i]
            print(f"Open: {msg.data}")
            pub.publish(msg)
            print("-"*40)

        elif mode_list[i] == "gripper_close":
            msg = Float64()
            msg.data = content_list[i]
            print(f"Close: {msg.data}")
            pub.publish(msg)
            print("-"*40)

        elif mode_list[i] == "choose_pipeline":
            if (name_list[i]).lower() == "ompl":
                pipeline = "ompl"
            elif (name_list[i]).lower() == "pilz":
                pipeline = "pilz_industrial_motion_planner"
            elif (name_list[i]).lower() == "pilz_industrial_motion_planner":
                pipeline = "pilz_industrial_motion_planner"
            else:
                print("choose_pipeline error!")
                sys.exit()

            print(f"Pipeline: {pipeline}")
            print(f"Solver: {content_list[i]}")
            move_group_interface.setPipelineId(pipeline)
            move_group_interface.setPlannerId(content_list[i])
            print("-"*40)

        elif mode_list[i] == "choose_follow_mode":
            print("WARNING! Follow mode is not available in task_executor_json")
            print("-"*40)

        elif mode_list[i] == "clear_scene":
            scene.clear()
            time.sleep(2)
            print("-"*40)

        else:
            print("Content error")
            print("-"*40)
            sys.exit()
    print("Finish!")
    sys.exit()

if __name__ == '__main__':
    if len(sys.argv) < 3: # Assuming TaskGenerator requires three arguments plus the script name
        if (sys.argv[1]).lower() == "help":
            print("Usage example:")
            print("rosrun moveit_python task_executer_json.py fr10 test.json")
            sys.exit()
        print("Error usage: rosrun moveit_python task_executer_json.py folder_name file_name")
        sys.exit()
    else:
        _, robot, mode, *arguments = sys.argv
        if not robot in ["fr3", "fr10"]:
            print("Error usage: arg1: [fr3, fr10]")
            sys.exit()
        if not isinstance(mode, str):
            print("Error usage: arg2 is not a string")
            sys.exit()

        rospy.init_node('pick_and_place_node')
        bot = moveit_commander.RobotCommander()
        if not bot.get_group_names()[0] == f"{robot}_arm":
            print(f"Wrong robot name: {robot}")
            sys.exit()

        scene = PlanningSceneInterface("/base_link")
        move_group_interface = MoveGroupInterface(group=f"{robot}_arm", frame="world")
        pick_place_interface = PickPlaceInterface(group=f"{robot}_arm", ee_group="gripper")
        pub = rospy.Publisher('/rh_p12_rn_position/command', Float64, queue_size=10)
        rate = rospy.Rate(10)

        file_path = f"/home/vboxuser/catkin_ws/src/frcobot_ros/moveit_python/tasks/{robot}/{mode}"
        with open(file_path, 'r') as file:
            data = json.load(file)

        mode_list = []
        name_list = []
        content_list = []

        if isinstance(data, list):
            for index, item in enumerate(data):
                for timer, content in item.items():
                    # print(f"\n")
                    for mode, content2 in content.items():
                        mode_list.append(mode)
                        for content_name, content3 in content2.items():
                            name_list.append(content_name)
                            content_list.append(content3)

        task(mode_list, name_list, content_list)
        sys.exit()