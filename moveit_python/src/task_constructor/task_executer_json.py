#!/usr/bin/env python3
import rospy
import actionlib
import json
from moveit_msgs.msg import MoveItErrorCodes
from moveit_msgs.msg import PickupAction, PickupGoal, PlaceAction, PlaceGoal
from geometry_msgs.msg import PoseStamped
from moveit_python import MoveGroupInterface, PickPlaceInterface, PlanningSceneInterface
import time
import sys
from std_msgs.msg import Float64

# Initialize ROS node
rospy.init_node('pick_and_place_node')
scene = PlanningSceneInterface("/base_link")
move_group_interface = MoveGroupInterface(group="fr10_arm", frame="world")
pick_place_interface = PickPlaceInterface(group="fr10_arm", ee_group="gripper")
pub = rospy.Publisher('/rh_p12_rn_position/command', Float64, queue_size=10)

def task(mode_list, name_list, content_list):
    for i in len(mode_list):
        target_task = name_list[i]
        content_task = content_list[i]

        if mode_list[i] == "get_joints_position":
            print("task_executer_json.py: get_joints_position")
            joint_list = []
            pos_list = []
            for joint, pos in content_task.items():
                joint_list.append(joint)
                pos_list.append(pos)
            move_group_interface.moveToJointPosition(joints=[joint_list], positions=[pos_list], tolerance=0.01, wait=True)
            time.sleep(5)

        elif mode_list[i] == "get_end_coordinate":
            print("task_executer_json.py: get_end_coordinate")
            for loc_type, coordinate in content_task.items():
                if loc_type == "position":
                    position = coordinate
                elif loc_type == "quaternion":
                    quaternion = coordinate
                else:
                    print("get_end_coordinate error")
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
            result = move_group_interface.moveToPose(pick_pose, gripper_frame=target_task, tolerance=0.01, wait=True)
            if result.error_code.val < 1:
                print(f"task_executer.py Error: {result.error_code}")
                sys.exit()

        elif mode_list[i] == "spawn_object":
            print("task_executer_json.py: spawn_object")
            axis_list = []
            value_list = []
            for axis, value in content_task.items():
                axis_list.append(axis)
                value_list.append(value)
            scene.addBox(target_task, 0.05, 0.05, 0.05, value_list[0], value_list[1], value_list[2], use_service=True)
            time.sleep(5)

        elif mode_list[i] == "attach_object":
            print("task_executer_json.py: attach_object")
            # scene.attachBox(target_task, 0.05, 0.05, 0.05, 0, 0, 0, "rh_p12_rn_tf_end")
            scene.attachBox(target_task, 0.05, 0.05, 0.05, 0, 0, 0, content_task)
            time.sleep(5)

        elif mode_list[i] == "detach_object":
            scene.removeAttachedObject(target_task)
            axis_list = []
            value_list = []
            for axis, value in content_task.items():
                axis_list.append(axis)
                value_list.append(value)
            scene.addBox(target_task, 0.05, 0.05, 0.05, value_list[0], value_list[1], value_list[2], use_service=True)
            time.sleep(5)

        elif mode_list[i] == "gripper_open":
            msg = Float64()
            msg.data = content_task
            print(f"task_executer.py: Open: {msg.data}")
            pub.publish(msg)

        elif mode_list[i] == "gripper_close":
            msg = Float64()
            if value < 0:
                msg.data = content_task
            else:
                msg.data = value
            print(f"task_executer.py: Close: {msg.data}")
            pub.publish(msg)

        elif mode_list[i] == "choose_pipeline":
            move_group_interface.setPipelineId(target_task)
            move_group_interface.setPlannerId(content_task)

        elif mode_list[i] == "choose_follow_mode":
            pass

        elif mode_list[i] == "clear_scene":
            scene.clear()

        else:
            print("Content error")

if __name__ == '__main__':
    file_path = "/home/vboxuser/catkin_ws/src/frcobot_ros/moveit_python/tasks/fr10/test.json"
    with open(file_path, 'r') as file:
        data = json.load(file)

    mode_list = []
    name_list = []
    content_list = []
    while not rospy.is_shutdown():
        if isinstance(data, list):
            for index, item in enumerate(data):
                # Assuming the first name is the first key in each item
                for timer, content in item.items():
                    # print(f"\n")
                    for mode, content2 in content.items():
                        mode_list.append(mode)
                        for content_name, content3 in content2.items():
                            name_list.append(content_name)
                            content_list.append(content3)
        break
    
    print("*"*40)
    print(mode_list)
    print("*"*40)
    print(name_list)
    print("*"*40)
    print(content_list)
    print("*"*40)
    task(mode_list, name_list, content_list)
    sys.exit()
