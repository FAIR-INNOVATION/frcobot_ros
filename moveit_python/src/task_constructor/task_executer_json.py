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
import rospkg

def multiply_quat(q1, q2):
        w0, x0, y0, z0 = q1
        w1, x1, y1, z1 = q2
        w = w0 * w1 - x0 * x1 - y0 * y1 - z0 * z1
        x = w0 * x1 + x0 * w1 + y0 * z1 - z0 * y1
        y = w0 * y1 - x0 * z1 + y0 * w1 + z0 * x1
        z = w0 * z1 + x0 * y1 - y0 * x1 + z0 * w1
        return [x, y, z, w]

def task(mode_list, name_list, content_list):
    print("#"*80)
    print(f"Task lenght: {len(mode_list)}\n")
    print(f"Mode list: {mode_list}\n")
    print(f"Name list: {name_list}\n")
    print("#"*80)
    pipeline = None
    latest_spawn_values = None  # This will store the latest values from value_list when mode is "spawn_object"
    latest_rot = None
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
            time.sleep(3)
            print("-"*40)

        elif mode_list[i] == "end_coordinate":
            for loc_type, coordinate in content_list[i].items():
                if loc_type == "position":
                    position = coordinate
                elif loc_type == "quaternion":
                    quaternion = coordinate
                    latest_rot = coordinate
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

            latest_spawn_values = value_list
            scene.addBox(name_list[i], 0.05, 0.05, 0.05, value_list[0], value_list[1], value_list[2], value_list[3], value_list[4], value_list[5], value_list[6], use_service=True)
            time.sleep(1)
            print("-"*40)

        elif mode_list[i] == "attach_object":
            if latest_spawn_values:
                rx = latest_spawn_values[3]
                ry = latest_spawn_values[4]
                rz = latest_spawn_values[5]
                rw = latest_spawn_values[6]
                if latest_rot:
                   rx,ry,rz,rw = multiply_quat([latest_rot[3],-latest_rot[0],-latest_rot[1],-latest_rot[2]], [rw,rx,ry,rz])
            else:
                print(f"No latest_spawn_values!")

            for link_name, coordinate in content_list[i].items():
                link_name = link_name
            print(f"'{name_list[i]}' attached to '{link_name}'")
            scene.attachBox(name_list[i], 0.05, 0.05, 0.05, 0, 0, 0, rx=rx, ry=ry, rz=rz, rw=rw, link_name=link_name)
            time.sleep(1)
            print("-"*40)

        elif mode_list[i] == "detach_object":
            scene.removeAttachedObject(name_list[i])
            time.sleep(1)
            print("-"*40)

        elif mode_list[i] == "remove_object":
            scene.removeAttachedObject(name_list[i])

        elif mode_list[i] == "gripper_open":
            msg = Float64()
            msg.data = content_list[i]
            print(f"Open: {msg.data}")
            pub.publish(msg)
            time.sleep(1)
            print("-"*40)

        elif mode_list[i] == "gripper_close":
            msg = Float64()
            msg.data = content_list[i]
            print(f"Close: {msg.data}")
            pub.publish(msg)
            time.sleep(1)
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
        if len(sys.argv) == 2:
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

        rospack = rospkg.RosPack()
        package_path = rospack.get_path('moveit_python')

        bot = moveit_commander.RobotCommander()
        if not bot.get_group_names()[0] == f"{robot}_arm":
            print(f"Wrong robot name: {robot}")
            sys.exit()

        scene = PlanningSceneInterface("/base_link")
        move_group_interface = MoveGroupInterface(group=f"{robot}_arm", frame="world")
        pick_place_interface = PickPlaceInterface(group=f"{robot}_arm", ee_group="gripper")
        pub = rospy.Publisher('/rh_p12_rn_position/command', Float64, queue_size=10)
        rate = rospy.Rate(10)

        file_path = f"{package_path}/tasks/{robot}/{mode}"
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