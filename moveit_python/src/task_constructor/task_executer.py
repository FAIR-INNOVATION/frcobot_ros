#!/usr/bin/env python3
import rospy
import actionlib
from moveit_msgs.msg import MoveItErrorCodes
from moveit_msgs.msg import PickupAction, PickupGoal, PlaceAction, PlaceGoal
from geometry_msgs.msg import PoseStamped
from moveit_python import MoveGroupInterface
from moveit_python import PickPlaceInterface
from moveit_python import PlanningSceneInterface
import time

rospy.init_node('pick_and_place_node')
scene = PlanningSceneInterface("/base_link")
move_group_interface = MoveGroupInterface(group="fr10_arm", frame="base_link")
pick_place_interface = PickPlaceInterface(group="fr10_arm", ee_group="gripper")

def spawn_obj():
    scene.addBox("hello_box", 0.1, 0.1, 0.1, 0, 0.5, 0.1, use_service=True)

def attach_obj():
        scene.attachBox("hello_box", 0.1, 0.1, 0.1, 0, 0.5, 0.1, "rh_p12_rn_tf_end")
        
def remove_obj():
    if True:
        for name in scene.getKnownCollisionObjects():
            print("Removing %s" % name)
            scene.removeCollisionObject(name, use_service=False)
        scene.waitForSync()
    elif True:
        print("Removing %s" % args.name)
        if args.attached:
            scene.removeAttachedObject(args.name)
        else:
            scene.removeCollisionObject(args.name)
    else:
        parser.print_help()


def task():

    remove_obj()
    
    place_pose = PoseStamped()
    place_pose.header.frame_id = "base_link"
    place_pose.pose.position.x = 0
    place_pose.pose.position.y = 0.5
    place_pose.pose.position.z = 0.5
    place_pose.pose.orientation.w = 1.0

    print("task_executer.py: Move to zero position")
    move_group_interface.moveToJointPosition(joints=["j1","j2","j3","j4","j5","j6"], positions=[0,0,0,0,0,0], tolerance=0.01, wait=True)
    time.sleep(1)

    # spawn_obj()

    print("task_executer.py: pick_pose")
    pick_pose = PoseStamped()
    pick_pose.header.frame_id = "base_link"
    pick_pose.pose.position.x = 0.5
    pick_pose.pose.position.y = 0
    pick_pose.pose.position.z = 0.5
    pick_pose.pose.orientation.w = 1.0
    move_group_interface.moveToPose(pick_pose, gripper_frame="rh_p12_rn_tf_end", tolerance=0.01, wait=True)
    time.sleep(5)

    spawn_obj()

    for name in scene.getKnownCollisionObjects():
        print(name)
    for name in scene.getKnownAttachedObjects():
        print(name)
    if len(scene.getKnownCollisionObjects() + scene.getKnownAttachedObjects()) == 0:
        print("No objects in planning scene.")

    print("edrgsertg")
    attach_obj()
    time.sleep(2)

    print("task_executer.py: place_pose")
    place_pose = PoseStamped()
    place_pose.header.frame_id = "base_link"
    place_pose.pose.position.x = 0
    place_pose.pose.position.y = 0.5
    place_pose.pose.position.z = 1.0
    place_pose.pose.orientation.w = 1.0
    move_group_interface.moveToPose(place_pose, gripper_frame="rh_p12_rn_tf_end", tolerance=0.01, wait=True)
    time.sleep(1)


    for name in scene.getKnownCollisionObjects():
        print(name)
    for name in scene.getKnownAttachedObjects():
        print(name)
    if len(scene.getKnownCollisionObjects() + scene.getKnownAttachedObjects()) == 0:
        print("No objects in planning scene.")

    remove_obj()
    time.sleep(5)
    # # Define your task
    # object_name = "your_object_name"
    # grasps = [...] # Define your grasps here
    # support_surface_name = "your_support_surface_name"
    # locations = [...] # Define your place locations here

    # # Execute the pick operation
    # pick_result = pick_place_interface.pickup(object_name, grasps, support_name=support_surface_name)
    # if pick_result.error_code.val == MoveItErrorCodes.SUCCESS:
    #     rospy.loginfo("Pick succeeded")
    # else:
    #     rospy.logerr("Pick failed with error code: %d", pick_result.error_code.val)

    # # Execute the place operation
    # place_result = pick_place_interface.place(object_name, locations, support_name=support_surface_name)
    # if place_result.error_code.val == MoveItErrorCodes.SUCCESS:
    #     rospy.loginfo("Place succeeded")
    # else:
    #     rospy.logerr("Place failed with error code: %d", place_result.error_code.val)

if __name__ == '__main__':
    spawn_obj()
    task()
