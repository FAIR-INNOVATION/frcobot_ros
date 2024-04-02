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
import sys

rospy.init_node('pick_and_place_node')
scene = PlanningSceneInterface("/base_link")
move_group_interface = MoveGroupInterface(group="fr10_arm", frame="base_link")
pick_place_interface = PickPlaceInterface(group="fr10_arm", ee_group="gripper")

def spawn_obj(x,y,z):
    scene.addBox("hello_box", 0.05, 0.05, 0.05, x, y, z, use_service=True)

def attach_obj():
    scene.attachBox("hello_box", 0.05, 0.05, 0.05, 0, 0, 0, "rh_p12_rn_tf_end")
        
def remove_obj_all():
    scene.clear()

def remove_collis_all():
    for name in scene.getKnownCollisionObjects():
        print("Removing %s" % name)
        scene.removeCollisionObject(name, use_service=False)
    scene.waitForSync()

def remove_collision(name):
    scene.removeCollisionObject(name)

def remove_attached(name):
    scene.removeAttachedObject(name)

def task():
    #---------------------------------------------------------------------------
    x=0
    y=0.5
    z=0.1
    remove_obj_all()
    spawn_obj(x,y,z)
    #---------------------------------------------------------------------------
    # CHOOSE OMPL Algorithm RRT
    #---------------------------------------------------------------------------
    print("task_executer.py: Move to zero positions")
    move_group_interface.moveToJointPosition(joints=["j1","j2","j3","j4","j5","j6"], positions=[0,0,0,0,0,0], tolerance=0.01, wait=True)
    time.sleep(5)
    #---------------------------------------------------------------------------
    print("task_executer.py: pick_pose")
    # remove_collision("hello_box")
    # time.sleep(5)
    pick_pose = PoseStamped()
    pick_pose.header.frame_id = "base_link"
    pick_pose.pose.position.x = x
    pick_pose.pose.position.y = y
    pick_pose.pose.position.z = z
    pick_pose.pose.orientation.w = 1.0
    result = move_group_interface.moveToPose(pick_pose, gripper_frame="rh_p12_rn_tf_end", tolerance=0.01, wait=True)

    if result.error_code.val < 1:
        print(f"{result.error_code}")
        sys.exit()
    #---------------------------------------------------------------------------
    print("task_executer.py: attach_obj")
    attach_obj()
    time.sleep(5)
    #---------------------------------------------------------------------------
    # CHOOSE PILZ Algorithm LIN
    #---------------------------------------------------------------------------
    print("task_executer.py: place_pose")
    place_pose = PoseStamped()
    place_pose.header.frame_id = "base_link"
    place_pose.pose.position.x = x
    place_pose.pose.position.y = y
    place_pose.pose.position.z = z+1
    place_pose.pose.orientation.w = 1.0
    result = move_group_interface.moveToPose(place_pose, gripper_frame="rh_p12_rn_tf_end", tolerance=0.01, wait=True)

    if result.error_code.val < 1:
        print(f"{result.error_code}")
        sys.exit()
    #---------------------------------------------------------------------------
    remove_attached("hello_box")
    spawn_obj(x,y,z+1)
    time.sleep(5)
    #---------------------------------------------------------------------------
    print("task_executer.py: Move to zero positions")
    result = move_group_interface.moveToJointPosition(joints=["j1","j2","j3","j4","j5","j6"], positions=[0,0,0,0,0,0], tolerance=0.01, wait=True)
    print(result.error_code.val)
    time.sleep(5)
    #---------------------------------------------------------------------------
    remove_obj_all()
    

if __name__ == '__main__':
    task()
