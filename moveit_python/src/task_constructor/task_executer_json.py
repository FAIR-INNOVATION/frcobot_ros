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
from std_msgs.msg import Float64
import json

rospy.init_node('pick_and_place_node')
scene = PlanningSceneInterface("/base_link")
move_group_interface = MoveGroupInterface(group="fr10_arm", frame="world")
pick_place_interface = PickPlaceInterface(group="fr10_arm", ee_group="gripper")

pub = rospy.Publisher('/rh_p12_rn_position/command', Float64, queue_size=10)

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

def gripper_open():
    msg = Float64()
    msg.data = 0.0
    print(f"task_executer.py: Open: {msg.data}")
    pub.publish(msg)

def gripper_close(value):
    msg = Float64()
    if value < 0:
        msg.data = 0.68 # set to max
    else:
        msg.data = value
    print(f"task_executer.py: Close: {msg.data}")
    pub.publish(msg)

def task():
    #---------------------------------------------------------------------------
    x=0
    y=0.5
    z=0.2
    remove_attached("hello_box")
    remove_obj_all()
    time.sleep(5)
    spawn_obj(x,y,z)
    gripper_open()
    #---------------------------------------------------------------------------
    # OMPL Algorithm is set as default
    move_group_interface.setPipelineId("ompl")
    move_group_interface.setPlannerId("RRTConnect")
    #---------------------------------------------------------------------------
    print("task_executer.py: Move to zero positions")
    move_group_interface.moveToJointPosition(joints=["j1","j2","j3","j4","j5","j6"], positions=[0,-1.57,1.57,0,0,0], tolerance=0.01, wait=True)
    time.sleep(5)
    #---------------------------------------------------------------------------
    print("task_executer.py: pick_pose")
    # remove_collision("hello_box")
    # time.sleep(5)
    pick_pose = PoseStamped()
    pick_pose.header.frame_id = "world"
    pick_pose.pose.position.x = x
    pick_pose.pose.position.y = y
    pick_pose.pose.position.z = z
    pick_pose.pose.orientation.w = 1.0
    result = move_group_interface.moveToPose(pick_pose, gripper_frame="rh_p12_rn_tf_end", tolerance=0.01, wait=True)
    if result.error_code.val < 1:
        print(f"task_executer.py Error: {result.error_code}")
        sys.exit()
    #---------------------------------------------------------------------------
    print("task_executer.py: attach_obj")
    attach_obj()
    gripper_open()
    time.sleep(5)
    #--------------------------------------------------------------------------- 
    move_group_interface.setPipelineId("pilz_industrial_motion_planner")
    move_group_interface.setPlannerId("LIN")
    #---------------------------------------------------------------------------
    print("task_executer.py: place_pose")
    place_pose = PoseStamped()
    place_pose.header.frame_id = "world"
    place_pose.pose.position.x = x
    place_pose.pose.position.y = y
    place_pose.pose.position.z = z+0.3
    place_pose.pose.orientation.w = 1.0
    result = move_group_interface.moveToPose(place_pose, gripper_frame="rh_p12_rn_tf_end", tolerance=0.01, wait=True, max_velocity_scaling_factor=0.1, max_acceleration_scaling_factor=0.1)

    if result.error_code.val < 1:
        print(f"task_executer.py Error: {result.error_code}")
        sys.exit()
    #---------------------------------------------------------------------------
    remove_attached("hello_box")
    gripper_open()
    spawn_obj(x,y,z+0.3)
    time.sleep(5)
    #---------------------------------------------------------------------------
    move_group_interface.setPipelineId("ompl")
    move_group_interface.setPlannerId("RRTConnect")
    #---------------------------------------------------------------------------
    print("task_executer.py: Move to zero positions")
    result = move_group_interface.moveToJointPosition(joints=["j1","j2","j3","j4","j5","j6"], positions=[0,-1.57,1.57,0,0,0], tolerance=0.01, wait=True)
    time.sleep(5)
    #---------------------------------------------------------------------------
    remove_obj_all()

    print("task_executer.py: Finish!")
    

if __name__ == '__main__':
    task()
