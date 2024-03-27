#!/usr/bin/env python3

import sys
import rospy
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg
import time

def main():
    # Initialize ROS and MoveIt
    moveit_commander.roscpp_initialize(sys.argv)
    rospy.init_node('move_robot_node', anonymous=True)

    # Create a RobotCommander object
    robot = moveit_commander.RobotCommander()

    # Create a PlanningSceneInterface object
    scene = moveit_commander.PlanningSceneInterface()

    # Create a MoveGroupCommander object for the manipulator group
    group_name = "fr10_arm"
    move_group = moveit_commander.MoveGroupCommander(group_name)

    # Set the pose goal
    pose_goal = geometry_msgs.msg.Pose()
    pose_goal.orientation.w = 1.0
    pose_goal.position.x = 0.4
    pose_goal.position.y = 0.1
    pose_goal.position.z = 0.4
    move_group.set_pose_target(pose_goal)

    plan = move_group.plan()
    time.sleep(10)
    
    # Plan and execute the motion
    plan = move_group.go(wait=True)
    move_group.stop()
    move_group.clear_pose_targets()

    # Shut down MoveIt and ROS
    moveit_commander.roscpp_shutdown()
    rospy.signal_shutdown("Done")

if __name__ == "__main__":
    main()
