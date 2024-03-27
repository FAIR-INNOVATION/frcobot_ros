#!/usr/bin/env python3

import sys
import rospy
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg

def main():
    # Initialize ROS and MoveIt
    moveit_commander.roscpp_initialize(sys.argv)
    rospy.init_node("move_robot_node", anonymous=True)

    # Create a RobotCommander object
    robot = moveit_commander.RobotCommander()

    # Create a PlanningSceneInterface object
    scene = moveit_commander.PlanningSceneInterface()

    # Create a MoveGroupCommander object for the manipulator group
    move_group = moveit_commander.MoveGroupCommander('fr10_arm')

    # Set the goal joint values
    joint_goal = move_group.get_current_joint_values()
    joint_goal[0] = 1.8919 # j1
    joint_goal[1] = -1.3514 # j2
    joint_goal[2] = 1.3903 # j3
    joint_goal[3] = -2.8701 # j4
    joint_goal[4] = -1.6031 # j5
    joint_goal[5] = 0.4894 # j6

    # Execute the plan
    move_group.go(joint_goal, wait=True)

    # Stop the manipulator
    move_group.stop()

    # Shut down MoveIt and ROS
    moveit_commander.roscpp_shutdown()
    rospy.signal_shutdown("Done")

if __name__ == "__main__":
    main()
