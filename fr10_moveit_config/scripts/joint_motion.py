#!/usr/bin/python
# -*- coding: utf-8 -*-
import rospy
import sys
import moveit_commander

class MoveItFkDemo:

    def __init__(self):
        # API for initializing move_group
        moveit_commander.roscpp_initialize(sys.argv)

        # Initialize the ROS node
        rospy.init_node('moveit_fk_demo', anonymous=True)
 
        # Initialize the arm group in the robotic arm that needs to be controlled by the move group
        self.arm = moveit_commander.MoveGroupCommander('fr5_arm')
        
        # Set the allowable error value of the robot arm movement
        self.arm.set_goal_joint_tolerance(0.001)

        # Set the maximum velocity and acceleration allowed
        self.arm.set_max_acceleration_scaling_factor(0.5)
        self.arm.set_max_velocity_scaling_factor(0.5)
        
    def move2pose(self, pose):         
        # Set the target position of the robotic arm, described by six-axis position data (unit: radian)
        # joint_positions = [0.391410, -0.676384, -0.376217, 0.0, 1.052834, 0.454125]
        # arm.set_joint_value_target(joint_positions)

        # Control the movement of the robotic arm to the preset position pose1
        self.arm.set_named_target(pose)
        self.arm.go()
        rospy.sleep(1)

        
        # close and exit moveit
        # moveit_commander.roscpp_shutdown()
        # moveit_commander.os._exit(0)

if __name__ == "__main__":
    try:
        demo = MoveItFkDemo()
        while 1:
            demo.move2pose('pose1')
            demo.move2pose('pose2')
            demo.move2pose('pose3')
            demo.move2pose('pose4')
    except rospy.ROSInterruptException:
        pass

