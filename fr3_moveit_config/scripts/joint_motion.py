#!/usr/bin/python
# -*- coding: utf-8 -*-
import rospy
import sys
import moveit_commander

class MoveItFkDemo:

    def __init__(self):
        # 初始化move_group的API
        moveit_commander.roscpp_initialize(sys.argv)

        # 初始化ROS节点
        rospy.init_node('moveit_fk_demo', anonymous=True)
 
        # 初始化需要使用move group控制的机械臂中的arm group
        self.arm = moveit_commander.MoveGroupCommander('fr3_arm')
        
        # 设置机械臂运动的允许误差值
        self.arm.set_goal_joint_tolerance(0.001)

        # 设置允许的最大速度和加速度
        self.arm.set_max_acceleration_scaling_factor(0.5)
        self.arm.set_max_velocity_scaling_factor(0.5)
        
    def move2pose(self, pose):         
        # 设置机械臂的目标位置，使用六轴的位置数据进行描述（单位：弧度）
        # joint_positions = [0.391410, -0.676384, -0.376217, 0.0, 1.052834, 0.454125]
        # arm.set_joint_value_target(joint_positions)

        # 控制机械臂运动到预设位置pose1
        self.arm.set_named_target(pose)
        self.arm.go()
        rospy.sleep(1)

        
        # 关闭并退出moveit
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

