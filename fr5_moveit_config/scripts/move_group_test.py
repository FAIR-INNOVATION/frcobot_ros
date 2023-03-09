#!/usr/bin/env python
# -*- coding: utf-8 -*-
import sys
import copy
import rospy
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg
from math import pi
from std_msgs.msg import String
from moveit_commander.conversions import pose_to_list


def all_close(goal, actual, tolerance):
    """
    用以测试actual的值是否在goal对应值的公差范围内。
    @param: goal       目标参数。浮点型列表、Pose 类型或 PoseStamped 类型消息
    @param: actual     测试参数。浮点型列表、Pose 类型或 PoseStamped 类型消息
    @param: tolerance  公差范围。浮点数
    @returns: bool
    """
    if type(goal) is list:
        for index in range(len(goal)):
            if abs(actual[index] - goal[index]) > tolerance:
                return False

    elif type(goal) is geometry_msgs.msg.PoseStamped:
        return all_close(goal.pose, actual.pose, tolerance)

    elif type(goal) is geometry_msgs.msg.Pose:
        return all_close(pose_to_list(goal), pose_to_list(actual), tolerance)

    return True


class MoveGroupTest(object):
    """MoveGroupTest"""

    def __init__(self):
        super(MoveGroupTest, self).__init__()

        # 初始化moveit_commander API和rospy节点
        moveit_commander.roscpp_initialize(sys.argv)
        rospy.init_node('move_group_test', anonymous=True)

        # 初始化RobotCommander对象，提供诸如机器人运动学模型和机器人当前关节状态等信息，机器人与外界的接口。
        robot = moveit_commander.RobotCommander()

        # 初始化PlanningSceneInterface对象，提供一个机器人与周围世界的接口。
        scene = moveit_commander.PlanningSceneInterface()

        # 初始化MoveGroupCommander对象。该对象是fr5机械臂规划组的接口，用于规划和执行机械臂运动。
        group_name = "fr5_arm"
        move_group = moveit_commander.MoveGroupCommander(group_name)

        # 创建一个用于在RViz中显示轨迹信息的ROS发布者
        display_trajectory_publisher = rospy.Publisher('/move_group/display_planned_path',
                                                       moveit_msgs.msg.DisplayTrajectory,
                                                       queue_size=20)                                                   

        # 获取机器人的参考系名称
        planning_frame = move_group.get_planning_frame()
        print "============ Planning frame: %s" % planning_frame

        # 获取当前规划组中机器人末端执行器连杆的名称
        eef_link = move_group.get_end_effector_link()
        print "============ End effector link: %s" % eef_link

        # 获取机器人所有规划组的名称，以数组形式输出
        group_names = robot.get_group_names()
        print "============ Available Planning Groups:", robot.get_group_names()

        # 获取机器人当前状态
        print "============ Printing robot state"
        print robot.get_current_state()
        print ""

        self.box_name = ''
        self.robot = robot
        self.scene = scene
        self.move_group = move_group
        self.display_trajectory_publisher = display_trajectory_publisher
        self.planning_frame = planning_frame
        self.eef_link = eef_link
        self.group_names = group_names

    def go_to_joint_state(self):

        # 规划并执行到一个关节目标位置
        # ^^^^^^^^^^^^^^^^^^^^^^^^
        # 法奥机器人初始位姿是一个奇异位姿，首先我们将其移动到一个更好的位置。
        # 先从接口中获取当前机器人的各关节值，再修改部分关节的值
        joint_goal = self.move_group.get_current_joint_values()
        print "============ Printing current joint values: ", joint_goal
        joint_goal[0] = 0
        joint_goal[1] = -pi/4
        joint_goal[2] = -pi/2
        joint_goal[3] = -pi/2
        joint_goal[4] = pi/3
        joint_goal[5] = 0
        print "============ Printing joint goal: ", joint_goal

        # 运动到关节目标位置
        self.move_group.go(joint_goal, wait=True)
        rospy.sleep(1)
        # 调用停止指令，确保没有残留运动
        self.move_group.stop()

        # For testing:
        current_joints = self.move_group.get_current_joint_values()
        return all_close(joint_goal, current_joints, 0.01)

    def go_to_pose_goal(self):

        # 规划并执行到末端目标位姿
        # ^^^^^^^^^^^^^^^^^^^^^^^
        pose_goal = geometry_msgs.msg.Pose()
        pose_goal.orientation.w = 1.0
        pose_goal.position.x = 0.4
        pose_goal.position.y = 0.1
        pose_goal.position.z = 0.4

        # 设置目标位姿
        self.move_group.set_pose_target(pose_goal)

        # 规划并执行到目标位姿
        self.move_group.go(wait=True)
        rospy.sleep(1)
        self.move_group.stop()

        # 清除目标位姿
        self.move_group.clear_pose_targets()

        # For testing:
        current_pose = self.move_group.get_current_pose().pose
        return all_close(pose_goal, current_pose, 0.01)

    def plan_cartesian_path(self, scale=1.0):

        # 规划并执行一条笛卡尔路径
        # ^^^^^^^^^^^^^^^^^^^^
        # 您可以通过指定末端执行器要通过的航路点列表来直接规划笛卡尔路径。
        waypoints = []

        wpose = self.move_group.get_current_pose().pose
        wpose.position.z -= scale * 0.1  # 按z轴向下移动
        wpose.position.y += scale * 0.2  # 按y轴向侧边移动
        waypoints.append(copy.deepcopy(wpose))

        wpose.position.x += scale * 0.1  # 按x轴向前移动
        waypoints.append(copy.deepcopy(wpose))

        wpose.position.y -= scale * 0.1  # 按y轴向侧边移动
        waypoints.append(copy.deepcopy(wpose))

        # 我们希望以1cm的分辨率插值笛卡尔路径，这就是为什么我们在笛卡尔转换中将0.01指定为eef_step的原因。 
        # 通过将其设置为0.0，可以禁用跳转阈值，而忽略对关节空间中不可行跳转的检查。
        (plan, fraction) = self.move_group.compute_cartesian_path(
            waypoints,   # waypoints to follow
            0.01,        # eef_step
            0.0)         # jump_threshold

        # 当前只是规划路径，并没有让move_group去执行并移动机器人
        return plan, fraction

    def display_trajectory(self, plan):

        # 显示运动轨迹
        # ^^^^^^^^^^
        # DisplayTrajectory msg具有两个主要字段，trajectory_start和trajectory。
        # 我们用当前的机器人状态填充trajectory_start，以复制任何AttachedCollisionObjects并将我们的计划添加到该轨迹。
        display_trajectory = moveit_msgs.msg.DisplayTrajectory()
        display_trajectory.trajectory_start = self.robot.get_current_state()
        display_trajectory.trajectory.append(plan)
        # 发布到/move_group/display_planned_path话题
        self.display_trajectory_publisher.publish(display_trajectory)

    def execute_plan(self, plan):

        # 执行规划的笛卡尔路径
        # ^^^^^^^^^^^^^^^^
        self.move_group.execute(plan, wait=True)

        # 注意：机器人的当前关节状态必须在RobotTrajectory中的第一个路径点的公差范围内，否则execute()将失败

def main():
    try:
        print ""
        print "----------------------------------------------------------"
        print "-------欢迎使用FR5机械臂基于ROS和MoveIt控制演示demo-------"
        print "----------------------------------------------------------"
        print "使用Ctrl+D退出该演示demo"
        print ""
        print "============ 按下 `Enter` 键设置并初始化moveit_commander以开始演示demo ..."
        raw_input()
        fr5demo = MoveGroupTest()

        print "============ 按下 `Enter` 键移动到一个关节空间的目标位置 ..."
        raw_input()
        fr5demo.go_to_joint_state()

        print "============ 按下 `Enter` 键移动到一个笛卡尔空间的目标位置 ..."
        raw_input()
        fr5demo.go_to_pose_goal()

        print "============ 按下 `Enter` 键规划并演示一条笛卡尔空间路径 ..."
        raw_input()
        cartesian_plan, fraction =  fr5demo.plan_cartesian_path()

        print "============ 按下 `Enter` 键演示一个已经保存的笛卡尔空间路径  ..."
        raw_input()
        fr5demo.display_trajectory(cartesian_plan)

        print "============ 按下 `Enter` 键执行已保存的笛卡尔空间路径 ..."
        raw_input()
        fr5demo.execute_plan(cartesian_plan)

        print "============ 演示完成!"
    except rospy.ROSInterruptException:
        return
    except KeyboardInterrupt:
        return


if __name__ == '__main__':
    main()