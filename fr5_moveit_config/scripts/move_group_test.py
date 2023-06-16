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
    It is used to test whether the value of actual is within the tolerance range of the corresponding value of goal.
    @param: goal       target parameter. List of floats, Pose type or PoseStamped type message
    @param: actual     Test parameters. List of floats, Pose type or PoseStamped type message
    @param: tolerance  tolerance range. floating point number
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

        # Initialize moveit_commander API and rospy node
        moveit_commander.roscpp_initialize(sys.argv)
        rospy.init_node('move_group_test', anonymous=True)

        # Initialize the RobotCommander object, provide information such as the robot kinematics model and the current joint state of the robot, and the interface between the robot and the outside world.
        robot = moveit_commander.RobotCommander()

        # Initialize the PlanningSceneInterface object to provide an interface between the robot and the surrounding world.
        scene = moveit_commander.PlanningSceneInterface()

        # Initialize the MoveGroupCommander object. This object is the interface of the fr5 manipulator planning group, which is used to plan and execute the manipulator motion.
        group_name = "fr5_arm"
        move_group = moveit_commander.MoveGroupCommander(group_name)

        # Create a ROS publisher for displaying trajectory information in RViz
        display_trajectory_publisher = rospy.Publisher('/move_group/display_planned_path',
                                                       moveit_msgs.msg.DisplayTrajectory,
                                                       queue_size=20)                                                   

        # Get the reference frame name of the robot
        planning_frame = move_group.get_planning_frame()
        print "============ Planning frame: %s" % planning_frame

        # Get the name of the robot's end effector linkage in the current planning group
        eef_link = move_group.get_end_effector_link()
        print "============ End effector link: %s" % eef_link

        # Get the names of all planning groups of the robot and output them in an array
        group_names = robot.get_group_names()
        print "============ Available Planning Groups:", robot.get_group_names()

        # Get the current state of the robot
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

        # Plan and execute to a joint target position
        # ^^^^^^^^^^^^^^^^^^^^^^^^
        # The initial pose of Fao robot is a singular pose, first we move it to a better position.
        # First obtain the joint values of the current robot from the interface, and then modify the values of some joints
        joint_goal = self.move_group.get_current_joint_values()
        print "============ Printing current joint values: ", joint_goal
        joint_goal[0] = 0
        joint_goal[1] = -pi/4
        joint_goal[2] = -pi/2
        joint_goal[3] = -pi/2
        joint_goal[4] = pi/3
        joint_goal[5] = 0
        print "============ Printing joint goal: ", joint_goal

        # Move to joint target position
        self.move_group.go(joint_goal, wait=True)
        rospy.sleep(1)
        # Invoke a stop instruction to make sure there is no residual motion
        self.move_group.stop()

        # For testing:
        current_joints = self.move_group.get_current_joint_values()
        return all_close(joint_goal, current_joints, 0.01)

    def go_to_pose_goal(self):

        # Plan and execute to end goal pose
        # ^^^^^^^^^^^^^^^^^^^^^^^
        pose_goal = geometry_msgs.msg.Pose()
        pose_goal.orientation.w = 1.0
        pose_goal.position.x = 0.4
        pose_goal.position.y = 0.1
        pose_goal.position.z = 0.4

        # Set target pose
        self.move_group.set_pose_target(pose_goal)

        # Plan and execute to target pose
        self.move_group.go(wait=True)
        rospy.sleep(1)
        self.move_group.stop()

        # clear target pose
        self.move_group.clear_pose_targets()

        # For testing:
        current_pose = self.move_group.get_current_pose().pose
        return all_close(pose_goal, current_pose, 0.01)

    def plan_cartesian_path(self, scale=1.0):

        # Plan and execute a Cartesian path
        # ^^^^^^^^^^^^^^^^^^^^
        # You can plan a Cartesian path directly by specifying a list of waypoints for the end effector to pass through.
        waypoints = []

        wpose = self.move_group.get_current_pose().pose
        wpose.position.z -= scale * 0.1  # Move down the z axis
        wpose.position.y += scale * 0.2  # Move sideways on the y axis
        waypoints.append(copy.deepcopy(wpose))

        wpose.position.x += scale * 0.1  # Move forward along the x-axis
        waypoints.append(copy.deepcopy(wpose))

        wpose.position.y -= scale * 0.1  # Move sideways on the y axis
        waypoints.append(copy.deepcopy(wpose))

        # We want to interpolate the Cartesian path with a resolution of 1cm, that's why we specified 0.01 as eef_step in the Cartesian transformation.
        # By setting it to 0.0, you can disable the jump threshold and ignore the check for jumps that are not feasible in joint space.
        (plan, fraction) = self.move_group.compute_cartesian_path(
            waypoints,   # waypoints to follow
            0.01,        # eef_step
            0.0)         # jump_threshold

        # At present, it is only planning the path, and does not let move_group execute and move the robot
        return plan, fraction

    def display_trajectory(self, plan):

        # Show motion track
        # ^^^^^^^^^^
        # DisplayTrajectory msg has two main fields，trajectory_start和trajectory。
        # We populate trajectory_start with the current robot state, to copy any AttachedCollisionObjects and add our plan to that trajectory.
        display_trajectory = moveit_msgs.msg.DisplayTrajectory()
        display_trajectory.trajectory_start = self.robot.get_current_state()
        display_trajectory.trajectory.append(plan)
        # Published to /move_group/display_planned_path topic
        self.display_trajectory_publisher.publish(display_trajectory)

    def execute_plan(self, plan):

        # Cartesian path of execution plan
        # ^^^^^^^^^^^^^^^^
        self.move_group.execute(plan, wait=True)

        # Note: The robot's current joint state must be within the tolerance of the first pathpoint in the RobotTrajectory, otherwise execute() will fail

def main():
    try:
        print ""
        print "----------------------------------------------------------"
        print "-------Welcome to the FR5 robotic arm control demo based on ROS and MoveIt-------"
        print "----------------------------------------------------------"
        print "Exit the demo with Ctrl+D"
        print ""
        print "============ Press `Enter` to set up and initialize moveit_commander to start the demo ..."
        raw_input()
        fr5demo = MoveGroupTest()

        print "============ Press the `Enter` key to move to a target position in joint space ..."
        raw_input()
        fr5demo.go_to_joint_state()

        print "============ Press the `Enter` key to move to a Cartesian space target position ..."
        raw_input()
        fr5demo.go_to_pose_goal()

        print "============ Press `Enter` to plan and demonstrate a path in Cartesian space ..."
        raw_input()
        cartesian_plan, fraction =  fr5demo.plan_cartesian_path()

        print "============ Press the `Enter` key to demonstrate a saved path in Cartesian space  ..."
        raw_input()
        fr5demo.display_trajectory(cartesian_plan)

        print "============ Press the `Enter` key to execute the saved Cartesian space path ..."
        raw_input()
        fr5demo.execute_plan(cartesian_plan)

        print "============ Demo complete!"
    except rospy.ROSInterruptException:
        return
    except KeyboardInterrupt:
        return


if __name__ == '__main__':
    main()