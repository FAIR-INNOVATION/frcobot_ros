# Copyright 2011-2014, Michael Ferguson
# All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions
# are met:
#
#  * Redistributions of source code must retain the above copyright
#    notice, this list of conditions and the following disclaimer.
#  * Redistributions in binary form must reproduce the above
#    copyright notice, this list of conditions and the following
#    disclaimer in the documentation and/or other materials provided
#    with the distribution.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
# "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
# LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
# FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
# COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
# INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
# BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
# LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
# CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
# LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
# ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.

import rospy
import actionlib
from tf.listener import TransformListener
from geometry_msgs.msg import *
from moveit_msgs.msg import MoveGroupAction, MoveGroupGoal
from moveit_msgs.msg import Constraints, JointConstraint, PositionConstraint, OrientationConstraint, BoundingVolume
from shape_msgs.msg import SolidPrimitive
import math
import numpy as np

## @brief Pure python interface to move_group action
class MoveGroupInterface(object):

    ## @brief Constructor for this utility
    ## @param group Name of the MoveIt! group to command
    ## @param frame Name of the fixed frame in which planning happens
    ## @param move_group Name of the action server
    ## @param listener A TF listener instance (optional, will create a new one if None)
    ## @param plan_only Should we only plan, but not execute?
    def __init__(self, group, frame, listener=None, plan_only=False, move_group="move_group"):
        self._group = group
        self._fixed_frame = frame
        self._action = actionlib.SimpleActionClient(move_group,
                                                    MoveGroupAction)
        self._action.wait_for_server()
        if listener == None:
            self._listener = TransformListener()
        else:
            self._listener = listener
        self.plan_only = plan_only
        self.planner_id = None
        self.pipeline_id = None
        self.planning_time = 15.0

    def get_move_action(self):
        return self._action

    ## @brief Move the arm to set of joint position goals
    def moveToJointPosition(self,
                            joints,
                            positions,
                            tolerance=0.01,
                            wait=True,
                            **kwargs):
        # Check arguments
        supported_args = ("max_velocity_scaling_factor",
                          "max_acceleration_scaling_factor",
                          "pipeline_id",
                          "planner_id",
                          "planning_scene_diff",
                          "planning_time",
                          "plan_only",
                          "start_state")
        for arg in kwargs.keys():
            if not arg in supported_args:
                rospy.loginfo("moveToJointPosition: unsupported argument: %s",
                              arg)

        # Create goal
        g = MoveGroupGoal()

        # 1. fill in workspace_parameters

        # 2. fill in start_state
        try:
            g.request.start_state = kwargs["start_state"]
        except KeyError:
            g.request.start_state.is_diff = True

        # 3. fill in goal_constraints
        c1 = Constraints()
        for i in range(len(joints)):
            c1.joint_constraints.append(JointConstraint())
            c1.joint_constraints[i].joint_name = joints[i]
            c1.joint_constraints[i].position = positions[i]
            c1.joint_constraints[i].tolerance_above = tolerance
            c1.joint_constraints[i].tolerance_below = tolerance
            c1.joint_constraints[i].weight = 1.0
        g.request.goal_constraints.append(c1)

        # 4. fill in path constraints

        # 5. fill in trajectory constraints

        # 5.5 fill pipeline id
        try:
            g.request.pipeline_id = kwargs["pipeline_id"]
        except KeyError:
            if self.pipeline_id:
                g.request.pipeline_id = self.pipeline_id

        # 6. fill in planner id
        try:
            g.request.planner_id = kwargs["planner_id"]
        except KeyError:
            if self.planner_id:
                g.request.planner_id = self.planner_id

        # 7. fill in group name
        g.request.group_name = self._group

        # 8. fill in number of planning attempts
        try:
            g.request.num_planning_attempts = kwargs["num_attempts"]
        except KeyError:
            g.request.num_planning_attempts = 10

        # 9. fill in allowed planning time
        try:
            g.request.allowed_planning_time = kwargs["planning_time"]
        except KeyError:
            g.request.allowed_planning_time = self.planning_time

        # Fill in velocity scaling factor
        try:
            g.request.max_velocity_scaling_factor = kwargs["max_velocity_scaling_factor"]
        except KeyError:
            pass  # do not fill in at all

        try:
            g.request.max_acceleration_scaling_factor = kwargs["max_acceleration_scaling_factor"]
        except KeyError:
            pass  # do not fill in at all

        # 10. fill in planning options diff
        try:
            g.planning_options.planning_scene_diff = kwargs["planning_scene_diff"]
        except KeyError:
            g.planning_options.planning_scene_diff.is_diff = True
            g.planning_options.planning_scene_diff.robot_state.is_diff = True

        # 11. fill in planning options plan only
        try:
            g.planning_options.plan_only = kwargs["plan_only"]
        except KeyError:
            g.planning_options.plan_only = self.plan_only

        # 12. fill in other planning options
        g.planning_options.look_around = False
        g.planning_options.replan = False

        # 13. send goal
        self._action.send_goal(g)
        if wait:
            self._action.wait_for_result()
            return self._action.get_result()
        else:
            return None

    def vector_to_quaternion(self, point1, point2):
        vector = np.array(point2) - np.array(point1)
        vector_norm = vector / np.linalg.norm(vector)
        # The z-axis is represented as [0, 0, 1]
        z_axis = np.array([0, 0, 1])
        cross_product = np.cross(vector_norm, z_axis)
        dot_product = np.dot(vector_norm, z_axis)

        w = 1 + dot_product
        x = cross_product[0]
        y = cross_product[1]
        z = cross_product[2]
        
        # Normalize the quaternion
        norm = math.sqrt(w**2 + x**2 + y**2 + z**2)
        quaternion = np.array([w, x, y, z]) / norm
        return quaternion
    
    def convert2right_vector(self, point1, point2):
        # Calculate the vector from point1 to point2
        vector = np.array(point2) - np.array(point1)
        vector_list = []
        for i in range(0,3):
            if i == np.argmax(abs(vector)):
                vector_list.append(vector[i]/abs(vector[i]))
            else:
                vector_list.append(0)
        # Return the vector and the nearest normal vector
        return vector_list
    
    ## @brief Move the arm, based on a goal pose_stamped for the end effector.
    def moveToPose(self,
                   pose_stamped,
                   gripper_frame,
                   tolerance=0.01,
                   wait=True,
                   rotation_mode=None,
                   rotation_data=None,
                   **kwargs):
        # Check arguments
        supported_args = ("max_velocity_scaling_factor",
                          "max_acceleration_scaling_factor",
                          "pipeline_id",
                          "planner_id",
                          "planning_time",
                          "plan_only",
                          "start_state")
        for arg in kwargs.keys():
            if not arg in supported_args:
                rospy.loginfo("moveToPose: unsupported argument: %s",
                              arg)

        # Create goal
        g = MoveGroupGoal()
        pose_transformed = self._listener.transformPose(self._fixed_frame, pose_stamped)

        x = pose_transformed.pose.position.x
        y = pose_transformed.pose.position.y
        z = pose_transformed.pose.position.z

        point1 = (0, 0, 0)
        point2 = (x, y, z)

        # if rotation_mode==None:
        #     pass
        # elif rotation_data==None:
        #     print("Error! 'rotation_data' is None")
        #     sys.exit()
        # elif rotation_mode == "right_angle":
        #     # rotation_data = [[0,0,0],[1,1,1]]
        #     point1 = (rotation_data[0][0], rotation_data[0][1], rotation_data[0][2])
        #     point2 = (x*rotation_data[1][0], y*rotation_data[1][1], z*rotation_data[1][2])

        #     vector = self.convert2right_vector(point1,point2)
        #     quat = self.vector_to_quaternion(point1,vector)
        #     if (math.sqrt(x*x+y*y+z*z) < 0.3) and (z < 0.1):
        #         pose_transformed.pose.orientation.x = 0
        #         pose_transformed.pose.orientation.y = 0.7071078
        #         pose_transformed.pose.orientation.z = 0
        #         pose_transformed.pose.orientation.w = 0.7071078
        #     else:
        #         pose_transformed.pose.orientation.x = quat[1]
        #         pose_transformed.pose.orientation.y = quat[2]
        #         pose_transformed.pose.orientation.z = quat[3]
        #         pose_transformed.pose.orientation.w = quat[0]
        # elif rotation_mode == "vector2point":
        #     # rotation_data = [[0,0,0],[1,0,0]]
        #     quaternion = self.vector_to_quaternion(rotation_data[0],rotation_data[1])
        #     pose_transformed.pose.orientation.x = quaternion[1]
        #     pose_transformed.pose.orientation.y = quaternion[2]
        #     pose_transformed.pose.orientation.z = quaternion[3]
        #     pose_transformed.pose.orientation.w = quaternion[0]
        # else:
        #     print("rotation_mode error")
        #     sys.exit()
            
        # 1. fill in request workspace_parameters

        # 2. fill in request start_state
        try:
            g.request.start_state = kwargs["start_state"]
        except KeyError:
            g.request.start_state.is_diff = True

        # 3. fill in request goal_constraints
        c1 = Constraints()

        c1.position_constraints.append(PositionConstraint())
        c1.position_constraints[0].header.frame_id = self._fixed_frame
        c1.position_constraints[0].link_name = gripper_frame
        b = BoundingVolume()
        s = SolidPrimitive()
        s.dimensions = [tolerance * tolerance]
        s.type = s.SPHERE
        b.primitives.append(s)
        b.primitive_poses.append(pose_transformed.pose)
        c1.position_constraints[0].constraint_region = b
        c1.position_constraints[0].weight = 1.0

        c1.orientation_constraints.append(OrientationConstraint())
        c1.orientation_constraints[0].header.frame_id = self._fixed_frame
        c1.orientation_constraints[0].orientation = pose_transformed.pose.orientation
        c1.orientation_constraints[0].link_name = gripper_frame
        c1.orientation_constraints[0].absolute_x_axis_tolerance = tolerance
        c1.orientation_constraints[0].absolute_y_axis_tolerance = tolerance
        c1.orientation_constraints[0].absolute_z_axis_tolerance = tolerance
        c1.orientation_constraints[0].weight = 1.0

        g.request.goal_constraints.append(c1)

        # 4. fill in request path constraints

        # 5. fill in request trajectory constraints

        # 5.5 fill pipeline id
        try:
            g.request.pipeline_id = kwargs["pipeline_id"]
        except KeyError:
            if self.pipeline_id:
                g.request.pipeline_id = self.pipeline_id

        # 6. fill in request planner id
        try:
            g.request.planner_id = kwargs["planner_id"]
        except KeyError:
            if self.planner_id:
                g.request.planner_id = self.planner_id

        # 7. fill in request group name
        g.request.group_name = self._group

        # 8. fill in request number of planning attempts
        try:
            g.request.num_planning_attempts = kwargs["num_attempts"]
        except KeyError:
            g.request.num_planning_attempts = 10

        # 9. fill in request allowed planning time
        try:
            g.request.allowed_planning_time = kwargs["planning_time"]
        except KeyError:
            g.request.allowed_planning_time = self.planning_time

        # Fill in velocity scaling factor
        try:
            g.request.max_velocity_scaling_factor = kwargs["max_velocity_scaling_factor"]
        except KeyError:
            pass  # do not fill in at all

        try:
            g.request.max_acceleration_scaling_factor = kwargs["max_acceleration_scaling_factor"]
        except KeyError:
            pass  # do not fill in at all

        # 10. fill in planning options diff
        g.planning_options.planning_scene_diff.is_diff = True
        g.planning_options.planning_scene_diff.robot_state.is_diff = True

        # 11. fill in planning options plan only
        try:
            g.planning_options.plan_only = kwargs["plan_only"]
        except KeyError:
            g.planning_options.plan_only = self.plan_only

        # 12. fill in other planning options
        g.planning_options.look_around = False
        g.planning_options.replan = False

        # 13. send goal
        self._action.send_goal(g)
        if wait:
            self._action.wait_for_result()
            return self._action.get_result()
        else:
            return None

    ## @brief Sets the planner_id used for all future planning requests.
    ## @param planner_id The string for the planner id, set to None to clear
    def setPlannerId(self, planner_id):
        self.planner_id = str(planner_id)

    ## @brief Sets the pipeline_id used for all future planning requests.
    ## @param pipeline_id The string for the planner id, set to None to clear
    def setPipelineId(self, pipeline_id):
        self.pipeline_id = str(pipeline_id)

    ## @brief Set default planning time to be used for future planning request.
    def setPlanningTime(self, time):
        self.planning_time = time
