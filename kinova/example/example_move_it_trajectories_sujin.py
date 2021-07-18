#!/usr/bin/env python

# Software License Agreement (BSD License)
#
# Copyright (c) 2013, SRI International
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
#  * Neither the name of SRI International nor the names of its
#    contributors may be used to endorse or promote products derived
#    from this software without specific prior written permission.
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
#
# Author: Acorn Pooley, Mike Lautman

# Inspired from http://docs.ros.org/kinetic/api/moveit_tutorials/html/doc/move_group_python_interface/move_group_python_interface_tutorial.html
# Modified by Alexandre Vannobel to test the FollowJointTrajectory Action Server for the Kinova Gen3 robot

# To run this node in a given namespace with rosrun (for example 'my_gen3'), start a Kortex driver and then run :
# rosrun kortex_examples example_moveit_trajectories.py __ns:=my_gen3
import os
import sys
import time
import rospy
import moveit_commander
import moveit_msgs.msg
from sensor_msgs.msg import JointState
from moveit_msgs.msg import Constraints
from moveit_msgs.msg import JointConstraint
import geometry_msgs.msg
from math import pi
import math
from std_srvs.srv import Empty
from std_msgs.msg import String
import copy
import numpy as np
from sensor_msgs.msg import Image
from sensor_msgs.msg import PointCloud2
import sys

sys.path.append('/home/cylee/PycharmProjects/pythonProject')
from cv_bridge import CvBridge, CvBridgeError
from yolo import detect, draw_bounding_box, net, meta
import cv2
import ros_numpy


class Subscriber():
    def __init__(self):
        self.sub_img = rospy.Subscriber("/camera/color/image_raw", Image, self.image_callback)
        self.sub_pc = rospy.Subscriber("/camera/depth_registered/points", PointCloud2, self.pc_callback)
        self.center_list = []
        self.center_coordinate = []
        self.depth = None

    def image_callback(self, img_msg):
        bridge = CvBridge()
        cv_image = bridge.imgmsg_to_cv2(img_msg, "passthrough")
        cv_image = cv2.cvtColor(cv_image, cv2.COLOR_BGR2RGB)
        r = detect(net, meta, cv_image)
        # image = cv2.imread("/home/cylee/ycb/image/N1_0.jpg")
        # r = detect(net, meta, image)
        for item in r:
            name, prob, box_info = item
            if prob >= 0.05:
                img = draw_bounding_box(cv_image, item)
                # img = draw_bounding_box(image, item)
                self.center_list.append((int(box_info[0]), int(box_info[1])))
        cv2.imshow('img', cv_image)
        cv2.waitKey(3)

    def pc_callback(self, point_msg):
        pc = ros_numpy.numpify(point_msg)
        for center in self.center_list:
            # x,y coordinate in rgb(2d) image
            x = center[0]
            y = center[1]
            # real world x, y, z (meter) 3D coordinate pc[y][x]
            if (not math.isnan(pc[y][x][0]) and not math.isnan(pc[y][x][1]) and not math.isnan(pc[y][x][2])):
                self.center_coordinate.append((float(pc[y][x][0]), float(pc[y][x][1]), pc[y][x][2]))


class ExampleMoveItTrajectories(object):
    """ExampleMoveItTrajectories"""

    def __init__(self):
        # Initialize the node
        super(ExampleMoveItTrajectories, self).__init__()
        moveit_commander.roscpp_initialize(sys.argv)
        rospy.init_node('example_move_it_trajectories')

        try:
            self.is_gripper_present = rospy.get_param(rospy.get_namespace() + "is_gripper_present", False)
            if self.is_gripper_present:
                gripper_joint_names = rospy.get_param(rospy.get_namespace() + "gripper_joint_names", [])
                self.gripper_joint_name = gripper_joint_names[0]
            else:
                gripper_joint_name = ""
            self.degrees_of_freedom = rospy.get_param(rospy.get_namespace() + "degrees_of_freedom", 7)

            # Create the MoveItInterface necessary objects
            arm_group_name = "arm"
            self.robot = moveit_commander.RobotCommander("robot_description")
            self.joint_names = self.robot.get_joint_names()
            self.scene = moveit_commander.PlanningSceneInterface(ns=rospy.get_namespace())
            self.arm_group = moveit_commander.MoveGroupCommander(arm_group_name, ns=rospy.get_namespace())
            self.display_trajectory_publisher = rospy.Publisher(
                rospy.get_namespace() + 'move_group/display_planned_path',
                moveit_msgs.msg.DisplayTrajectory,
                queue_size=20)
            self.arm_group.set_max_velocity_scaling_factor(0.9)

            #
            # self.gripper_constraint = JointConstraint()
            # self.gripper_constraint.position = pi / 6 - pi / 3
            # self.gripper_constraint.tolerance_below = pi / 18
            # self.gripper_constraint.tolerance_above = pi / 18
            # self.gripper_constraint.joint_name = 'joint_6'
            # self.gripper_constraint.weight = 100
            # self.constraint.joint_constraints.append(self.gripper_constraint)

            if self.is_gripper_present:
                gripper_group_name = "gripper"
                self.gripper_group = moveit_commander.MoveGroupCommander(gripper_group_name, ns=rospy.get_namespace())

            rospy.loginfo("Initializing node in namespace " + rospy.get_namespace())
        except Exception as e:
            print (e)
            self.is_init_success = False
        else:
            self.is_init_success = True

    def reach_named_position(self, target):
        arm_group = self.arm_group

        # Going to one of those targets
        rospy.loginfo("Going to named target " + target)
        # Set the target
        arm_group.set_named_target(target)
        # Plan the trajectory
        planned_path1 = arm_group.plan()
        # Execute the trajectory and block while it's not finished
        return arm_group.execute(planned_path1, wait=True)

    def reach_gripper_position(self, relative_position):
        gripper_group = self.gripper_group
        # We only have to move this joint because all others are mimic!
        gripper_joint = self.robot.get_joint(self.gripper_joint_name)
        gripper_max_absolute_pos = gripper_joint.max_bound()
        gripper_min_absolute_pos = gripper_joint.min_bound()
        print(gripper_max_absolute_pos)
        print(gripper_min_absolute_pos)
        try:
            val = gripper_joint.move(
                relative_position * (gripper_max_absolute_pos - gripper_min_absolute_pos) + gripper_min_absolute_pos,
                True)
            return val
        except:
            return False

    def reach_target_position(self, position):
        # target_position = input("Give target position as list [x, y, z] : ")
        rospy.loginfo("Reaching to the target position...")
        position[2] = float(position[2])

        self.joint_positions = JointState()
        self.joint_positions.name = ["joint_1"]
        self.joint_positions.position = [0]
        # self.joint_positions[1] = 0
        # self.joint_positions[2] = pi / 2
        # self.joint_positions[3] = pi / 4
        # self.joint_positions[4] = pi / 4
        # self.joint_positions[5] = pi / 2
        # self.arm_group.set_joint_value_target(self.joint_positions)
        # self.arm_group.go(True, wait=True)
        # self.arm_group.stop()

        self.arm_group.clear_path_constraints()
        # self.constraint = Constraints()
        # self.joint_constraint = JointConstraint()
        # self.joint_constraint.position = - 3 * pi / 4
        # self.joint_constraint.tolerance_above = 0
        # self.joint_constraint.tolerance_below = 0
        # self.joint_constraint.joint_name = 'joint_1'
        # self.joint_constraint.weight = 1
        # self.constraint.joint_constraints.append(self.joint_constraint)
        # self.arm_group.set_path_constraints(self.constraint)

        self.arm_group.set_position_target(position)
        self.arm_group.go(True, wait=True)
        self.arm_group.stop()

        # rotate = input("Give the angle to rotate gripper. Give -1 to stay. : ")
        # while rotate!=-1 :
        # joint_positions = self.arm_group.get_current_joint_values()
        # joint_positions[4] = pi / 2
        # self.arm_group.set_joint_value_target(joint_positions)
        # rospy.loginfo("Setting joint position to grasp object...")
        # self.arm_group.go(wait=True)
        #
        # joint_positions = self.arm_group.get_current_joint_values()
        # joint_positions[5] = pi / 2
        # self.arm_group.set_joint_value_target(joint_positions)
        # rospy.loginfo("Setting gripper position to grasp object...")
        # self.arm_group.go(wait=True)
        #   rotate = input("Give the angle to rotate gripper. Give -1 to stay. : ")
        #
        # rotate = input("Give the angle to rotate joint_5. Give -1 to stay. : ")
        # while rotate != -1:
        #
        #   rotate = input("Give the angle to rotate joint_5. Give -1 to stay. : ")
        #
        # rospy.loginfo("relocating...")
        # self.move(self.arm_group, position[0], position[1], 0.05

    def reach_pose_target(self, x, y, z):
        goal_pose = self.arm_group.get_current_pose().pose
        goal_pose.position.x = float(x)
        goal_pose.position.y = float(y)
        goal_pose.position.z = 0.1
        goal_pose.orientation.x = float(x)
        goal_pose.orientation.y = float(y)
        goal_pose.orientation.z = 0.1
        goal_pose.orientation.w = math.cos(pi / 18)
        self.arm_group.set_pose_target(goal_pose)
        self.arm_group.go(True, wait=True)
        self.arm_group.stop()

    def move(self, armgroup, value_x, value_y, value_z):
        waypoints = []
        wpose = armgroup.get_current_pose().pose
        wpose.position.x = float(value_x)
        wpose.position.y = float(value_y)
        wpose.position.z = 0.05
        waypoints.append(copy.deepcopy(wpose))
        (plan, fraction) = armgroup.compute_cartesian_path(
            waypoints, 0.01, 0.0
        )
        armgroup.execute(plan, wait=True)


def main(sub):
    example = ExampleMoveItTrajectories()
    group = example.arm_group

    # For testing purposes
    success = example.is_init_success
    try:
        rospy.delete_param("/kortex_examples_test_results/moveit_general_python")
    except:
        pass

    if success:
        while len(sub.center_coordinate) == 0:
            pass
        print(sub.center_coordinate[0])
        # height_i = 0.6
        # rospy.loginfo("Modulating the height...")
        # example.move(group, group.get_current_pose().pose.position.x, group.get_current_pose().pose.position.y, height_i)

        rospy.loginfo("Reaching Named Target Vertical...")
        example.reach_named_position("vertical")
        # print(group.get_current_joint_values())
        # rospy.loginfo("Setting joint values...")
        # joint_positions = example.arm_group.get_current_joint_values()
        # joint_positions[0] = 0
        # joint_positions[1] = 0
        # joint_positions[2] = pi / 2
        # joint_positions[3] = pi / 2
        # joint_positions[4] = 0
        # joint_positions[5] = pi / 2
        # example.arm_group.set_joint_value_target(joint_positions)
        # example.arm_group.go(wait=True)
        # # print(group.get_current_pose().pose.orientation.w)
        #
        # group.clear_path_constraints()
        # example.constraint = Constraints()
        # example.joint_constraint = JointConstraint()
        # example.joint_constraint.position = 0
        # example.joint_constraint.tolerance_above = 0
        # example.joint_constraint.tolerance_below = pi / 2
        # example.joint_constraint.joint_name = 'joint_1'
        # example.joint_constraint.weight = 1
        # example.constraint.joint_constraints.append(example.joint_constraint)
        # group.set_path_constraints(example.constraint)

        # example.reach_target_position([sub.center_coordinate[0][0], sub.center_coordinate[0][1], sub.center_coordinate[0][2]])
        # example.move(group, sub.center_coordinate[0][0], sub.center_coordinate[0][1], sub.center_coordinate[0][2])
        # rospy.loginfo("Reaching target pose...")
        # example.reach_pose_target(sub.center_coordinate[0][0], sub.center_coordinate[0][1], sub.center_coordinate[0][2])

        # example.reach_gripper_position(1)
        # height_o = 0.1
        # rospy.loginfo("Preparing...")
        # example.move(group, group.get_current_pose().pose.position.x, group.get_current_pose().pose.position.y, 0.5*height_o)

        # size = 0.8
        # example.reach_gripper_position(size)

        # rospy.loginfo("locating...")
        # example.reach_target_position([-0.4, 0.4, 0.2])
        # example.reach_gripper_position(1)

        rospy.set_param("/kortex_examples_test_results/moveit_general_python", success)

    if not success:
        rospy.logerr("The example encountered an error.")


if __name__ == '__main__':
    sub = Subscriber()
    main(sub)
