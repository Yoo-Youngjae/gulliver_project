#!/usr/bin/env python

import sys
import os
if 'ROS_NAMESPACE' not in os.environ:
    os.environ['ROS_NAMESPACE'] = 'my_gen3_lite'

import rospy

import moveit_msgs.msg
from kortex_driver.srv import *
from kortex_driver.msg import *
import moveit_commander
import moveit_msgs.msg

from math import pi

class RotateJoint(object):
    def __init__(self):
        super(RotateJoint, self).__init__()
        moveit_commander.roscpp_initialize(sys.argv)
        rospy.init_node('kortex_driver_move_it_good')
        try:
            self.degrees_of_freedom = rospy.get_param(rospy.get_namespace() + "degrees_of_freedom", 6)

            arm_group_name = "arm"
            self.robot = moveit_commander.RobotCommander("robot_description")
            self.scene = moveit_commander.PlanningSceneInterface(ns=rospy.get_namespace())
            self.arm_group = moveit_commander.MoveGroupCommander(arm_group_name, ns=rospy.get_namespace())
            self.display_trajectory_publisher = rospy.Publisher(
                rospy.get_namespace() + 'move_group/display_planned_path',
                moveit_msgs.msg.DisplayTrajectory,
                queue_size=20)
            # play_joint_trajectory_full_name = '/' + self.robot_name + '/base/play_joint_trajectory'
            # rospy.wait_for_service(play_joint_trajectory_full_name)
            # self.play_joint_trajectory = rospy.ServiceProxy(play_joint_trajectory_full_name, PlayJointTrajectory)

        except Exception as e:
            print (e)
            self.is_init_success = False
        else:
            self.is_init_success = True
    
    def change_joint_angle(self, num, angle):
        joint_positions = self.arm_group.get_current_joint_values()
        if num == 1:
            current = joint_positions[0]
            angle = float(angle) / 180 * pi
            joint_positions[0] += angle
            if joint_positions[0] < -154.1 / 180 * pi or joint_positions[0] > 154.1 / 180 * pi:
                print("Give an angle between (-154.1, +154.1)")
                joint_positions[0] = current
                print("current joint_1 angle: {0:0.1f}".format(current * 180 / pi))
            else:
                self.arm_group.set_joint_value_target(joint_positions)
                self.arm_group.go(wait=True)
                print("current joint_1 angle: {0:0.1f}".format(joint_positions[0] * 180 / pi))
        elif num == 2:
            current = joint_positions[1]
            angle = float(angle) / 180 * pi
            joint_positions[1] += angle
            if joint_positions[1] < -150.1 / 180 * pi or joint_positions[1] > 150.1 / 180 * pi:
                print("Give an angle between (-150.1, +150.1")
                joint_positions[1] = current
                print("current joint_2 angle: {0:0.1f}".format(current * 180 / pi))
            else:
                self.arm_group.set_joint_value_target(joint_positions)
                self.arm_group.go(wait=True)
                print("current joint_2 angle: {0:0.1f}".format(joint_positions[1] * 180 / pi))
        elif num == 3:
            current = joint_positions[2]
            angle = float(angle) / 180 * pi
            joint_positions[2] += angle
            if joint_positions[2] < -150.1 / 180 * pi or joint_positions[2] > 150.1 / 180 * pi:
                print("Give an angle between (-150.1, +150.1")
                joint_positions[2] = current
                print("current joint_3 angle: {0:0.1f}".format(current * 180 / pi))
            else:
                self.arm_group.set_joint_value_target(joint_positions)
                self.arm_group.go(wait=True)
                print("current joint_3 angle: {0:0.1f}".format(joint_positions[2] * 180 / pi))
        elif num == 4:
            current = joint_positions[3]
            angle = float(angle) / 180 * pi
            joint_positions[3] += angle
            if joint_positions[3] < -148.98 / 180 or joint_positions[3] > 148.98 / 180:
                print("Give an angle between (-148.98, +148.98)")
                joint_positions[3] = current
                print("current joint_4 angle: {0:0.1f}".format(current * 180))
            else:
                self.arm_group.set_joint_value_target(joint_positions)
                self.arm_group.go(wait=True)
                print("current joint_4 angle: {0:0.1f}".format(joint_positions[3] * 180))
        elif num == 5:
            current = joint_positions[4]
            angle = float(angle) / 180
            joint_positions[4] += angle
            if joint_positions[4] < -144.97 / 180 or joint_positions[4] > 145.0 / 180:
                print("Give an angle between (-144.97, +145.0)")
                joint_positions[4] = current
                print("current joint_5 angle: {0:0.1f}".format(current * 180))
            else:
                self.arm_group.set_joint_value_target(joint_positions)
                self.arm_group.go(wait=True)
                print("current joint_5 angle: {0:0.1f}".format(joint_positions[4] * 180))
        elif num == 6:
            current = joint_positions[5]
            angle = float(angle) / 180
            joint_positions[5] += angle
            if joint_positions[5] < -148.98 / 180 or joint_positions[5] > 148.98 / 180:
                print("Give an angle between (-148.98, +148.98)")
                joint_positions[5] = current
                print("current joint_6 angle: {0:0.1f}".format(current * 180))
            else:
                self.arm_group.set_joint_value_target(joint_positions)
                self.arm_group.go(wait=True)
                print("current joint_6 angle: {0:0.1f}".format(joint_positions[5] * 180))
        else:
            print("Give a number between 1 to 6.")


if __name__ == '__main__':
    example = RotateJoint()
    joints = example.arm_group.get_current_joint_values()
    joints[0] = 0
    joints[1] = 0
    joints[2] = 0
    joints[3] = 0
    joints[4] = 0
    joints[5] = 0
    example.arm_group.set_joint_value_target(joints)
    example.arm_group.go(wait=True)
    while input("Press q to quit : ") != 'q':
        joint_num = input("Give the number of joint to rotate : ")
        joint_angle = input("Give the rotation angle in degrees : ")
        example.change_joint_angle(joint_num, joint_angle)