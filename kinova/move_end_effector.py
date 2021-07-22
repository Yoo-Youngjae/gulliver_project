import sys
import time
import os

if 'ROS_NAMESPACE' not in os.environ:
    os.environ['ROS_NAMESPACE'] = 'my_gen3_lite'

import rospy
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg
from math import pi, radians
from std_srvs.srv import Empty
from moveit_msgs.msg import Constraints
from moveit_msgs.msg import JointConstraint
from tf.transformations import quaternion_from_euler, euler_from_quaternion
from kortex_driver.msg import *
import kortex_driver.srv
import math



class MoveEndEffector(object):

    def __init__(self):

        # Initialize the node
        super(MoveEndEffector, self).__init__()
        moveit_commander.roscpp_initialize(sys.argv)
        rospy.init_node('move_end_effector')

        try:
            self.is_gripper_present = rospy.get_param(rospy.get_namespace() + "is_gripper_present", False)
            if self.is_gripper_present:
                gripper_joint_names = rospy.get_param(rospy.get_namespace() + "gripper_joint_names", [])
                self.gripper_joint_name = gripper_joint_names[0]
            else:
                self.gripper_joint_name = ""
            self.degrees_of_freedom = rospy.get_param(rospy.get_namespace() + "degrees_of_freedom", 6)

            # Create the MoveItInterface necessary objects
            arm_group_name = "arm"
            self.robot = moveit_commander.RobotCommander("robot_description")
            self.scene = moveit_commander.PlanningSceneInterface(ns=rospy.get_namespace())
            self.arm_group = moveit_commander.MoveGroupCommander(arm_group_name, ns=rospy.get_namespace())
            self.display_trajectory_publisher = rospy.Publisher(
                rospy.get_namespace() + 'move_group/display_planned_path',
                moveit_msgs.msg.DisplayTrajectory,
                queue_size=20)
            # self.velocity_publisher = rospy.Publisher(rospy.get_namespace() + 'in/joint_velocity',
            #                                            Base_JointSpeeds, latch=True,
            #                                            queue_size=20)
            # self.base_msg = Base_JointSpeeds()
            # joint_speeds = []
            # for joint_idx in range(6):
            #     msg = JointSpeed()
            #     msg.joint_identifier = joint_idx
            #     msg.value = 55
            #     msg.duration = 0
            #     joint_speeds.append(msg)
            # self.base_msg.joint_speeds = joint_speeds
            # self.base_msg.duration = 0
            # self.velocity_publisher.publish(self.base_msg)

            # self.velocity_publisher.publish(base_msg)
            self.arm_group.set_max_velocity_scaling_factor(1)
            self.arm_group.set_max_acceleration_scaling_factor(1)

            if self.is_gripper_present:
                gripper_group_name = "gripper"
                self.gripper_group = moveit_commander.MoveGroupCommander(gripper_group_name, ns=rospy.get_namespace())


        except Exception as e:
            print (e)
            self.is_init_success = False
        else:
            self.is_init_success = True


    def set_position(self, key):
        pose = self.arm_group.get_current_pose().pose
        joint_positions = self.arm_group.get_current_joint_values()

        eef_constraints = moveit_msgs.msg.Constraints()
        orientation_constraint = moveit_msgs.msg.OrientationConstraint()
        orientation_constraint.orientation = pose.orientation
        eef_constraints.orientation_constraints.append(orientation_constraint)
        changed = 0
        try:
            if key == 'y-':
                pose.position.y -= 0.1
                base_position = joint_positions[0]
                self.set_base_constraint(0, pi / 2, base_position)
                changed = 1
            elif key == 'y+':
                pose.position.y += 0.1
                base_position = joint_positions[0]
                self.set_base_constraint(pi / 2, 0, base_position)
                changed = 1
            elif key == 'x+':
                pose.position.x += 0.1
                base_position = joint_positions[0]
                self.set_base_constraint(pi / 2, 0, base_position)
                changed = 1
            elif key == 'x-':
                pose.position.x -= 0.1
                base_position = joint_positions[0]
                self.set_base_constraint(0, pi / 2, base_position)
                changed = 1
            elif key == 'z+':
                pose.position.z += 0.1
                base_position = joint_positions[0]
                changed = 1
            elif key == 'z-':
                pose.position.z -= 0.1
                base_position = joint_positions[0]
                changed = 1
            else:
                print("Press the key again.")
        except Exception as e:
            print(e)
        finally:
            if changed == 0:
                if key == 'x+':
                    pose.position.x += 0.1
                elif key == 'x-':
                    pose.position.x -= 0.1
                elif key == 'y-':
                    pose.position.y -= 0.1
                elif key == 'y+':
                    pose.position.y += 0.1
                elif key == 'z+':
                    pose.position.z += 0.1
                elif key == 'z-':
                    pose.position.z -= 0.1
                else:
                    print("Press the key again.")
        self.arm_group.set_pose_target(pose)
        self.arm_group.go(True, wait=True)
        self.arm_group.stop()

    def set_base_constraint(self, above, below, position):
        self.arm_group.clear_path_constraints()
        constraint = Constraints()
        joint_constraint = JointConstraint()
        joint_constraint.position = position
        joint_constraint.tolerance_above = above
        joint_constraint.tolerance_below = below
        joint_constraint.joint_name = 'joint_1'
        joint_constraint.weight = 1
        constraint.joint_constraints.append(joint_constraint)
        self.arm_group.set_path_constraints(constraint)

    def set_angle(self, key, angle):
        pose = self.arm_group.get_current_pose().pose
        angle = radians(angle)
        current_qt = [pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w]
        current_euler = list(euler_from_quaternion(current_qt))
        try:
            if key == 'r':
                current_euler[0] += angle
                q = quaternion_from_euler(current_euler[0], current_euler[1], current_euler[2])
            elif key == 'y':
                current_euler[1] += angle
                q = quaternion_from_euler(current_euler[0], current_euler[1], current_euler[2])
            elif key == 'p':
                current_euler[2] += angle
                q = quaternion_from_euler(current_euler[0], current_euler[1], current_euler[2])
            else:
                print("Press the key again.")
            pose.orientation.x = q[0]
            pose.orientation.y = q[1]
            pose.orientation.z = q[2]
            pose.orientation.w = q[3]
            self.arm_group.set_pose_target(pose)
            self.arm_group.go(True, wait=True)
            self.arm_group.stop()
        except Exception as e:
            print(e)

    def send_home(self):
        joint_positions = self.arm_group.get_current_joint_values()
        joint_positions[0] = 0
        joint_positions[1] = 0
        joint_positions[2] = 0
        joint_positions[3] = pi / 2
        joint_positions[4] = radians(-144)
        joint_positions[5] = pi / 2
        self.arm_group.set_joint_value_target(joint_positions)
        self.arm_group.go(wait=True)


if __name__ == '__main__':
    move = MoveEndEffector()
    # move.send_home()
    while raw_input("Press q to quit. Press other key to continue : ") != 'q':
        input_key = raw_input("Press the key : ")
        if (input_key != 'r' and input_key != 'y' and input_key != 'p'):
            move.set_position(input_key)
        else:
            move.set_angle(input_key, 10)