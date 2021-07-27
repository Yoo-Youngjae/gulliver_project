import rospy
import os
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg
import sys
from geometry_msgs.msg import Point
from math import pi, radians
from std_srvs.srv import Empty
from moveit_msgs.msg import Constraints
from moveit_msgs.msg import JointConstraint
from tf.transformations import quaternion_from_euler, euler_from_quaternion
from kortex_driver.msg import *

if 'ROS_NAMESPACE' not in os.environ:
    os.environ['ROS_NAMESPACE'] = 'my_gen3_lite'

class MoveRobot(object):
    def __init__(self):

        super(MoveRobot, self).__init__()
        moveit_commander.roscpp_initialize(sys.argv)
        rospy.init_node('vr_controller_point_arm_move_end_effector')
        # self.hand_point_sub = rospy.Subscriber('rightHandPoint', Point, self.hand_point_callback)
        self.x = None
        self.y = None
        self.z = None

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

            self.arm_group.set_max_velocity_scaling_factor(1)
            self.arm_group.set_max_acceleration_scaling_factor(0.5)

            if self.is_gripper_present:
                gripper_group_name = "gripper"
                self.gripper_group = moveit_commander.MoveGroupCommander(gripper_group_name, ns=rospy.get_namespace())

        except Exception as e:
            print (e)
            self.is_init_success = False
        else:
            self.is_init_success = True

    def hand_point_callback(self, data):
        print(data) # todo : for test
        # self.hand_point_data = data
        self.x = data.x
        self.y = data.y
        self.z = data.z
        move_robot.arm_group.set_position_target([self.x, self.y, self.z])
        move_robot.arm_group.go(True, wait=True)


if __name__ == '__main__':
    move_robot = MoveRobot()
    while True:
        hand_point_sub = rospy.Subscriber('rightHandPoint', Point, move_robot.hand_point_callback)

