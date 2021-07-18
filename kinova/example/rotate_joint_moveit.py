import sys
import rospy
import moveit_commander
import moveit_msgs.msg
from kortex_driver.srv import *
from kortex_driver.msg import *


class ExampleFullArmMovement(object):
    def __init__(self):
        rospy.init_node('kortex_driver')

        try:
            arm_group_name = "arm"
            self.robot = moveit_commander.RobotCommander("robot_description")
            self.arm_group = moveit_commander.MoveGroupCommander(arm_group_name, ns=rospy.get_namespace())
            self.display_trajectory_publisher = rospy.Publisher(
                rospy.get_namespace() + 'my_gen3_lite/joint_state',
                moveit_msgs.msg.DisplayTrajectory,
                queue_size=20)

        except Exception as e:
            print (e)
            self.is_init_success = False
        else:
            self.is_init_success = True

    def change_joint_angle(self, num, angle):
        joint_positions = self.arm_group.get_current_joint_values()
        if num == 1:
            current = joint_positions[0]
            angle = angle / 180
            joint_positions[0] += angle
            if joint_positions[0] < -154.1 / 180 or joint_positions[0] > 154.1 / 180:
                print("Give an angle between (-154.1, +154.1")
                joint_positions[0] = current
            else:
                self.arm_group.set_joint_value_target(joint_positions)
                self.arm_group.go(wait=True)
        elif num == 2:
            current = joint_positions[1]
            angle = angle / 180
            joint_positions[1] += angle
            if joint_positions[1] < -150.1 / 180 or joint_positions[1] > 150.1 / 180:
                print("Give an angle between (-150.1, +150.1")
                joint_positions[1] = current
            else:
                self.arm_group.set_joint_value_target(joint_positions)
                self.arm_group.go(wait=True)
        elif num == 3:
            current = joint_positions[2]
            angle = angle / 180
            joint_positions[2] += angle
            if joint_positions[2] < -150.1 / 180 or joint_positions[2] > 150.1 / 180:
                print("Give an angle between (-150.1, +150.1")
                joint_positions[2] = current
            else:
                self.arm_group.set_joint_value_target(joint_positions)
                self.arm_group.go(wait=True)
        elif num == 4:
            current = joint_positions[3]
            angle = angle / 180
            joint_positions[3] += angle
            if joint_positions[3] < -148.98 / 180 or joint_positions[3] > 148.98 / 180:
                print("Give an angle between (-148.98, +148.98")
                joint_positions[3] = current
            else:
                self.arm_group.set_joint_value_target(joint_positions)
                self.arm_group.go(wait=True)
        elif num == 5:
            current = joint_positions[4]
            angle = angle / 180
            joint_positions[4] += angle
            if joint_positions[4] < -144.97 / 180 or joint_positions[4] > 145.0 / 180:
                print("Give an angle between (-144.97, +145.0")
                joint_positions[4] = current
            else:
                self.arm_group.set_joint_value_target(joint_positions)
                self.arm_group.go(wait=True)
        elif num == 6:
            current = joint_positions[5]
            angle = angle / 180
            joint_positions[5] += angle
            if joint_positions[5] < -148.98 / 180 or joint_positions[5] > 148.98 / 180:
                print("Give an angle between (-148.98, +148.98")
                joint_positions[5] = current
            else:
                self.arm_group.set_joint_value_target(joint_positions)
                self.arm_group.go(wait=True)
        else:
            print("Give a number between 1 to 6.")


if __name__ == '__main__':
    example = ExampleFullArmMovement()
    while raw_input("Press q to quit : ") != 'q':
        joint_num = input("Give the number of joint to rotate : ")
        joint_angle = input("Give the rotation angle in degrees : ")
        example.change_joint_angle(joint_num, joint_angle)

