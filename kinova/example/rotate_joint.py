import sys
import rospy
import math
from sensor_msgs.msg import JointState
from kortex_driver.srv import *
from kortex_driver.msg import *


class ExampleFullArmMovement:
    def __init__(self):
        rospy.init_node('kortex_driver_rotate_custom')

        try:
            self.HOME_ACTION_IDENTIFIER = 2
            self.robot_name = rospy.get_param('~robot_name', "my_gen3_lite")
            self.degrees_of_freedom = rospy.get_param("/" + self.robot_name + "/degrees_of_freedom", 6)
            self.is_gripper_present = rospy.get_param("/" + self.robot_name + "/is_gripper_present", False)

            self.joint_sub = rospy.Subscriber('/my_gen3_lite/joint_states', JointState, self.joint_sub_callback)

            read_action_full_name = '/' + self.robot_name + '/base/read_action'
            rospy.wait_for_service(read_action_full_name)
            self.read_action = rospy.ServiceProxy(read_action_full_name, ReadAction)

            execute_action_full_name = '/' + self.robot_name + '/base/execute_action'
            rospy.wait_for_service(execute_action_full_name)
            self.execute_action = rospy.ServiceProxy(execute_action_full_name, ExecuteAction)

            play_joint_trajectory_full_name = '/' + self.robot_name + '/base/play_joint_trajectory'
            print(play_joint_trajectory_full_name)
            rospy.wait_for_service(play_joint_trajectory_full_name)
            self.play_joint_trajectory = rospy.ServiceProxy(play_joint_trajectory_full_name, PlayJointTrajectory)

        except Exception as e:
            print (e)
            self.is_init_success = False
        else:
            self.is_init_success = True

    def joint_sub_callback(self, data):
        self.my_robot_joint_state = data.position

    def example_home_the_robot(self):
        # The Home Action is used to home the robot. It cannot be deleted and is always ID #2:
        self.last_action_notif_type = None
        req = ReadActionRequest()
        req.input.identifier = self.HOME_ACTION_IDENTIFIER
        try:
            res = self.read_action(req)
        except rospy.ServiceException:
            rospy.logerr("Failed to call ReadAction")
            return False
        # Execute the HOME action if we could read it
        else:
            # What we just read is the input of the ExecuteAction service
            req = ExecuteActionRequest()
            req.input = res.output
            rospy.loginfo("Sending the robot home...")
            try:
                self.execute_action(req)
            except rospy.ServiceException:
                rospy.logerr("Failed to call ExecuteAction")
                return False

    def change_joint_angle(self, num, angle):
        # joint_positions = self.arm_group.get_current_joint_values()

        if num == 1:
            req = PlayJointTrajectoryRequest()
            temp_angle = JointAngle()
            temp_angle.joint_identifier = 0
            cur_target_angle = self.my_robot_joint_state
            temp_angle.value = cur_target_angle[0] + math.radians(float(angle))
            print('target angle',temp_angle.value)
            req.input.joint_angles.joint_angles.append(temp_angle)
            if temp_angle.value < math.radians(-154.1) or temp_angle.value > math.radians(154.1):
                print("Give an angle between (-154.1, +154.1)")
                temp_angle.value = cur_target_angle[0]
                return False

            for i in range(1,6):
                temp_angle = JointAngle()
                temp_angle.joint_identifier = i
                print(cur_target_angle[i])
                temp_angle.value = cur_target_angle[i]
                req.input.joint_angles.joint_angles.append(temp_angle)

            try:
                self.play_joint_trajectory(req)
                print('play_joint_trajectory')
            except rospy.ServiceException:
                rospy.logerr("Failed to call PlayJointTrajectory")
                return False
        # elif num == 2:
        #     current = joint_positions[1]
        #     angle = angle / 180
        #     joint_positions[1] += angle
        #     if joint_positions[1] < -150.1 / 180 or joint_positions[1] > 150.1 / 180:
        #         print("Give an angle between (-150.1, +150.1")
        #         joint_positions[1] = current
        #     else:
        #         self.arm_group.set_joint_value_target(joint_positions)
        #         self.arm_group.go(wait=True)
        # elif num == 3:
        #     current = joint_positions[2]
        #     angle = angle / 180
        #     joint_positions[2] += angle
        #     if joint_positions[2] < -150.1 / 180 or joint_positions[2] > 150.1 / 180:
        #         print("Give an angle between (-150.1, +150.1")
        #         joint_positions[2] = current
        #     else:
        #         self.arm_group.set_joint_value_target(joint_positions)
        #         self.arm_group.go(wait=True)
        # elif num == 4:
        #     current = joint_positions[3]
        #     angle = angle / 180
        #     joint_positions[3] += angle
        #     if joint_positions[3] < -148.98 / 180 or joint_positions[3] > 148.98 / 180:
        #         print("Give an angle between (-148.98, +148.98")
        #         joint_positions[3] = current
        #     else:
        #         self.arm_group.set_joint_value_target(joint_positions)
        #         self.arm_group.go(wait=True)
        # elif num == 5:
        #     current = joint_positions[4]
        #     angle = angle / 180
        #     joint_positions[4] += angle
        #     if joint_positions[4] < -144.97 / 180 or joint_positions[4] > 145.0 / 180:
        #         print("Give an angle between (-144.97, +145.0")
        #         joint_positions[4] = current
        #     else:
        #         self.arm_group.set_joint_value_target(joint_positions)
        #         self.arm_group.go(wait=True)
        # elif num == 6:
        #     current = joint_positions[5]
        #     angle = angle / 180
        #     joint_positions[5] += angle
        #     if joint_positions[5] < -148.98 / 180 or joint_positions[5] > 148.98 / 180:
        #         print("Give an angle between (-148.98, +148.98")
        #         joint_positions[5] = current
        #     else:
        #         self.arm_group.set_joint_value_target(joint_positions)
        #         self.arm_group.go(wait=True)
        # else:
        #     print("Give a number between 1 to 6.")


if __name__ == '__main__':
    example = ExampleFullArmMovement()
    while raw_input("Press q to quit : ") != 'q':
        example.example_home_the_robot()
        joint_num = input("Give the number of joint to rotate : ")
        joint_angle = input("Give the rotation angle in degrees : ")
        example.change_joint_angle(joint_num, joint_angle)

