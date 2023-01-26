import sys
import os

if 'ROS_NAMESPACE' not in os.environ:
    os.environ['ROS_NAMESPACE'] = 'my_gen3_lite'

import rospy
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg
import cv2
import ros_numpy
import math
from sensor_msgs.msg import Image
from sensor_msgs.msg import PointCloud2
from cv_bridge import CvBridge, CvBridgeError
from control_msgs.msg import GripperCommand
from moveit_msgs.msg import Constraints, JointConstraint, PositionConstraint

sys.path.append('/home/tidy/Pycharmprojects/gulliver_project/kinova/yolo.py')
from yolo import net, meta, detect, draw_bounding_box

class MoveWithYolo(object):
    def __init__(self):

        # Initialize the node
        super(MoveWithYolo, self).__init__()
        moveit_commander.roscpp_initialize(sys.argv)

        rospy.init_node('move_end_effector')
        self.sub_image = rospy.Subscriber("/camera/color/image_raw", Image, self.image_callback)
        self.sub_pc = rospy.Subscriber("/camera/depth_registered/points", PointCloud2, self.pc_callback)

        self.coordinate_list = []
        self.center_rgb_list = [] # x, y in rgb image
        self.center_pointcloud = (None, None, None) # x, y, z in pointcloud
        self.picked = []
        self.name_list = []
        self.obj_dict = {}
        self.angle = 0

        self.disable_img_sub = True

        try:
            self.is_gripper_present = rospy.get_param(rospy.get_namespace() + "is_gripper_present", False)
            if self.is_gripper_present:
                gripper_joint_names = rospy.get_param(rospy.get_namespace() + "gripper_joint_names", [])
                self.gripper_joint_name = gripper_joint_names[0]
            else:
                self.gripper_joint_name = ""
            self.degrees_of_freedom = rospy.get_param(rospy.get_namespace() + "degrees_of_freedom", 7)
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
            self.arm_group.set_max_acceleration_scaling_factor(1)

            if self.is_gripper_present:
                gripper_group_name = "gripper"
                self.gripper_group = moveit_commander.MoveGroupCommander(gripper_group_name, ns=rospy.get_namespace())


        except Exception as e:
            print (e)
            self.is_init_success = False
        else:
            self.is_init_success = True

    def home_pose(self):
        joint_positions = self.arm_group.get_current_joint_values()
        joint_positions[0] = -0.07124435284958164
        joint_positions[1] = 0.28432546956347565
        joint_positions[2] = 0.8544049282991455
        joint_positions[3] = 1.5463900533009745
        joint_positions[4] = 1.9835516746630073
        joint_positions[5] = -1.563087008071335
        self.arm_group.set_joint_value_target(joint_positions)
        self.arm_group.go(wait=True)
        return self.arm_group.get_current_pose().pose

    def image_callback(self, img_msg):
        bridge = CvBridge()
        cv_image = bridge.imgmsg_to_cv2(img_msg, "passthrough")
        cv_image = cv2.cvtColor(cv_image, cv2.COLOR_BGR2RGB)
        if self.disable_img_sub == False:
            r = detect(net, meta, cv_image)
            for item in r:
                name, prob, box_info = item
                if name in ['teddy bear', 'bottle', 'cup']:
                    self.name_list.append(name)
                if prob >= 0.01:
                    img = draw_bounding_box(cv_image, item)
                    center_rgb = (int(box_info[0]), int(box_info[1]))
                    self.center_rgb_list.append(center_rgb)
            self.disable_img_sub = True

        cv2.imshow('img', cv_image)
        cv2.waitKey(3)

    def pc_callback(self, point_msg):
        self.pc = ros_numpy.numpify(point_msg)

    def get_object_pc(self):
        for center_rgb in self.center_rgb_list:
            x = center_rgb[0]
            y = center_rgb[1]
            while math.isnan(self.pc[y][x][0]) or math.isnan(self.pc[y][x][1]):
                pass
            self.coordinate_list.append([self.pc[y][x][0], self.pc[y][x][1], self.pc[y][x][2]])
        return self.coordinate_list

    def rotate_joints(self, name):
        joint_positions = self.arm_group.get_current_joint_values()
        joint_positions[0] += self.angle
        joint_positions[4] = math.radians(60)
        self.arm_group.set_joint_value_target(joint_positions)
        self.arm_group.go(wait=True)

    def set_pose(self, coordinate, name):
        x = coordinate[0]
        y = coordinate[1]
        z = coordinate[2]

        cur_pose = self.arm_group.get_current_pose().pose

        # cup_offset_x_left = 0.35
        # cup_offset_x_right = 0.3
        # cup_offset_y = 0
        # cup_offset_z = 0.16
        # bottle_offset_x_left = 0.20
        # bottle_offset_x_right = 0.25
        # bottle_offset_y = 0
        # bottle_offset_z = 0.2
        # teddy_bear_offset_x_left = 0.31
        # teddy_bear_offset_x_right = 0.31
        # teddy_bear_offset_y = 0
        # teddy_bear_offset_z = 0.05

        cup_offset_x_left = -0.1
        cup_offset_x_right = -0.1
        cup_offset_y = 0
        cup_offset_z = 0.16
        bottle_offset_x_left = -0.1
        bottle_offset_x_right = -0.1
        bottle_offset_y = 0
        bottle_offset_z = 0.08
        teddy_bear_offset_x_left = -0.1
        teddy_bear_offset_x_right = -0.1
        teddy_bear_offset_y = 0
        teddy_bear_offset_z = 0.05

        cup_offset_list = [cup_offset_x_left, cup_offset_x_right, cup_offset_y, cup_offset_z]
        bottle_offset_list = [bottle_offset_x_left, bottle_offset_x_right, bottle_offset_y, bottle_offset_z]
        teddy_bear_offset_list = [teddy_bear_offset_x_left, teddy_bear_offset_x_right, teddy_bear_offset_y, teddy_bear_offset_z]

        self.obj_dict = {"cup": cup_offset_list, "bottle": bottle_offset_list, "teddy bear": teddy_bear_offset_list}
        radian_58 = math.radians(58)
        if x < 0:
            cur_pose.position.x += (z * math.sin(radian_58)) - (y * math.cos(radian_58)) + self.obj_dict[name][0]
        else:
            cur_pose.position.x += (z * math.sin(radian_58)) - (y * math.cos(radian_58)) + self.obj_dict[name][1]
        cur_pose.position.y += -x + self.obj_dict[name][2]
        cur_pose.position.z = self.obj_dict[name][3]

        self.angle = math.atan(cur_pose.position.y / cur_pose.position.x)

        return cur_pose

    def pick_bottle(self, pc_coordinate):
        first_pose = self.arm_group.get_current_pose().pose
        self.rotate_joints('bottle')
        second_pose = self.arm_group.get_current_pose().pose
        position_diff_x = second_pose.position.x - first_pose.position.x
        position_diff_y = second_pose.position.y - first_pose.position.y

        goal_pose = self.set_pose(pc_coordinate, 'bottle')
        goal_pose.position.x -= position_diff_x
        goal_pose.position.y -= position_diff_y
        # self.set_base_constraint()
        self.arm_group.set_pose_target(goal_pose)
        print("bottle goal {0}".format(goal_pose))
        try:
            is_success = self.arm_group.go(wait=True)
            print(is_success)
        except Exception as e:
            print("error message ", e)

        cur_pose = self.arm_group.get_current_pose().pose
        cur_pose.position.x += 0.1
        cur_pose.position.z -= 0.13
        self.set_joint_5_constraint()
        self.arm_group.set_pose_target(cur_pose)
        try:
            self.arm_group.go(wait=True)
        except Exception as e:
            print("error message ", e)

    def pick_teddy_bear(self, pc_coordinate):
        goal_pose = self.set_pose(pc_coordinate, 'teddy bear')
        # self.set_base_constraint()
        self.arm_group.set_pose_target(goal_pose)
        print("teddy bear goal {0}".format(goal_pose))
        self.arm_group.go(wait=True)

    def pick_cup(self, pc_coordinate):
        first_pose = self.arm_group.get_current_pose().pose
        self.rotate_joints('cup')
        second_pose = self.arm_group.get_current_pose().pose
        position_diff_x = second_pose.position.x - first_pose.position.x
        position_diff_y = second_pose.position.y - first_pose.position.y

        goal_pose = self.set_pose(pc_coordinate, 'cup')
        goal_pose.position.x -= position_diff_x + 0.1
        goal_pose.position.y -= position_diff_y
        # self.set_base_constraint()
        self.arm_group.set_pose_target(goal_pose)
        print("cup goal {0}".format(goal_pose))
        is_success = self.arm_group.go(wait=True)
        print(is_success)

        cur_pose = self.arm_group.get_current_pose().pose
        cur_pose.position.x += 0.1

        self.arm_group.set_pose_target(cur_pose)
        # self.set_joint_5_constraint()
        is_fuck_sucess = self.arm_group.go(wait=True)
        print('is_fuck_sucess', is_fuck_sucess)



    def set_base_constraint(self):
        self.arm_group.clear_path_constraints()
        constraint = Constraints()
        joint_constraint = JointConstraint()
        joint_1 = self.arm_group.get_current_joint_values()[0]
        joint_constraint.position = joint_1
        joint_constraint.tolerance_above = math.pi / 4
        joint_constraint.tolerance_below = math.pi / 4
        joint_constraint.joint_name = 'joint_1'
        joint_constraint.weight = 1
        constraint.joint_constraints.append(joint_constraint)
        self.arm_group.set_path_constraints(constraint)

    def set_joint_5_constraint(self):
        self.arm_group.clear_path_constraints()
        constraint = Constraints()
        joint_constraint = JointConstraint()
        if self.arm_group.get_current_joint_values()[4] > 0:
            joint_constraint.position = self.arm_group.get_current_joint_values()[4]
            joint_constraint.tolerance_above = math.pi * 2
            joint_constraint.tolerance_below = math.pi / 18
            joint_constraint.joint_name = 'joint_5'
            joint_constraint.weight = 1
        else:
            joint_constraint.position = self.arm_group.get_current_joint_values()[4]
            joint_constraint.tolerance_above = math.pi / 18
            joint_constraint.tolerance_below = math.pi * 2
            joint_constraint.joint_name = 'joint_5'
            joint_constraint.weight = 1
        constraint.joint_constraints.append(joint_constraint)
        self.arm_group.set_path_constraints(constraint)

    def pick(self):
        gripper_value = self.gripper_group.get_current_joint_values()
        gripper_value[0] = 0
        gripper_value[2] = 0
        self.gripper_group.set_joint_value_target(gripper_value)
        self.gripper_group.go(wait=True)

    def place_1(self):
        basket_position = [0, 0, 0]
        basket_position[0] = -0.19
        basket_position[1] = -0.23
        basket_position[2] = 0.15
        self.arm_group.set_position_target(basket_position)
        self.arm_group.go(wait=True)

    def place_2(self):
        basket_position = [0, 0, 0]
        basket_position[0] = 0.18
        basket_position[1] = -0.52
        basket_position[2] = 0.15
        self.arm_group.set_position_target(basket_position)
        self.arm_group.go(wait=True)

    def place_3(self):
        basket_position = [0, 0, 0]
        basket_position[0] = -0.19
        basket_position[1] = 0.19
        basket_position[2] = 0.15
        self.arm_group.set_position_target(basket_position)
        self.arm_group.go(wait=True)


    def open_gripper(self):
        gripper_value = self.gripper_group.get_current_joint_values()
        gripper_value[0] = -0.96
        gripper_value[2] = 0.96
        self.gripper_group.set_joint_value_target(gripper_value)
        self.gripper_group.go(wait=True)



if __name__ == '__main__':
    moveclass = MoveWithYolo()
    # init variables

    print('1. [Start] go to home pose')
    moveclass.home_pose()
    moveclass.open_gripper()
    print('1. [End] go to home pose')

    print('2. [Start] Yolo detection')
    moveclass.disable_img_sub = False
    while len(moveclass.center_rgb_list) == 0:
        print('Detecting ...')
        rospy.sleep(1)
    print('2. [End] Yolo detection')

    obj_name_list = moveclass.name_list
    pointcloud_list = moveclass.get_object_pc()
    for name in obj_name_list:
        if name == 'cup' or name == 'bowl':
            cup_index = obj_name_list.index(name)
            obj_name_list.remove(name)
            obj_name_list.append(name)
            cup_pc = pointcloud_list[cup_index]
            del pointcloud_list[cup_index]
            pointcloud_list.append(cup_pc)

    for center_pointcloud in pointcloud_list:
        cur_object_name = moveclass.name_list[pointcloud_list.index(center_pointcloud)]
        print('target : {0}'.format(cur_object_name))
        print('3. [Start] go to target pose')
        if cur_object_name == 'bottle':
            moveclass.pick_bottle(center_pointcloud)
        elif cur_object_name == 'teddy bear' or cur_object_name == 'broccoli':
            moveclass.pick_teddy_bear(center_pointcloud)
        else:
            moveclass.pick_cup(center_pointcloud)
        print('3. [End] go to target pose')
        moveclass.arm_group.clear_path_constraints()
        print('4. [Start] Pick {0}'.format(cur_object_name))
        moveclass.pick()
        print('4. [End] Pick {0}'.format(cur_object_name))
        print('5. [Start] Place')
        moveclass.place_3()
        print('6. [End] Place')
        moveclass.home_pose()
        moveclass.open_gripper()






