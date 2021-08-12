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
from std_msgs.msg import Float64MultiArray, Int32, Bool
from cv_bridge import CvBridge, CvBridgeError
from control_msgs.msg import GripperCommand
import time
from moveit_msgs.msg import Constraints, JointConstraint

sys.path.append('/home/tidy/Pycharmprojects/gulliver_project/kinova/yolo.py')
from yolo import net, meta, detect, draw_bounding_box

class MoveRobotWithVR(object):
    def __init__(self):

        # Initialize the node
        super(MoveRobotWithVR, self).__init__()
        moveit_commander.roscpp_initialize(sys.argv)

        rospy.init_node('move_end_effector')
        self.sub_image = rospy.Subscriber("/camera/color/image_raw", Image, self.image_callback)
        self.sub_pc = rospy.Subscriber("/camera/depth_registered/points", PointCloud2, self.pc_callback)
        self.obj_id_sub = rospy.Subscriber('/grabbed_object_id', Int32, self.obj_id_callback)

        self.coordinate_list = []
        self.center_pointcloud = (None, None, None) # x, y, z in pointcloud
        self.center_rgb_list = []
        self.name_list = []
        self.object_id = None
        self.obj_id_num = 0
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
                self.name_list.append(name)
                if prob >= 0.01:
                    img = draw_bounding_box(cv_image, item)
                    center_rgb = (int(box_info[0]), int(box_info[1]))
                    self.center_rgb_list.append(center_rgb)
                    print(center_rgb)
            self.disable_img_sub = True

        cv2.imshow('img', cv_image)
        cv2.waitKey(3)

    def pc_callback(self, point_msg):
        self.pc = ros_numpy.numpify(point_msg)

    def obj_id_callback(self, received_id):
        if self.object_id != received_id.data and received_id.data is not None:
            self.obj_id_num += 1
            self.object_id = received_id.data
        else:
            if self.obj_id_num == 3:
                self.obj_id_num = -1

    def get_object_pc(self):
        for center_rgb in self.center_rgb_list:
            x = center_rgb[0]
            y = center_rgb[1]
            self.coordinate_list.append([self.pc[y][x][0], self.pc[y][x][1], self.pc[y][x][2]])
        return self.coordinate_list

    def rotate_joints(self):
        joint_positions = self.arm_group.get_current_joint_values()
        joint_positions[0] += self.angle
        joint_positions[4] = math.radians(60)
        self.arm_group.set_joint_value_target(joint_positions)
        self.arm_group.go(wait=True)

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

    def set_rotation_constraint(self):
        self.arm_group.clear_path_constraints()
        constraint = Constraints()
        joint_constraint_1 = JointConstraint()

        joint_constraint_1.position = self.arm_group.get_current_joint_values()[0]
        joint_constraint_1.tolerance_above = math.pi / 6
        joint_constraint_1.tolerance_below = math.pi / 6
        joint_constraint_1.joint_name = 'joint_1'
        joint_constraint_1.weight = 1
        constraint.joint_constraints.append(joint_constraint_1)

        joint_constraint_2 = JointConstraint()
        joint_constraint_2.position = 0
        joint_constraint_2.tolerance_above = math.pi
        joint_constraint_2.tolerance_below = 0
        joint_constraint_2.joint_name = 'joint_3'
        joint_constraint_2.weight = 1
        constraint.joint_constraints.append(joint_constraint_2)

        self.arm_group.set_path_constraints(constraint)

    def set_base_constraint(self):
        self.arm_group.clear_path_constraints()
        constraint = Constraints()
        joint_constraint_1 = JointConstraint()

        joint_constraint_1.position = self.arm_group.get_current_joint_values()[0]
        joint_constraint_1.tolerance_above = math.pi / 6
        joint_constraint_1.tolerance_below = math.pi / 6
        joint_constraint_1.joint_name = 'joint_1'
        joint_constraint_1.weight = 1
        constraint.joint_constraints.append(joint_constraint_1)

    def set_pose(self, coordinate, obj_name):
        x = coordinate[0]
        y = coordinate[1]
        z = coordinate[2]

        cur_pose = self.arm_group.get_current_pose().pose

        cup_offset_x_left = 0.44 - 0.05
        cup_offset_x_right = 0.44 - 0.05
        cup_offset_y = 0
        cup_offset_z = 0.05
        bottle_offset_x_left = 0.44 - 0.13
        bottle_offset_x_right = 0.44 - 0.13
        bottle_offset_y = 0
        bottle_offset_z = 0.2
        teddy_bear_offset_x_left = 0.44 + 0.04
        teddy_bear_offset_x_right = 0.44 + 0.04
        teddy_bear_offset_y = 0
        teddy_bear_offset_z = 0.03

        cup_offset_list = [cup_offset_x_left, cup_offset_x_right, cup_offset_y, cup_offset_z]
        bottle_offset_list = [bottle_offset_x_left, bottle_offset_x_right, bottle_offset_y, bottle_offset_z]
        teddy_bear_offset_list = [teddy_bear_offset_x_left, teddy_bear_offset_x_right, teddy_bear_offset_y, teddy_bear_offset_z]

        self.obj_dict = {"cup": cup_offset_list, "bottle": bottle_offset_list, "teddy bear": teddy_bear_offset_list}
        if x < 0:
            cur_pose.position.x += (-y / math.sin(58)) + self.obj_dict[obj_name][0]
        else:
            cur_pose.position.x += (-y / math.sin(58)) + self.obj_dict[obj_name][1]
        cur_pose.position.y += -x + self.obj_dict[obj_name][2]
        cur_pose.position.z = self.obj_dict[obj_name][3]

        self.angle = math.atan(cur_pose.position.y / cur_pose.position.x)

        return cur_pose

    def pick_bottle(self, pc_coordinate):
        first_pose = self.arm_group.get_current_pose().pose
        self.rotate_joints()
        second_pose = self.arm_group.get_current_pose().pose
        position_diff_x = second_pose.position.x - first_pose.position.x
        position_diff_y = second_pose.position.y - first_pose.position.y

        goal_pose = self.set_pose(pc_coordinate, 'bottle')
        goal_pose.position.x -= position_diff_x
        goal_pose.position.y -= position_diff_y
        self.arm_group.set_pose_target(goal_pose)
        self.set_rotation_constraint()
        self.arm_group.go(wait=True)
        self.arm_group.clear_path_constraints()

        cur_pose = self.arm_group.get_current_pose().pose
        cur_pose.position.z -= 0.1
        self.set_base_constraint()
        self.arm_group.set_pose_target(cur_pose)
        self.arm_group.go(wait=True)
        self.arm_group.clear_path_constraints()

    def pick_teddy_bear(self, pc_coordinate):
        goal_pose = self.set_pose(pc_coordinate, 'teddy bear')
        self.arm_group.set_pose_target(goal_pose)
        self.arm_group.go(wait=True)

    def pick_cup(self, pc_coordinate):
        first_pose = self.arm_group.get_current_pose().pose
        self.rotate_joints()
        second_pose = self.arm_group.get_current_pose().pose
        position_diff_x = second_pose.position.x - first_pose.position.x
        position_diff_y = second_pose.position.y - first_pose.position.y

        goal_pose = self.set_pose(pc_coordinate, 'cup')
        goal_pose.position.x -= position_diff_x + 0.1
        goal_pose.position.y -= position_diff_y
        self.arm_group.set_pose_target(goal_pose)
        self.arm_group.go(wait=True)

        cur_pose = self.arm_group.get_current_pose().pose
        cur_pose.position.x += 0.15

        self.arm_group.set_pose_target(cur_pose)
        self.arm_group.go(wait=True)

        return cur_pose

    def pick(self):
        gripper_value = self.gripper_group.get_current_joint_values()
        gripper_value[0] = -0.07
        gripper_value[2] = 0.07
        self.gripper_group.set_joint_value_target(gripper_value)
        self.gripper_group.go(wait=True)

    def open_gripper(self):
        gripper_value = self.gripper_group.get_current_joint_values()
        gripper_value[0] = -0.96
        gripper_value[2] = 0.96
        self.gripper_group.set_joint_value_target(gripper_value)
        self.gripper_group.go(wait=True)

    def place(self, id):
        place_dict = {2: [0.08, 0.5, 0.25], 0: [-0.2, -0.15, 0.25], 1: [-0.2, 0.15, 0.25]}

        if id == 2:
            joint_positions = self.arm_group.get_current_joint_values()
            joint_positions[0] = 0.0001241033067976925
            joint_positions[1] = -0.28029983525971147
            joint_positions[2] = 1.3112899206943054
            joint_positions[3] = -0.0009912285577282631
            joint_positions[4] = -1.0472843702481347
            joint_positions[5] = -0.00042184471666839585
            self.arm_group.set_joint_value_target(joint_positions)
            self.arm_group.go(wait=True)

            cur_pose = self.arm_group.get_current_pose().pose
            cur_pose.position.x = place_dict[id][0]
            cur_pose.position.y = place_dict[id][1]
            cur_pose.position.z = place_dict[id][2]
            self.arm_group.set_pose_target(cur_pose)
            self.arm_group.go(wait=True)

        else:
            joint_positions = self.arm_group.get_current_joint_values()
            joint_positions[0] = -0.13965510229931954
            joint_positions[1] = -0.5898539624617261
            joint_positions[2] = 0.11399847467360485
            joint_positions[3] = 1.549371728457427
            joint_positions[4] = 2.3046921505035205
            joint_positions[5] = -1.7319165074802543
            self.arm_group.set_joint_value_target(joint_positions)
            self.arm_group.go(wait=True)

            cur_pose = self.arm_group.get_current_pose().pose
            cur_pose.position.x = place_dict[id][0]
            cur_pose.position.y = place_dict[id][1]
            cur_pose.position.z = place_dict[id][2]
            self.arm_group.set_pose_target(cur_pose)
            self.arm_group.go(wait=True)


if __name__ == '__main__':
    moveclass = MoveRobotWithVR()
    # init variables
    moveclass.home_pose()
    moveclass.center_rgb_list = []
    object_list = []

    moveclass.disable_img_sub = False
    while len(moveclass.center_rgb_list) != 3:
        pass
    if len(moveclass.center_rgb_list) == 3:
        # todo: try without initializing the value
        pc_coordinate = moveclass.get_object_pc()
        for i, name in enumerate(moveclass.name_list):
            if name == 'bottle':
                object_list += [0.0, pc_coordinate[i][0], pc_coordinate[i][1]]
            elif name == 'teddy bear':
                object_list += [1.0, pc_coordinate[i][0], pc_coordinate[i][1]]
            elif name == 'cup' or name == 'bowl':
                object_list += [2.0, pc_coordinate[i][0], pc_coordinate[i][1]]

        obj_list_pub = rospy.Publisher('object_list', Float64MultiArray, queue_size=20)
        pub_array = Float64MultiArray()
        pub_array.data = object_list
        start_time = time.time()
        while time.time() <= start_time + 5:
            obj_list_pub.publish(pub_array)

    activation_pub = rospy.Publisher('vr_activate', Bool, queue_size=20)
    object_id = None
    activation_pub.publish(True)

    sub_list_1 = [object_list[0], object_list[1], object_list[2]]
    sub_list_2 = [object_list[3], object_list[4], object_list[5]]
    sub_list_3 = [object_list[6], object_list[7], object_list[8]]
    object_list = [sub_list_1, sub_list_2, sub_list_3]
    while moveclass.object_id is None:
        pass
    if moveclass.object_id is not None:
        while moveclass.obj_id_num != -1:
            if object_id == moveclass.object_id:
                continue
            object_id = moveclass.object_id
            activation_pub.publish(False)
            for object_coordinate in object_list:
                if int(object_coordinate[0]) == object_id:
                    object_name_idx = 0
                    if object_id == 0:
                        object_name_idx = moveclass.name_list.index('bottle')
                        moveclass.pick_bottle(moveclass.coordinate_list[object_name_idx])
                    elif object_id == 1:
                        object_name_idx = moveclass.name_list.index('teddy bear')
                        moveclass.pick_teddy_bear(moveclass.coordinate_list[object_name_idx])
                    else:
                        if 'cup' in moveclass.name_list:
                            object_name_idx = moveclass.name_list.index('cup')
                        else:
                            object_name_idx = moveclass.name_list.index('cup')
                        moveclass.pick_cup(moveclass.coordinate_list[object_name_idx])
                    moveclass.pick()
                    moveclass.place(object_id)
                    moveclass.open_gripper()
                    moveclass.home_pose()
                    activation_pub.publish(True)














