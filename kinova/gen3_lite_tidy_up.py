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
from moveit_msgs.msg import Constraints, JointConstraint

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

        self.pc_coordinate = []
        self.center_rgb = (None, None) # x, y in rgb image
        self.picked = []
        self.name = None
        self.name_list = []
        self.obj_dict = {}
        self.angle = 0
        self.obj_count = 0
        self.obj_box = (0, 0, 0, 0)
        self.min_point_y = 0
        self.min_y = 0
        self.min_x = 0

        self.success = True
        self.end_pick = False
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
            print(e)
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

        ### for show bounding box
        r = detect(net, meta, cv_image)
        for item in r:
            name, prob, box_info = item
            if prob >= 0.01:
                draw_bounding_box(cv_image, item)
                cv_image = cv2.circle(cv_image, (self.min_x, self.min_y), 2, (0, 255, 0), -1)
                cv_image = cv2.circle(cv_image, (320, 240), 3, (0, 0, 255), -1)

        cv2.imshow('img', cv_image)
        cv2.waitKey(3)
        ######
        if self.disable_img_sub == False:
            # item = (name, prob, (x, y, w, h))
            cup_list = []
            remove_list = []
            for item in r:
                if item[0] in ['cup', 'bowl']:
                    cup_list.append(item)
                    remove_list.append(item)
                elif item[0] not in ['cup', 'bowl', 'bottle', 'teddy bear']:
                    remove_list.append(item)
            for r_item in remove_list:
                r.remove(r_item)
            r += cup_list

            if len(r) == 0:
                self.end_pick = True
                return
            name, prob, box_info = r[0]
            if prob >= 0.01:
                self.name = name
                self.object_list = r
                self.obj_box = box_info
                self.center_rgb = (int(box_info[0]), int(box_info[1]))
            self.disable_img_sub = True


    def pc_callback(self, point_msg):
        self.pc = ros_numpy.numpify(point_msg)

    def get_object_pc(self, name):
        x = int(self.obj_box[0])
        y = int(self.obj_box[1])
        w = int(self.obj_box[2])
        h = int(self.obj_box[3])
        center_x = self.center_rgb[0]
        center_y = self.center_rgb[1]
        depth_min = 1
        min_x = 0
        min_y = 0

        for i in range(int(x-w/2), int(x+w/2)):
            for j in range(int(y-h/2), int(y+h/2)):
                if self.pc[j][i][2] < depth_min:
                    depth_min = self.pc[j][i][2]
                    if math.isnan(self.pc[j][i][1]):
                        continue
                    min_x = i
                    min_y = j
        self.min_x = min_x
        self.min_y = min_y

        goal_x = self.pc[center_y][center_x][0]
        goal_y = self.pc[min_y][min_x][1]
        while math.isnan(goal_x) or math.isnan(goal_y):
            print("Value is Nan")
            goal_x = self.pc[center_y][center_x][0]
            goal_y = self.pc[min_y][min_x][1]

        self.pc_coordinate = [goal_x, goal_y, depth_min]

        return self.pc_coordinate

    def rotate_joints(self):
        joint_positions = self.arm_group.get_current_joint_values()
        joint_positions[0] += self.angle
        joint_positions[4] = math.radians(60)
        self.arm_group.set_joint_value_target(joint_positions)
        self.arm_group.go(wait=True)

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
        teddy_bear_offset_z = 0.04

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

        success = self.arm_group.go(wait=True)
        for i in range(3):
            if success:
                break
            success = self.arm_group.go(wait=True)
        if not success:
            self.success = False
        self.arm_group.clear_path_constraints()

        cur_pose = self.arm_group.get_current_pose().pose
        cur_pose.position.z -= 0.1
        self.set_base_constraint()
        self.arm_group.set_pose_target(cur_pose)

        success = self.arm_group.go(wait=True)
        for i in range(3):
            if success:
                break
            success = self.arm_group.go(wait=True)
        if not success:
            self.success = False
        self.arm_group.clear_path_constraints()

    def pick_teddy_bear(self, pc_coordinate):
        goal_pose = self.set_pose(pc_coordinate, 'teddy bear')
        self.arm_group.set_pose_target(goal_pose)
        success = self.arm_group.go(wait=True)
        for i in range(3):
            if success:
                break
            success = self.arm_group.go(wait=True)
        if not success:
            self.success = False

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
        success = self.arm_group.go(wait=True)
        for i in range(3):
            if success:
                break
            success = self.arm_group.go(wait=True)
        if not success:
            self.success = False

        cur_pose = self.arm_group.get_current_pose().pose
        cur_pose.position.x += 0.15

        self.arm_group.set_pose_target(cur_pose)
        success = self.arm_group.go(wait=True)
        for i in range(3):
            if success:
                break
            success = self.arm_group.go(wait=True)
        if not success:
            self.success = False

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

    def set_joint_3_constraint(self):
        self.arm_group.clear_path_constraints()
        constraint = Constraints()
        joint_constraint = JointConstraint()
        joint_constraint.position = self.arm_group.get_current_joint_values()[2]
        joint_constraint.tolerance_above = math.pi * 2
        joint_constraint.tolerance_below = 0
        joint_constraint.joint_name = 'joint_3'
        joint_constraint.weight = 1
        constraint.joint_constraints.append(joint_constraint)
        self.arm_group.set_path_constraints(constraint)

    def pick(self):
        gripper_value = self.gripper_group.get_current_joint_values()
        gripper_value[0] = -0.07
        gripper_value[2] = 0.07
        self.gripper_group.set_joint_value_target(gripper_value)
        self.gripper_group.go(wait=True)

    def place(self, name):
        if name == 'bowl':
            name = 'cup'
        place_dict = {"cup": [0.08, 0.5, 0.25], "bottle": [-0.2, -0.15, 0.25], "teddy bear": [-0.2, 0.15, 0.25]}

        if name == 'cup':
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
            cur_pose.position.x = place_dict[name][0]
            cur_pose.position.y = place_dict[name][1]
            cur_pose.position.z = place_dict[name][2]
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

            if name == 'bottle':
                joint_positions = self.arm_group.get_current_joint_values()
                joint_positions[0] = -2.6651904188298783
                self.arm_group.set_joint_value_target(joint_positions)
                self.arm_group.go(wait=True)

                joint_positions = self.arm_group.get_current_joint_values()
                joint_positions[2] = 0.8050075511359198
                self.arm_group.set_joint_value_target(joint_positions)
                self.arm_group.go(wait=True)
            else:
                cur_pose = self.arm_group.get_current_pose().pose
                cur_pose.position.x = place_dict[name][0]
                cur_pose.position.y = place_dict[name][1]
                cur_pose.position.z = place_dict[name][2]
                self.arm_group.set_pose_target(cur_pose)
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
    while moveclass.center_rgb[0] is None:
        print('Detecting ...')
        rospy.sleep(1)
    print('2. [End] Yolo detection')

    while True:
        cur_object_name = moveclass.name
        center_pointcloud = moveclass.get_object_pc(cur_object_name)
        print('target : {0}'.format(cur_object_name))
        print('target object_list : ', moveclass.object_list)
        print('3. [Start] go to target pose')
        if cur_object_name == 'bottle':
            moveclass.pick_bottle(center_pointcloud)
        elif cur_object_name == 'teddy bear':
            moveclass.pick_teddy_bear(center_pointcloud)
        else:
            moveclass.pick_cup(center_pointcloud)
        print('3. [End] go to target pose')
        if moveclass.success:
            moveclass.arm_group.clear_path_constraints()
            print('4. [Start] Pick {0}'.format(cur_object_name))
            moveclass.pick()
            print('4. [End] Pick {0}'.format(cur_object_name))
            print('5. [Start] Place')
            moveclass.place(cur_object_name)
            moveclass.open_gripper()
            print('6. [End] Place')
            moveclass.home_pose()
            print("Home pose done!")
            rospy.sleep(2)
            moveclass.disable_img_sub = False
            rospy.sleep(2)
        else:
            moveclass.home_pose()
            print("Home pose done!")
            moveclass.success = True
            rospy.sleep(2)
            moveclass.disable_img_sub = False
            rospy.sleep(2)
        # end condition
        if moveclass.end_pick == True:
            print('End Tidy up! Bye Bye')
            break
