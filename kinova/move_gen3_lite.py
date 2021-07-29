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

        self.center_rgb = (None, None) # x, y in rgb image
        self.center_pointcloud = (None, None, None) # x, y, z in pointcloud
        self.picked = []
        self.name = None
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
            item = r[0]
            name, prob, box_info = item
            self.name = name
            if prob >= 0.01:
                img = draw_bounding_box(cv_image, item)
                self.center_rgb = (int(box_info[0]), int(box_info[1]))
                self.disable_img_sub = True

        cv2.imshow('img', cv_image)
        cv2.waitKey(3)

    def pc_callback(self, point_msg):
        self.pc = ros_numpy.numpify(point_msg)

    def get_object_pc(self):
        center = self.center_rgb
        x = center[0]
        y = center[1]
        return (self.pc[y][x][0], self.pc[y][x][1], self.pc[y][x][2])


    def target_pose(self, coordinate):
        x = coordinate[0]
        y = coordinate[1]
        z = coordinate[2]

        cur_pose = self.arm_group.get_current_pose().pose
        cur_pose.position.x += -y + 0.36
        cur_pose.position.y += -x
        cur_pose.position.z += -z + 0.3

        return cur_pose

    def pick(self):
        gripper_joint = self.robot.get_joint('right_finger_bottom_joint')
        gripper_value = self.gripper_group.get_current_joint_values()
        gripper_value[0] = 0
        gripper_value[2] = 0
        self.gripper_group.set_joint_value_target(gripper_value)
        self.gripper_group.go(wait=True)

    def open_gripper(self):
        gripper_joint = self.robot.get_joint('right_finger_bottom_joint')
        gripper_value = self.gripper_group.get_current_joint_values()
        gripper_value[0] = -0.96
        gripper_value[2] = 0.96
        self.gripper_group.set_joint_value_target(gripper_value)
        self.gripper_group.go(wait=True)



if __name__ == '__main__':
    moveclass = MoveWithYolo()
    # init variables
    moveclass.center_rgb = (None, None)

    print('1. [Start] go to home pose')
    moveclass.home_pose()
    print('1. [End] go to home pose')
    print('2. [Start] Yolo detection')
    moveclass.disable_img_sub = False
    while moveclass.center_rgb[0] is None:
        rospy.sleep(1)

    print('2. [End] Yolo detection')
    center_pointcloud = moveclass.get_object_pc()
    cur_object_name = moveclass.name
    print('{0} target : {1}'.format(cur_object_name, moveclass.target_pose(center_pointcloud)))
    print('3. [Start] go to target pose')
    moveclass.arm_group.set_pose_target(moveclass.target_pose(center_pointcloud))
    moveclass.arm_group.go(wait=True)
    print('3. [End] go to target pose')
    print('4. [Start] Pick', cur_object_name)
    moveclass.pick()
    print('4. [End] Pick', cur_object_name)
    moveclass.open_gripper()
    moveclass.home_pose()




