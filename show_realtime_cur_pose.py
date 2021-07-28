import sys
import os
import rospy
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg
import cv2
import ros_numpy
from math import pi, radians
from std_srvs.srv import Empty
from moveit_msgs.msg import Constraints
from moveit_msgs.msg import JointConstraint
from tf.transformations import quaternion_from_euler, euler_from_quaternion
from sensor_msgs.msg import Image
from sensor_msgs.msg import PointCloud2
from cv_bridge import CvBridge, CvBridgeError

if 'ROS_NAMESPACE' not in os.environ:
    os.environ['ROS_NAMESPACE'] = 'my_gen3_lite'

sys.path.append('/home/cylee/PycharmProjects/pythonProject')

class JointController(object):
    def __init__(self):
        rospy.init_node('show_realtime_cur_pose')
        # Initialize the node
        super(JointController, self).__init__()
        moveit_commander.roscpp_initialize(sys.argv)
        arm_group_name = "arm"
        self.arm_group = moveit_commander.MoveGroupCommander(arm_group_name, ns=rospy.get_namespace())
        self.arm_group.set_max_velocity_scaling_factor(1)
        self.arm_group.set_max_acceleration_scaling_factor(1)




if __name__ == '__main__':

    joint_controller = JointController()
    cur_pose = joint_controller.arm_group.get_current_pose().pose
    print(cur_pose)
    print(joint_controller.arm_group.get_current_joint_values())
