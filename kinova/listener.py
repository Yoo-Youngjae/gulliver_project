import rospy
from geometry_msgs.msg import Point

def listener():

    rospy.init_node('listener', anonymous=True)

    rospy.Subscriber('rightHandPoint', Point, callback) #TOPIC

    rospy.spin()

def callback(data):
    print(data);
    
if __name__ == '__main__':
    listener()
