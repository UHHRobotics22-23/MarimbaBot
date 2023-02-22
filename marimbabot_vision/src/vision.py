#!/usr/bin/env python
import rospy
from std_msgs.msg import String
from sensor_msgs import Image

def callbackImage(data):    # TODO
    pass
#  rospy.loginfo()
    
def listener():

    # In ROS, nodes are uniquely named. If two nodes with the same
    # name are launched, the previous one is kicked off. The
    # anonymous=True flag means that rospy will choose a unique
    # name for our 'listener' node so that multiple listeners can
    # run simultaneously.
    rospy.init_node('vision_receiver', anonymous=True)

    rospy.Subscriber("cv_camera_node", Image, callbackImage)

    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()

if __name__ == '__main__':
    listener()
