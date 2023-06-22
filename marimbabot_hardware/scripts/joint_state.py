#!/usr/bin/env python

import rospy
from sensor_msgs.msg import JointState

def JointStatePublisher():
    pub = rospy.Publisher('/joint_states', JointState, queue_size=10)
    rospy.init_node('joint_state_publisher')
    rate = rospy.Rate(100) # 10hz
    Joint_state_pub = JointState()

    while not rospy.is_shutdown():
        Joint_state_pub.header.stamp = rospy.Time.now()
        Joint_state_pub.name.append("mallet_finger")
        Joint_state_pub.position.append(0.0)
        pub.publish(Joint_state_pub)
        rate.sleep()

if __name__ == '__main__':
    try:
        JointStatePublisher()
    except rospy.ROSInterruptException:
        pass