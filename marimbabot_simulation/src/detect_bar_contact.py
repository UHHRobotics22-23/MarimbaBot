# Detects contacts with bars in Gazebo

#!/usr/bin/env python

import rospy


def callback(data):
    rospy.loginfo(rospy.get_caller_id() + "I heard %s", data.data)


def listener():
    rospy.init_node("listener", anonymous=True)

    # subscribe to joint states 

    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()


if __name__ == "__main__":
    listener()
