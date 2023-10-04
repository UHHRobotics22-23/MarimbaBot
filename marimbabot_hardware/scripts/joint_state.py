#!/usr/bin/env python

import rospy
from sensor_msgs.msg import JointState
from moveit_msgs.msg import MoveGroupActionGoal
from std_msgs.msg import Header


def JointStatePublisher():
    pub = rospy.Publisher("/joint_states", JointState, queue_size=10)
    rospy.init_node("joint_state_publisher")
    rate = rospy.Rate(100)  # 100hz

    while not rospy.is_shutdown():
        goal_msg: MoveGroupActionGoal = rospy.wait_for_message("/move_group/goal", MoveGroupActionGoal)

        constraints = goal_msg.goal.request.goal_constraints

        mallet_finger_position = None
        for constraint in constraints:
            for joint_constraint in constraint.joint_constraints:
                if joint_constraint.joint_name == "mallet_finger":
                    mallet_finger_position = joint_constraint.position
                    break

        if mallet_finger_position is not None:
            Joint_state_pub = JointState()
            Joint_state_pub.header = Header()
            Joint_state_pub.header.stamp = rospy.Time.now()
            Joint_state_pub.name.append("mallet_finger")
            Joint_state_pub.position.append(mallet_finger_position)
            # Joint_state_pub.position.append(0.0)
            pub.publish(Joint_state_pub)

        rate.sleep()


if __name__ == "__main__":
    try:
        JointStatePublisher()
    except rospy.ROSInterruptException:
        pass
