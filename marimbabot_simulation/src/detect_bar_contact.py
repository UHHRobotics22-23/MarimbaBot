# Detects contacts with bars in Gazebo

#!/usr/bin/env python

import rospy
from sensor_msgs.msg import JointState
from std_msgs.msg import String

# velocity threshold to count as a bar hit
VEL_THRESHOLD = 0.01

# names of bars we are interested in
# corresponding joints are <bar_name>/joint
# e.g. "bar_a4" <-> "bar_a4/joint"
bar_jOINT_NAMES = [
  "bar_a4/joint",
  "bar_a5/joint",
  "bar_a6/joint",
  "bar_ais4/joint",
  "bar_ais5/joint",
  "bar_ais6/joint",
  "bar_b4/joint",
  "bar_b5/joint",
  "bar_b6/joint",
  "bar_c4/joint",
  "bar_c5/joint",
  "bar_c6/joint",
  "bar_c7/joint",
  "bar_cis4/joint",
  "bar_cis5/joint",
  "bar_cis6/joint",
  "bar_d4/joint",
  "bar_d5/joint",
  "bar_d6/joint",
  "bar_dis4/joint",
  "bar_dis5/joint",
  "bar_dis6/joint",
  "bar_e4/joint",
  "bar_e5/joint",
  "bar_e6/joint",
  "bar_f4/joint",
  "bar_f5/joint",
  "bar_f6/joint",
  "bar_fis4/joint",
  "bar_fis5/joint",
  "bar_fis6/joint",
  "bar_g4/joint",
  "bar_g5/joint",
  "bar_g6/joint",
  "bar_gis4/joint",
  "bar_gis5/joint",
  "bar_gis6/joint",
]

joint2note = {
    joint: joint[4:joint.index("/")] for joint in bar_jOINT_NAMES
}

pub = None


def joint_states_callback(message):
    for i, name in enumerate(message.name):
        if name not in joint2note:
            continue
        absvel = abs(message.velocity[i])
        if absvel > VEL_THRESHOLD:
            rospy.loginfo(f"Detected contact with {joint2note[name]} vel={absvel:.5f}")
            pub.publish(joint2note[name])
    return


def listener():
    rospy.init_node("detect_bar_contact")
    global pub
    pub = rospy.Publisher("sim_notes", String, queue_size=10)
    rospy.Subscriber("joint_states", JointState, joint_states_callback, queue_size=1)
    rospy.spin()


if __name__ == "__main__":
    listener()
