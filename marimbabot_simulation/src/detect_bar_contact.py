# Detects contacts with bars in Gazebo

#!/usr/bin/env python

import rospy
from sensor_msgs.msg import JointState

import threshold_detector

print


# names of bars we are interested in
# corresponding joints are <bar_name>/joint
# e.g. "bar_A4" <-> "bar_A4/joint"
BAR_JOINT_NAMES = [
  "bar_A4/joint",
  "bar_A5/joint",
  "bar_A6/joint",
  "bar_As4/joint",
  "bar_As5/joint",
  "bar_As6/joint",
  "bar_B4/joint",
  "bar_B5/joint",
  "bar_B6/joint",
  "bar_C4/joint",
  "bar_C5/joint",
  "bar_C6/joint",
  "bar_C7/joint",
  "bar_Cs4/joint",
  "bar_Cs5/joint",
  "bar_Cs6/joint",
  "bar_D4/joint",
  "bar_D5/joint",
  "bar_D6/joint",
  "bar_Ds4/joint",
  "bar_Ds5/joint",
  "bar_Ds6/joint",
  "bar_E4/joint",
  "bar_E5/joint",
  "bar_E6/joint",
  "bar_F4/joint",
  "bar_F5/joint",
  "bar_F6/joint",
  "bar_Fs4/joint",
  "bar_Fs5/joint",
  "bar_Fs6/joint",
  "bar_G4/joint",
  "bar_G5/joint",
  "bar_G6/joint",
  "bar_Gs4/joint",
  "bar_Gs5/joint",
  "bar_Gs6/joint",
]

detector = threshold_detector.ThresholdDetector(pos_threshold=0.1, vel_threshold=0.1)


def log(msg, *args, **kwargs):
    rospy.loginfo("[detect_bar_contact] " + msg, *args, **kwargs)


def joint_states_callback(message):
    # parse joints of interest into a dict
    state = {}
    for i, name in zip(message.name):
        if name in BAR_JOINT_NAMES:
            state[name] = threshold_detector.PosVel(message.position[i], message.velocity[i])
    contacts = detector.update(state)
    return


def listener():
    rospy.init_node("detect_bar_contact")
    rospy.Subscriber("joint_states", JointState, joint_states_callback, queue_size=1)
    rospy.spin()


if __name__ == "__main__":
    listener()
