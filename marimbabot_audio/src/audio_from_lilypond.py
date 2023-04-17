#!/usr/bin/env python

import rospy
from sensor_msgs.msg import Image as ROSImage
from std_msgs.msg import String


def create_audio_from_lilypond(sentence):

    return sentence

def callback_vision_results(data: String, callback_args):
    rospy.logdebug("received recognized sentence")
    audio_publisher = callback_args

    audioType = create_audio_from_lilypond(data)
    audio_publisher.publish(audioType)


def listener():
    rospy.init_node('visualization_node')

    # TODO: create audio from lilypond publisher
    pub = rospy.Publisher('audio_output_of_visualization', ROSImage, queue_size=10)
    rospy.Subscriber("vision_node/recognized_sentence", String, callback_vision_results, callback_args=(pub))

    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()

if __name__ == '__main__':
    listener()
