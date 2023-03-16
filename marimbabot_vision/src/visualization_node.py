#!/usr/bin/env python

import tempfile

import cv2
import numpy as np
import rospy
from abjad import Block, LilyPondFile
from abjad.persist import as_png
from cv_bridge import CvBridge
from sensor_msgs.msg import Image as ROSImage
from std_msgs.msg import String


def callbackVisionResults(sentence, args):
    pub = args

    lilypond_string = f"\\relative {{{sentence}}}"
    header_block = Block(name="header")
    header_block.tagline = "#ff"
    lilypond_file = LilyPondFile(
        items=[
            header_block,
            """#(set-default-paper-size "a8" 'landscape)""",
            "#(set-global-staff-size 16)",
            lilypond_string,
        ]
    )
    # store lilypond as png temporarily
    temp_ = tempfile.TemporaryFile()
    as_png(lilypond_file, temp_.name, resolution=200)

    # read as PIL and convert to ROSImage
    cv_image = cv2.imread(temp_.name)
    bridge = CvBridge()
    image_message = bridge.cv2_to_imgmsg(cv_image, encoding="passthrough")

    pub.publish(image_message)

def listener():
    rospy.init_node('visualization_node')

    pub = rospy.Publisher('detection_visualization', ROSImage)
    rospy.Subscriber("vision_node/recognized_sentence", String, callbackVisionResults, callback_args=(pub))

    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()


if __name__ == '__main__':
    listener()
