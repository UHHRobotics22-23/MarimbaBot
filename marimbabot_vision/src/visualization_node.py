#!/usr/bin/env python

import os
import tempfile

import cv2
import rospy
from abjad import Block, LilyPondFile, Staff, Voice
from abjad.persist import as_png
from cv_bridge import CvBridge
from sensor_msgs.msg import Image as ROSImage
from std_msgs.msg import String


def callback_vision_results(sentence, args):
    rospy.logdebug(f"Received sentence: {sentence.data}")
    pub = args

    # generate abjad staff
    voice = Voice(sentence.data)
    staff = Staff([voice])

    # generate lilypond file
    header_block = Block(name="header")
    header_block.tagline = "#ff"
    lilypond_file = LilyPondFile(
        items=[
            header_block,
            """#(set-default-paper-size "a8" 'landscape)""",
            "#(set-global-staff-size 16)",
            staff,
        ]
    )
    # store lilypond as png temporarily
    temp_ = tempfile.TemporaryFile()
    as_png(lilypond_file, str(temp_.name), remove_ly=True, resolution=200)
    png_filename = str(temp_.name) + ".png"

    # read as PIL and convert to ROSImage
    cv_image = cv2.rotate(cv2.imread(png_filename, cv2.IMREAD_UNCHANGED), cv2.ROTATE_90_CLOCKWISE)
    bridge = CvBridge()
    image_message = bridge.cv2_to_imgmsg(cv_image, encoding="passthrough")

    pub.publish(image_message)
    # remove temporary file after publishing
    # this is needed because tempfile creates a file with a random name 
    # but abjad creates a new file adding the postfix .png/.ly
    os.remove(png_filename)

def listener():
    rospy.init_node('visualization_node')

    pub = rospy.Publisher('detection_visualization', ROSImage, queue_size=10)
    rospy.Subscriber("vision_node/recognized_sentence", String, callback_vision_results, callback_args=(pub))

    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()


if __name__ == '__main__':
    listener()
