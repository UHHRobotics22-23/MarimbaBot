#!/usr/bin/env python

import tempfile

import numpy as np
import rospy
from abjad import Block, LilyPondFile
from abjad.persist import as_png
from PIL import Image as PILImage
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
    im = PILImage.open(temp_.name)
    im = im.convert('RGB')

    msg = ROSImage()
    msg.header.stamp = rospy.Time.now()
    msg.height = im.height
    msg.width = im.width
    msg.encoding = "rgb8"
    msg.is_bigendian = False
    msg.step = 3 * im.width
    msg.data = np.array(im).tobytes()

    pub.publish(msg)

def listener():
    rospy.init_node('visualization_node')

    pub = rospy.Publisher('detection_visualization')
    rospy.Subscriber("vision_node/recognized_sentence", String, callbackVisionResults, callback_args=(pub))

    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()


if __name__ == '__main__':
    listener()
