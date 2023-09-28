#!/usr/bin/env python

import os
import tempfile

import cv2
import rospy
from abjad import Block, LilyPondFile, Staff, Voice, Repeat, attach
from abjad.persist import as_png
from cv_bridge import CvBridge
from sensor_msgs.msg import Image as ROSImage
from std_msgs.msg import String

"""
Duplicate Method as in marimbabot_behavior/interpreter.

Creates a Staff from a list of notes
Also contains the logic for repeats. That is not taken care of by abjad.
"""
def create_staff_from_notes(notes):
    # check if file string includes repeat. If yes, remove it as it is added later
    set_repeat = False
    if "\\repeat volta" in notes:
        rospy.logdebug("Found a 'repeat volta' in the notes. Removing it as it is added manually.")
        notes = notes.replace("\\repeat volta 2", "")
        set_repeat = True

    voice_1 = Voice(notes, name="Voice_1")
    if set_repeat:
        repeat = Repeat()
        attach(repeat, voice_1)

    staff = Staff([voice_1], name="Staff_1")

    return staff

def callback_vision_results(notes, args):
    rospy.logdebug(f"Received notes: {notes.data}")
    pub = args

    # generate abjad staff
    staff = create_staff_from_notes(notes.data)

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

    pub = rospy.Publisher('~detection_visualization', ROSImage, queue_size=10)
    rospy.Subscriber("vision_node/recognized_notes", String, callback_vision_results, callback_args=(pub), queue_size=1)

    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()


if __name__ == '__main__':
    listener()
