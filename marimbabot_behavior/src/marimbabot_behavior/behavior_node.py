#!/usr/bin/env python3

import rospy
import actionlib
from std_msgs.msg import String
from marimbabot_behavior.interpreter import read_notes
from abjad.exceptions import LilyPondParserError
from marimbabot_msgs.msg import HitSequenceAction
from marimbabot_msgs.msg import Command

class ActionDecider:
    def __init__(self):
        # the currently active note sequence
        # will be updated with the latest sentence from vision_node/recognized_sentence after the 'read' command was issued
        self.sentence = None

        # the currently active hit sequence
        # will be updated together with sentence after the 'read' command was issued
        self.hit_sequence = None

        # publishes a response for the synthesized speech
        self.response_pub = rospy.Publisher('~response', String, queue_size=10) 

        # listens to the recognized commands from speech node
        self.command_sub = rospy.Subscriber('speech_node/command', Command, self.callback_command)

        # action client to send the hit sequence to the planning action server
        self.client = actionlib.SimpleActionClient('hit_sequence', HitSequenceAction)

    def callback_command(self, command_msg):
        command = command_msg.command
        rospy.loginfo(f"received command: {command}")

        if command == 'Marimbabot read':
            rospy.loginfo('reading notes')
            # update the sentence variable with the latest sentence from vision_node/recognized_sentence to signal that notes have been read
            try:
                # wait for the vision node to publish a recognized sentence
                self.sentence = rospy.wait_for_message('vision_node/recognized_notes', String, timeout=5).data 
                rospy.loginfo(f"recognized notes: {self.sentence}")
                self.hit_sequence = read_notes(self.sentence)
                self.response_pub.publish('Notes recognized. Say play to play the notes.')
            except rospy.ROSException:
                rospy.logwarn('No notes recognized. Make sure the notes are readable and visible to the camera.')
                self.response_pub.publish('No notes recognized. Make sure the notes are readable and visible to the camera.')
            except LilyPondParserError:
                rospy.logwarn('Lilypond string not valid. Make sure the note are valid and readable.')
                self.response_pub.publish('Lilypond string not valid. Make sure the note are valid and readable.')

        elif command == 'Marimbabot play':
            # if a note sequence has been read via the 'read' command and the corresponding hit sequence is valid, the hit sequence is send to the planning action server
            if self.hit_sequence:
                # check if action server is busy
                if not self.client.gh:
                    rospy.loginfo(f"playing notes: {self.sentence}")
                    rospy.loginfo(f"goal_hit_sequence: {self.hit_sequence}")
                    self.client.send_goal(self.hit_sequence)
                else:    
                    rospy.logwarn('Action server is busy. Try again later.')
                    self.response_pub.publish('Action server is busy. Try again later.')
            else:
                rospy.logwarn('No notes to play. Say reed to read notes.')
                self.response_pub.publish('No notes to play. Say reed to read notes.')

        # TODO: handle ROS exceptions from the planning side (e.g. planning failed, execution failed, ...)

        # TODO: elif command == 'stop':
        
        # TODO: add more cases (loop, repeat, faster, slower, save as <name_of_piece>, play <name_of_piece>, ...)

        else:
            self.response_pub.publish('Command not recognized.')

if __name__ == '__main__':

    rospy.init_node('behavior_node')

    ActionDecider()

    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()