#!/usr/bin/env python

import rospy
from std_msgs.msg import String

class ActionDecider:
    def __init__(self):
        # the currently active note sequence
        # will be updated with the latest sentence from vision_node/recognized_sentence after the 'read' command was issued
        self.sentence = None

        # publishes the latest recognized sentence after the 'read' command was issued
        self.read_pub = rospy.Publisher('behavior_node/read_sentence', String, queue_size=10) 
        
        # publishes the read sentence after the 'play' command was issued
        self.play_pub = rospy.Publisher('behavior_node/play_sentence', String, queue_size=10) 

        # publishes a response for the synthesized speech
        self.response_pub = rospy.Publisher('behavior_node/response', String, queue_size=10) 

        # listens to the recognized commands from voice_recognition_node
        self.command_sub = rospy.Subscriber('voice_recognition_node/recognized_command', String, self.callback_command)

    def callback_command(self, command):

        if command.data == 'read':
            # update the sentence variable with the latest sentence from vision_node/recognized_sentence to signal that notes have been read
            self.sentence = rospy.wait_for_message('vision_node/recognized_sentence', String).data # wait for the vision node to publish a recognized sentence
            self.read_pub.publish(self.sentence)
            self.response_pub.publish('Notes recognzed. Say play to play the notes.')

        elif command.data == 'play':
            # if a sentence has been read via the 'read' command, publish it to behavior_node/play_sentence to signal that notes should be played
            if self.sentence:
                self.play_pub.publish(self.sentence)
                # TODO: include tempo (for future commands: faster, slower)
            else:
                self.response_pub.publish('No notes to play. Say read to read notes.')

        elif command.data == 'stop':
            # publish an empty string to behavior_node/play_sentence to signal aborting the playing of notes
            self.play_pub.publish('')

        # TODO: add more cases (loop, repeat, faster, slower, save as <name_of_piece>, play <name_of_piece>, ...)

        else:
            self.response_pub.publish('Command not recognized.')

if __name__ == '__main__':

    rospy.init_node('behavior_node')

    ActionDecider()

    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()