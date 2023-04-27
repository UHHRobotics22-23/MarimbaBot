#!/usr/bin/env python3

import rospy
from std_msgs.msg import String
import os
import subprocess

class SpeechSynthesis:
    def __init__(self):
        # initialize voice and speed
        self.voice = "en_UK/apope_low"
        self.speed = "1.0"

        # listens to the defined text responses from behavior node
        self.response_sub = rospy.Subscriber("behavior_node/response", String, self.callback_response)

    def callback_response(self, response):
        text = response.data

        mimic_subprocess = subprocess.Popen(
            ('mimic3', '--voice', self.voice, '--length-scale', self.speed, text), 
            stdout=subprocess.PIPE)

        # Play the audio from the previous process with aplay
        aplay_subprocess = subprocess.Popen(
            ('aplay', '-'), 
            stdin=mimic_subprocess.stdout, 
            stdout=subprocess.PIPE)
        
        # Wait for the process to finish
        aplay_subprocess.wait()

if __name__ == '__main__':
    rospy.init_node('speech_synthesis_node')

    SpeechSynthesis()

    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()