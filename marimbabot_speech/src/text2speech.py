#!/usr/bin/env python3

import subprocess
import tempfile

import rospy
from playsound import playsound
from std_msgs.msg import String


class SpeechSynthesis:
    def __init__(self):
        # initialize voice and speed
        self.voice = "en_UK/apope_low"
        self.speed = "1.5"

        # listens to the defined text responses from behavior node
        self.response_sub = rospy.Subscriber("behavior_node/response", String, self.callback_response)

    def callback_response(self, response):
        text = response.data
        
        if text == '':
            return
        else:
            temp_ = tempfile.TemporaryFile()
            audio_filename = str(temp_.name) + ".wav"

            with open(audio_filename, "w+") as f:
                mimic_subprocess = subprocess.Popen(
                    ('mimic3', '--voice', self.voice, '--length-scale', self.speed, text), 
                    # stdout=subprocess.PIPE)
                    stdout=f)

                # Wait for the process to finish
                mimic_subprocess.wait()

            playsound(audio_filename)

if __name__ == '__main__':
    rospy.init_node('speech_tts_node')

    SpeechSynthesis()

    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()