#!/usr/bin/env python3

import rospy
from sound_play.msg import SoundRequest
from std_msgs.msg import String

class SpeechSynthesis:
    def __init__(self):
        # initialize voice and speed
        self.voice = "en_UK/apope_low"
        self.speed = "1.0"

        # listens to the defined text responses from behavior node
        self.response_sub = rospy.Subscriber("behavior_node/response", String, self.callback_response)
        self.audio_pub = rospy.Publisher('robotsound', SoundRequest, queue_size=1)

    def callback_response(self, response):
        text = response.data

        # generate sound request message
        # http://docs.ros.org/en/api/sound_play/html/msg/SoundRequest.html
        sound_request = SoundRequest()
        
        if text == '':
            sound_request.command = 0 
            sound_request.sound = -1

        else:
            sound_request.command = 1
            sound_request.volume = 1

            # speak from text 
            sound_request.sound = -3
            sound_request.arg = text

        # publish audio file to audio node (sound_play package)
        self.audio_pub.publish(sound_request)

if __name__ == '__main__':
    rospy.init_node('speech_stt_node')

    SpeechSynthesis()

    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()