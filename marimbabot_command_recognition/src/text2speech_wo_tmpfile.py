import rospy
from sound_play.msg import SoundRequest
from marimbabot_command_recognition.msg import Speaking as SpeakingMsg

# know issues: can not give a feedback by end of speech, could influence the performance of speech recognition
class SpeechSynthesis:
    def __init__(self):
        # initialize voice and speed
        self.voice = "en_UK/apope_low"
        self.speed = "1.0"

        # listens to the defined text responses from other nodes
        self.response_sub = rospy.Subscriber("speech_node/tts",
                                             SpeakingMsg,
                                             self.callback_response,
                                             queue_size=10,
                                             tcp_nodelay=True)
        self.audio_pub = rospy.Publisher('robotsound',
                                         SoundRequest,
                                         queue_size=10,
                                         tcp_nodelay=True)

    def callback_response(self, msg):
        # id = msg.data.speaking_id
        text = msg.data.text
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
    def run(self):
        rospy.spin()

if __name__ == '__main__':
    rospy.init_node('speech_synthesis_node')
    sst = SpeechSynthesis()
    sst.run()