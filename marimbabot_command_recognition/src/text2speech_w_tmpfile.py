from gtts import gTTS
# import vlc
from std_msgs.msg import String
from sound_play.msg import SoundRequest
import rospy
from marimbabot_command_recognition.msg import Speaking as SpeakingMsg
from marimbabot_command_recognition.msg import SpeakingEnd as SpeakingEndMsg


class TTS():
	def __init__(self):
		self.tmp_file = "/tmp/stt.mp3"

	def init_ros_node(self):
		self.sub = rospy.Subscriber(
			"/speech_node/tts",
			String,
			self.callback)

		self.audio_pub = rospy.Publisher(
			'robotsound',
			SoundRequest,
			queue_size=10,
			tcp_nodelay=True)

		self.speaking_end_pub = rospy.Publisher(
			"/speech_node/tts_end",
			SpeakingEndMsg,
			queue_size=10,
			tcp_nodelay=True)

	def callback(self,msg):
		id = msg.data.speaking_id
		text = msg.data.text
		# generate sound request message
		if text is not '':
			if len(text) >= 100:
				rospy.logwarn("Text is too long for TTS. Max 100 characters. Text: {}".format(text))
				return
			tts = gTTS(text)
			tts.save(self.tmp_file)
			# play audio file
			sound_request = SoundRequest()
			sound_request.command = 1  # 0: Stop this sound from playing, 1: play once 2: Play the sound in loop until stop
			sound_request.volume = 1  # Volume of the sound. 0 as mute and 1.0 as 100%.
			sound_request.sound = -3  # Selects which sound to play (see above) 1-5, see SoundRequest.msg for more details
			sound_request.arg = self.tmp_file
			self.audio_pub.publish(sound_request)
			