from gtts import gTTS
import rospy
from marimbabot_command_recognition.srv import Speak as SpeakSrv, SpeakResponse
from playsound import playsound

# use the gTTS package to generate a mp3 file, it has better quality than the espeak
# use the service to call the function, because the function is blocking
class TTS():
	def __init__(self):
		self.tmp_file = "/tmp/stt.mp3"
		self.init_ros_node()

	def init_ros_node(self):
		rospy.Service(
			"/speech_node/tts",
			SpeakSrv,
			self.speak_srv_callback)

	def speak_srv_callback(self, req):
		try:
			text = req.text
			tts = gTTS(text)
			tts.save(self.tmp_file)
			playsound(self.tmp_file)
			return True
		except Exception as e:
			rospy.logerr(e)
			return False

if __name__ == '__main__':
	rospy.init_node('text2speech')
	tts = TTS()
	rospy.spin()