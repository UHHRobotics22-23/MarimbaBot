from precise_runner import PreciseEngine, PreciseRunner, ReadWriteStream
import rospy
import webrtcvad
from playsound import playsound
from audio_common_msgs.msg import AudioDataStamped, AudioData


class SpeechExtraction:
	"""
		SpeechExtraction is a class to detect the keyword from the audio stream.
		It is based on the precise-engine, which is a RNN-based lightweight wake word listener.

		Pipeline:
		1. First, use keyword spotting to detect the keyword "Hi/hello marimbabot" from the audio stream.
		2. When keyword is spotted, then the audio stream is first processed by the VAD (Voice Activity Detection)
		to detect the activity of human speech, if there is 1 second of silence or reach maximum length,
		then this audio stream will be considered to be a complete sentence, and then sent it to the STT node (Speech to Text).
		3. The STT will return the text of the speech sentence in form of text, which is then sent to the text2command node
		and also published to the speech topic.
		4. The text2command node will extract the command from the text through some regular expression, the command will
		be published it to the command topic.
	"""

	def __init__(self,
	             sensitivity=0.5,
	             engine_path=None,
	             model_path=None,
	             remind_sound_path=None,
	             trigger_level=5):
		self.VAD_ON = False
		self.KWS_ON = True
		self.remind_sound_path = remind_sound_path
		self.stream = None
		self.init_audio()
		self.init_kws(engine_path, model_path,sensitivity,trigger_level)
		self.init_wad()
		self.init_ros()

	def init_ros(self):
		self.speech_buffer_pub = rospy.Publisher('/speech_node/speech_buffer', AudioData, queue_size=500)
		self.audio_sub = rospy.Subscriber('/speech_node/audio_stamped', AudioDataStamped, self.audio_stream_callback,
		                                  queue_size=500, tcp_nodelay=True)

	def init_audio(self):
		self.buffer = b''  # buffer for audio data
		self.sr = 16000  # sample rate

	def init_kws(self, engine_path, model_path,sensitivity=0.5,trigger_level=5):
		# use ReadWriteStream to forward the audio stream from ros to precise-engine
		self.stream = ReadWriteStream()  # 16000 Hz 1 channel int16 audio
		# the engine is the binary file to load
		self.engine = PreciseEngine(engine_path, model_path)
		self.runner = PreciseRunner(
			# Object containing info on the binary engine
			engine=self.engine,
			# From 0.0 to 1.0, how sensitive the network should be
			sensitivity=sensitivity,
			# Number of chunk activations needed to trigger on_activation
			trigger_level=trigger_level,
			# callback for when the wake word is heard
			on_activation=self.on_activation,
			# callback for every new prediction
			on_prediction=self.on_prediction,
			# Binary audio stream to read 16000 Hz 1 channel int16 audio from. If not given, the microphone is used
			stream=self.stream)



	# init the human voice activity detection
	def init_wad(self):
		self.sentence_start_t = None
		self.silence_t = float(rospy.get_param('~wad_silence_t', 1))
		self.silence_start_t = None
		self.max_sentence_t = float(rospy.get_param('~wad_max_t', 20))
		self.vad = webrtcvad.Vad(3)  # 0: highest, 1: high, 2: middle, 3: low (default)

	def reset(self):
		self.buffer = b''
		self.silence_start_t = None
		self.sentence_start_t = None
		self.VAD_ON = False
		self.KWS_ON = True

	def start(self):
		self.runner.start()

	# triggered for keyword activation, condition refer to the trigger_level
	def on_activation(self):
		rospy.logdebug("Keyword spotted!")
		playsound(self.remind_sound_path)
		rospy.sleep(0.1)
		self.VAD_ON = True
		self.KWS_ON = False

	# triggered for each prediction
	def on_prediction(self, prob):
		rospy.loginfo_once("Keyword Spotting is started, you can speak after whisper model is on.")

	def stop(self):
		self.runner.stop()

	def __del__(self):
		self.runner.stop()
		self.runner = None
		self.engine = None

	def audio_stream_callback(self, msg):

		if self.VAD_ON:
			self.buffer += msg.audio.data
			# If the listening is longer than the max_sentence_t(10 sec as default),
			# then trigger the speech recognition.
			# It could cut off the speaking in the middle, but it is better than waiting for a long time,
			# since VAD is kind of sensitive.
			if self.sentence_start_t is None:
				self.sentence_start_t = msg.header.stamp
			else:
				if msg.header.stamp - self.sentence_start_t > rospy.Duration.from_sec(self.max_sentence_t):
					self.speech_buffer_pub.publish(AudioData(data=self.buffer))
					self.reset()
					return

			# use the VAD to detect the silence, if the silence is longer than the silence_t, then trigger the speech recognition
			if self.vad.is_speech(msg.audio.data, self.sr):
				self.silence_start_t = msg.header.stamp
			# if the silence is longer than the silence_t, then trigger the speech recognition
			elif self.silence_start_t is not None and \
					msg.header.stamp - self.silence_start_t > rospy.Duration.from_sec(self.silence_t):
				self.speech_buffer_pub.publish(AudioData(data=self.buffer))
				self.reset()
				return

		if self.KWS_ON and self.stream is not None:
			# froward th stream from ros to precise-engine
			self.stream.write(msg.audio.data)


if __name__ == '__main__':
	rospy.init_node('keyword_spotting_node', anonymous=True, log_level=rospy.DEBUG)
	precise_engine_path = rospy.get_param(
		"~precise_engine_path",
		default="/home/wang/workspace/marimbabot_ws/kws_ws/src/marimbabot/marimbabot_speech/lib/precise-engine/precise-engine")
	model_path = rospy.get_param(
		'~model_path',
	    default="/home/wang/workspace/marimbabot_ws/kws_ws/src/marimbabot/marimbabot_speech/config/kws/hi-marimbabot.pb")  # model of KWS
	sensitivity = rospy.get_param('~sensitivity', 0.5)
	trigger_level = rospy.get_param('~trigger_level', 5)
	remind_sound_path = rospy.get_param(
		'~remind_sound_path',
		default="/home/wang/workspace/marimbabot_ws/kws_ws/src/marimbabot/marimbabot_speech/config/kws/reminder.wav"
	)  # remind sound for when KWS is activated
	keyword_spotting = SpeechExtraction(
		engine_path=precise_engine_path,
		model_path=model_path,
		remind_sound_path=remind_sound_path,
		sensitivity=sensitivity,
		trigger_level=trigger_level
	)
	keyword_spotting.start()
	rospy.spin()
