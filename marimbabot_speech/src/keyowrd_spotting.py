import os
from nvidia import cudnn

os.environ['LD_LIBRARY_PATH'] = f'{os.path.dirname(cudnn.__file__)}/lib:$LD_LIBRARY_PATH'
# since I haven't specified the version of TensorFlow, it seems to install the newest version,
# so according to official instruction, the environment needs to be configured.
from precise_runner import PreciseEngine, PreciseRunner, ReadWriteStream
from audio_common_msgs.msg import AudioDataStamped,AudioData
from marimbabot_msgs.srv import SpeechRecognition,SpeechRecognitionResponse
from std_msgs.msg import Float32
import rospy
import webrtcvad
from playsound import playsound

VAD_ON = False
KWS_ON = True

class KeywordSpotting:
	"""
		Keyword spotting is a class to detect the keyword from the audio stream.
		It is based on the precise-engine, which is a RNN-based lightweight wake word listener.
	"""
	def __init__(self,
	             sensitivity=0.5,
	             log_level=rospy.DEBUG,
	             engine_path=None,
	             model_path=None,
	             silence_t=1,
	             remind_sound_path=None):
		self.log_level = log_level
		self.sensitivity = sensitivity
		self.remind_sound_path = remind_sound_path
		self.stream = None
		self.init_audio()
		try:
			rospy.wait_for_service('/speech_node/speech_recognition', timeout=60)  # STT must on before KWS
		except rospy.ROSException:
			rospy.logerr("STT service is not available!")
			raise rospy.ROSException("STT service is not available!")
		self.init_kws(engine_path, model_path)
		self.init_wad(silence_t=silence_t)
		self.warmup()

	def init_audio(self):
		self.buffer = b''  # buffer for audio data
		# subscribe to the audio stream from ros
		self.audio_sub = rospy.Subscriber('/speech_node/audio_stamped', AudioDataStamped, self.audio_stream_callback,
		                                  queue_size=500, tcp_nodelay=True)
		self.sr = 16000  # sample rate

	def init_kws(self, engine_path, model_path, sensitivity=0.5, trigger_level=5):
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
		if self.log_level == rospy.DEBUG:
			self.prob_pub = rospy.Publisher('/speech_node/keyword_spotting/prob', Float32, queue_size=20)

	# init the human voice activity detection
	def init_wad(self, silence_t):
		self.sentence_start_t = None
		self.silence_t = silence_t
		self.silence_start_t = None
		self.max_sentence_t = 10
		self.vad = webrtcvad.Vad(3)  # 0: highest, 1: high, 2: middle, 3: low (default)

	def reset(self):
		self.buffer = b''
		self.silence_start_t = None
		self.sentence_start_t = None
		global VAD_ON
		global KWS_ON
		VAD_ON = False
		KWS_ON = True

	def start(self):
		self.runner.start()

	# triggered for keyword activation, condition refer to the trigger_level
	def on_activation(self):
		global KWS_ON
		global VAD_ON
		if self.log_level == rospy.DEBUG:
			rospy.logdebug("Keyword Spotting is activated!!!")
		VAD_ON = True
		KWS_ON = False
		# TODO check if it works
		playsound(self.remind_sound_path)
		print("hi, I am here")

	def warmup(self):
		pass

	# triggered for each prediction
	def on_prediction(self, prob):
		rospy.loginfo_once("Keyword Spotting is started.")
		if prob > self.sensitivity:
			self.prob_pub.publish(prob)

	def stop(self):
		self.runner.stop()

	def __del__(self):
		self.runner.stop()
		self.runner = None
		self.engine = None

	def call_speech_recognition_service(self, buffer):
		try:
			rospy.wait_for_service('/speech_node/speech_recognition',timeout=4)
		except rospy.ROSException as e:
			rospy.logwarn("Speech Recognition Service is not on: %s" % e)
			return None
		audio_data = AudioData()
		audio_data.data = buffer  # the buffer is the audio data, same data type as the audio stream from topic /speech_node/audio_stamped
		try:
			speech_recognition = rospy.ServiceProxy('/speech_node/speech_recognition', SpeechRecognition)
			resp = speech_recognition(audio_data)
			return resp.text
		except rospy.ServiceException as e:
			print("Service call failed: %s" % e)

	def audio_stream_callback(self, msg):
		global KWS_ON
		global VAD_ON

		if VAD_ON:
			self.buffer += msg.audio.data
			# If the listening is longer than the max_sentence_t(10 sec as default),
			# then trigger the speech recognition.
			# It could cut off the speaking in the middle, but it is better than waiting for a long time,
			# since VAD is kind of sensitive.
			if self.sentence_start_t is None:
				self.sentence_start_t = msg.header.stamp
			else:
				if msg.header.stamp - self.sentence_start_t > rospy.Duration.from_sec(self.max_sentence_t):
					text = self.call_speech_recognition_service(self.buffer)
					# TODO call command exraction service
					self.reset()
					return

			# use the VAD to detect the silence, if the silence is longer than the silence_t, then trigger the speech recognition
			if self.vad.is_speech(msg.audio.data, self.sr):
				self.silence_start_t = msg.header.stamp
			# if the silence is longer than the silence_t, then trigger the speech recognition
			elif self.silence_start_t is not None and \
					msg.header.stamp - self.silence_start_t > rospy.Duration.from_sec(self.silence_t):
					text = self.call_speech_recognition_service(self.buffer)
					# TODO call command exraction service
					self.reset()
					return

		if KWS_ON and self.stream is not None:
			# froward th stream from ros to precise-engine
			self.stream.write(msg.audio.data)

if __name__ == '__main__':
	# TODO: reset the way to load precise-engine
	log_level = rospy.DEBUG
	rospy.init_node('keyword_spotting_node', anonymous=True, log_level=log_level)
	engine_path_default = "/home/wang/workspace/marimbabot_ws/env_this/bin/precise-engine"
	model_path_default = '/home/wang/workspace/marimbabot_ws/kws_ws/src/marimbabot/marimbabot_speech/hi-marimbabot.pb'
	remind_sound_path_default = "/home/wang/workspace/marimbabot_ws/kws_ws/src/marimbabot/marimbabot_speech/src/resource/kws/reminder.wav"

	# start the keyword spotting
	engine_path = rospy.get_param('~engine_path', default=engine_path_default)  # engine of KWS
	model_path = rospy.get_param('~model_path', default=model_path_default)  # model of KWS
	remind_sound_path = rospy.get_param('~remind_sound_path', default=remind_sound_path_default)  # remind sound for when KWS is activated
	keyword_spotting = KeywordSpotting(
		engine_path=engine_path,
		model_path=model_path,
		log_level=log_level,
		remind_sound_path=remind_sound_path,
	)
	keyword_spotting.start()
	rospy.spin()
