import os
from nvidia import cudnn

os.environ['LD_LIBRARY_PATH'] = f'{os.path.dirname(cudnn.__file__)}/lib:$LD_LIBRARY_PATH'
# since I haven't specified the version of TensorFlow, it seems to install the newest version,
# so according to official instruction, the environment needs to be configured.
from precise_runner import PreciseEngine, PreciseRunner, ReadWriteStream
from audio_common_msgs.msg import AudioDataStamped
from marimbabot_msgs.msg import Command as CommandMsg
from marimbabot_msgs.msg import Speech as SpeechMsg
from marimbabot_msgs.srv import SpeechRecognition,SpeechRecognitionResponse
import rospy
import whisper
import webrtcvad
import numpy as np
import struct
from utils.command import get_commands


# Speech to text recognition, it is a wrapper of whisper
class STT:
	def __init__(self, log_level=rospy.INFO):
		self.log_level = log_level
		self.prompt = self.generate_prompt()

		self.init_audio_config()
		self.init_whisper_model()
		self.warm_up()  # warm up the whisper model
		self.init_ros_node()


	def init_audio_config(self):
		self.sr = rospy.get_param('~sr', 16000)

	def init_whisper_model(self):
		# 'tiny.en', 'tiny', 'base.en', 'base', 'small.en', 'small', 'medium.en', 'medium', 'large-v1', 'large-v2', 'large'
		self.model = whisper.load_model('medium.en')

	def warm_up(self):
		# 30 denote sec, since whisper take fixed length of audio data as input
		self.recognize(np_data=np.zeros(self.sr*30, dtype=np.float32),pub_speech=False)
		rospy.loginfo("Whisper model is warmed up!")

	def init_ros_node(self):
		rospy.Service('speech_recognition', SpeechRecognition, self.handle_speech2text_service)
		# To publish the recognized text
		self.speech_pub = rospy.Publisher(
			'speech',
			SpeechMsg,
			queue_size=100,
			tcp_nodelay=True)

	def handle_speech2text_service(self, req):
		if self.log_level == rospy.DEBUG:
			rospy.logdebug(f"STT service triggered.")
		text, no_speech_prob = self.recognize(np_data=self.unpack_stream(req.audio.data))
		return SpeechRecognitionResponse(text=text, no_speech_prob=no_speech_prob)

	# To address the way how to unpack the data for whisper
	def unpack_stream(self, data):
		return np.array(struct.unpack(f"{int(len(data) / 2)}h", bytes(data)), dtype=float) / 526

	def generate_prompt(self):
		# commands = get_commands()
		# base_prompt = '''
		# 	Marimbabot is a marimba playing robot arm. You are able to give it commands, if you confuse just give it "None". The possible commands include:
		# '''
		# for command in commands:
		# 	base_prompt += ''.join(f'{command.strip()}, ')
		# rospy.logdebug(f"Generate prompt as: {base_prompt}.")
		# return base_prompt
		return "Marimbabot is a instrument playing robot arm. You are able to give it several common robot's commands"

	def run(self):
		rospy.spin()

	def recognize(self, file_path=None, np_data=None, pub_speech=True):
		time_0 = rospy.Time.now()
		# load audio and pad/trim it to fit 30 seconds
		if file_path is not None:
			audio = whisper.load_audio(file_path)
		elif np_data is not None:
			audio = np_data.astype(np.float32)
		else:
			rospy.logerr("No audio data is provided! Either file_path or np_data should be provided!")
			return

		audio = whisper.pad_or_trim(audio)  # each time feed 16000(sample rate)*30(sec) data samples to whisper model
		# make log-Mel spectrogram and move to the same device as the model
		mel = whisper.log_mel_spectrogram(audio).to(self.model.device)

		# decode the audio
		options = whisper.DecodingOptions(fp16=True, language='en')
		time_1 = rospy.Time.now()
		result = whisper.decode(self.model, mel, options)
		# options = whisper.DecodingOptions(fp16=True, language='en',prompt=self.prompt)
		time_2 = rospy.Time.now()
		text = result.text
		no_speech_prob = result.no_speech_prob
		if self.log_level == rospy.DEBUG:
			rospy.logdebug('*' * 30)
			rospy.logdebug(f"Pre-processing time:{(time_1 - time_0).to_sec():.4f}")
			rospy.logdebug(f"Prediction time:{(time_2 - time_1).to_sec():.4f}")
			rospy.logdebug(f"Result: [{text}]")
			rospy.logdebug(f"No_speech_prob: {no_speech_prob:.4f}")
		if pub_speech:
			# publish the recognized text
			self.speech_pub.publish(
				SpeechMsg(
					header=rospy.Time.now(),
					text=text,
					no_speech_prob=no_speech_prob)
			)
		return text, no_speech_prob

if __name__ == '__main__':
	# TODO: Finish the command extraction
	# TODO: reset the way to load precise-engine
	# TODO: consider the voice from other group members, i.e. change the dataset and retrain the model
	log_level = rospy.DEBUG
	rospy.init_node('speech_recognition_node', log_level=log_level)
	sst = STT(log_level=log_level)
	sst.run()

