import os
from nvidia import cudnn
os.environ['LD_LIBRARY_PATH'] = f'{os.path.dirname(cudnn.__file__)}/lib:$LD_LIBRARY_PATH'
# since I haven't specified the version of TensorFlow, it seems to install the newest version, so according to official instruction, the environment needs to be configured.

import rospy
import whisper
from utils.file_control import WAVFile
from marimbabot_msgs.msg import Speech as SpeechMsg
from marimbabot_msgs.msg import TmpFile as TmpFileMsg
import time
from utils.command import get_commands

class STT:
	def __init__(self):
		# 'tiny.en', 'tiny', 'base.en', 'base', 'small.en', 'small', 'medium.en', 'medium', 'large-v1', 'large-v2', 'large'
		self.model = whisper.load_model('medium.en')
		self.file_controller = WAVFile()
		self.speech_pub = rospy.Publisher('/speech_node/speech', SpeechMsg, queue_size=100, tcp_nodelay=True)
		self.tmp_sub = rospy.Subscriber('/speech_node/audio_tmp', TmpFileMsg, self.tmp_callback, queue_size=10, tcp_nodelay=True)
		self.recognize_freq = 2  # Hz
		self.recognize_rate = rospy.Rate(self.recognize_freq)
		self.no_speech_prob_filter = 0.5
		#rospy.logdebug(f"cwd:{os.getcwd()}")
		self.prompt = self.generate_prompt()

	def generate_prompt(self):
		commands = get_commands()
		base_prompt = '''We do a demo of Marimbabot a marimba playing robot arm. Now some demonstrations. marimbabot read. marimbabot play.'''
		"""
		for command in commands:
			base_prompt += ''.join(f'{command.strip()}, ')
		"""
		rospy.logdebug(f"Generate prompt as: {base_prompt}.")
		return base_prompt
		


	def tmp_callback(self, tmp_file_msg):
		file_path = tmp_file_msg.file_path
		text, no_speech_prob = self.recognize(file_path)
		if no_speech_prob > self.no_speech_prob_filter:
			return
		if len(text)>300:
			return
		speech_msg = SpeechMsg()
		speech_msg.header.stamp = rospy.Time.now()
		speech_msg.speech = text
		speech_msg.is_finished = tmp_file_msg.is_finished
		self.speech_pub.publish(speech_msg)
		#rospy.logdebug(f"speech published.")


	def recognize(self, file_path:str):
		time_0 = time.time()
		# load audio and pad/trim it to fit 30 seconds
		audio = whisper.load_audio(file_path)
		audio = whisper.pad_or_trim(audio)

		# make log-Mel spectrogram and move to the same device as the model
		mel = whisper.log_mel_spectrogram(audio).to(self.model.device)

		# decode the audio
		options = whisper.DecodingOptions(fp16=True, language='en',prompt=self.prompt)
		time_1 = time.time()
		rospy.logdebug('*'*30)
		#rospy.logdebug(f"prerpocesing time:{time_1-time_0}")
		result = whisper.decode(self.model, mel, options)
		#rospy.logdebug(f"decoding time:{time.time()-time_1}")
		text = result.text
		no_speech_prob = result.no_speech_prob
		#rospy.logdebug(f"no_speech_prob: {no_speech_prob}")
		#rospy.logdebug(f" TEXT:{text}")
		return text, no_speech_prob

	def run(self):
		rospy.spin()

if __name__ == '__main__':
	rospy.init_node('speech_stt_node', log_level=rospy.DEBUG)
	speech_recognition = STT()
	speech_recognition.run()


	# speech_recognition.generate_prompt()
	# rospy.spin()
