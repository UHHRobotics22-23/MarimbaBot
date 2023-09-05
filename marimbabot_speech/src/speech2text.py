import struct
import rospy
import whisper
import numpy as np
from marimbabot_msgs.msg import Speech as SpeechMsg
from audio_common_msgs.msg import AudioData

# Speech to text recognition, it is a wrapper of whisper
class STT:
	def __init__(self):
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
		# input blank data to warmup the whisper model.
		# 30 denote sec, since whisper take fixed length of audio data as input
		self.recognize(np_data=np.zeros(self.sr*30, dtype=np.float32),pub_speech=False)
		rospy.loginfo("Whisper model is warmed up!")

	def init_ros_node(self):
		self.buffer_sub = rospy.Subscriber(
			'/speech_node/speech_buffer',
			AudioData,
			self.handle_audio_buffer,
			queue_size=100,
			tcp_nodelay=True)

		# To publish the recognized text
		self.speech_pub = rospy.Publisher(
			'speech',
			SpeechMsg,
			queue_size=100,
			tcp_nodelay=True)

	def handle_audio_buffer(self, req):
		text, no_speech_prob = self.recognize(np_data=self.unpack_stream(req.data))
		msg = SpeechMsg()
		msg.header.stamp = rospy.Time.now()
		msg.text = text
		msg.no_speech_prob = no_speech_prob
		self.speech_pub.publish(msg)

	# To address the way how to unpack the data for whisper
	def unpack_stream(self, data):
		# return np.array(struct.unpack(f"{int(len(data) / 2)}h", bytes(data)), dtype=float) / 526
		return np.frombuffer(data, dtype=np.float64)/526


	def generate_prompt(self):
		# commands = get_commands()
		# base_prompt = '''Marimbabot is a marimba playing robot arm. You are able to give it commands, if you confuse just give it "None". The possible commands include:'''
		# for command in commands:
		# 	base_prompt += ''.join(f'{command.strip()}, ')
		# rospy.logdebug(f"Generate prompt as: {base_prompt}.")
		# return base_prompt
		prompt = "Marimbabot is a instrument playing robot arm. You are able to give it several common robot's commands." \
		         "play in 60 BPM, play louder by 40%, ..."
		return prompt

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
		options = whisper.DecodingOptions(fp16=True, language='en', prompt=self.generate_prompt())
		time_1 = rospy.Time.now()
		result = whisper.decode(self.model, mel, options)
		# options = whisper.DecodingOptions(fp16=True, language='en',prompt=self.prompt)
		time_2 = rospy.Time.now()
		text = result.text
		no_speech_prob = result.no_speech_prob
		rospy.logdebug('*' * 30)
		rospy.logdebug(f"Pre-processing time:{(time_1 - time_0).to_sec():.4f}")
		rospy.logdebug(f"Prediction time:{(time_2 - time_1).to_sec():.4f}")
		rospy.logdebug(f"Result: [{text}]")
		rospy.logdebug(f"No_speech_prob: {no_speech_prob:.4f}")
		return text, no_speech_prob

if __name__ == '__main__':
	rospy.init_node('speech_recognition_node', log_level=rospy.DEBUG)
	sst = STT()
	sst.run()

