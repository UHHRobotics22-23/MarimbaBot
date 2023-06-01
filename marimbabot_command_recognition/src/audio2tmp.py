import webrtcvad
import rospy
from audio_common_msgs.msg import AudioDataStamped
from marimbabot_command_recognition.msg import TmpFile as TmpFileMsg

from file_control import WAVFile

class Audio2FileNode:
	def __init__(self):
		self.wav_file = WAVFile()
		self.wav_file.remove_all_cache()

		self.audio_sub = rospy.Subscriber(
			'/audio_stamped',
			AudioDataStamped,
			self.audio_callback,
			queue_size=100,
			tcp_nodelay=True
		)

		self.tmp_pub = rospy.Publisher(
			'/audio_tmp',
			TmpFileMsg,
			queue_size=10,
			tcp_nodelay=True
		)

		self.vad = webrtcvad.Vad(3)  # 0: min/off, 1: low, 2: mid, 3: high
		self.audio_files = WAVFile()
		self.sample_rate = self.audio_files.sample_rate
		self.silence_count = 0
		self.freq = 100  # Hz of ros pubiliching rate for tpoic /audio_stamped
		self.silence_limit = 1  # sec
		self.file_id = 0
		self.buffer_limit = 60  # sec
		self.buffer_count = 0
		self.buffer = b''

	def reset(self):
		self.silence_count = 0
		self.buffer_count = 0
		self.buffer = b''

	def save_and_start_new_sentence(self):
		if len(self.buffer) > 0:
			rospy.logdebug(f"save audio file {self.file_id}")
			self.wav_file.save_audio(self.buffer, self.file_id)
			tmp_file_msg = TmpFileMsg()
			tmp_file_msg.sentence_id = self.file_id
			tmp_file_msg.file_path = self.wav_file.file_id2path(self.file_id)
			self.tmp_pub.publish(tmp_file_msg)
			self.file_id += 1
		self.reset()

	def audio_callback(self, msg):
		audio_data = msg.audio.data
		# self.wav_file.save_audio(audio_data, self.file_id)
		self.buffer_count += 1
		# start new sentence if buffer is full
		if self.buffer_count > self.buffer_limit * self.freq:
			rospy.logdebug(f"buffer full, save and start new sentence")
			self.save_and_start_new_sentence()
		else:
			# start new sentence if silence is too long
			if self.vad_is_speech(audio_data):
				self.buffer += audio_data
				self.silence_count = 0
			else:
				self.silence_count += 1
				# if len(self.buffer) > 0:
				# 	self.buffer += audio_data
				if self.silence_count > self.silence_limit * self.freq:
					self.save_and_start_new_sentence()

	def run(self):
		rospy.spin()

	def vad_is_speech(self, audio):
		return self.vad.is_speech(audio, self.sample_rate)

if __name__ == '__main__':
	rospy.init_node('audio2file', log_level=rospy.DEBUG)
	audio2file = Audio2FileNode()
	rospy.logdebug(f"audio2file node started")
	audio2file.run()
