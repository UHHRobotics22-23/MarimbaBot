import webrtcvad
import rospy
from audio_common_msgs.msg import AudioDataStamped
from file_control import WAVFile


class Audio2FileNode:
	def __init__(self):
		self.wav_file = WAVFile()
		self.audio_topic = '/audio_stamped'
		self.audio_sub = rospy.Subscriber(self.audio_topic, AudioDataStamped, self.audio_callback, queue_size=1)
		self.vad = webrtcvad.Vad(1)  # 0: min/off, 1: low, 2: mid, 3: high
		self.audio_files = WAVFile()
		self.sample_rate = self.audio_files.sample_rate
		self.silence_count = 0
		self.freq = 100  # Hz of ros pubiliching rate for tpoic /audio_stamped
		self.silence_limit = 3  # sec
		self.file_id = 0
		self.buffer_limit = 60  # sec
		self.buffer_count = 0

	def start_new_sentence(self):
		self.silence_count = 0
		self.buffer_count = 0
		self.file_id += 1

	def audio_callback(self, msg):
		audio_data = msg.audio.data
		# self.wav_file.save_audio(audio, self.file_id)
		self.buffer_count += 1
		# start new sentence if buffer is full
		if self.buffer_count > self.buffer_limit * self.freq:
			rospy.logdebug(f"buffer count: {self.buffer_count}, start new sentence")
			self.start_new_sentence()
		else:
			# start new sentence if silence is too long
			if self.vad_is_speech(audio_data):
				self.silence_count = 0
			else:
				self.silence_count += 1
				if self.silence_count > self.silence_limit * self.freq:
					rospy.logdebug(f"silence count: {self.silence_count}, start new sentence")
					self.start_new_sentence()

	def run(self):
		rospy.spin()

	def vad_is_speech(self, audio):
		return self.vad.is_speech(audio, self.sample_rate)

if __name__ == '__main__':
	rospy.init_node('audio2file',log_level=rospy.DEBUG)
	audio2file = Audio2FileNode()
	audio2file.run()