import webrtcvad
import rospy
import os
import wave
from audio_common_msgs.msg import AudioDataStamped

class WAVFile:
	def __init__(self):
		self.tmp_folder = "/tmp/"
		self.sample_rate = 16000
		self.channels = 1
		self.sample_width = 2
		self.cache_size_limit = 20

	def file_id2path(self, file_id):
		return os.path.join(self.tmp_folder, f"tmp_audio_file_{file_id:04d}.wav")

	def path2file_id(self, path):
		return int(path.split("_")[-1].split(".")[0])

	# save audio to file
	def save_audio(self, audio, file_id):
		path = self.file_id2path(file_id)
		self.write_audio(path, audio)
		return path

	# write audio with wave module
	def write_audio(self, path, audio):
		self.remove_cache()
		with wave.open(path, "wb") as f:
			f.setnchannels(self.channels)
			f.setsampwidth(self.sample_width)
			f.setframerate(self.sample_rate)
			f.writeframes(audio)
			f.close()

	# read audio with wave module
	def read_audio(self, path):
		with wave.open(path, "rb") as f:
			assert f.getframerate() == self.sample_rate
			assert f.getnchannels() == self.channels
			assert f.getsampwidth() == self.sample_width
			frames = f.readframes(f.getnframes())
			f.close()
			return frames

	def remove_cache(self):
		# remove old cache
		tmp_files = []
		ids = []
		for file in os.listdir(self.tmp_folder):
			if 'tmp_audio_file_' in file:
				tmp_files.append(os.path.join(self.tmp_folder, file))
				ids.append(self.path2file_id(file))
		ids.sort()
		if len(ids) > self.cache_size_limit:
			for i in range(len(ids) - self.cache_size_limit):
				os.remove(tmp_files[i])

class Audio2FileNode:
	def __init__(self):
		self.audio_topic = '/audio_stamped'
		self.audio_sub = rospy.Subscriber(self.audio_topic, AudioDataStamped, self.audio_callback, queue_size=1)
		self.vad = webrtcvad.Vad(1)  # 0: min/off, 1: low, 2: mid, 3: high
		self.audio_files = WAVFile()
		self.sample_rate = self.audio_files.sample_rate
		self.silence_count = 0
		self.freq = 10  # Hz of ros pubiliching rate for tpoic /audio_stamped

	def audio_callback(self, msg):
		audio = msg.data
		file_id = msg.header.seq
		wav_file = WAVFile()
		path = wav_file.save_audio(audio, file_id)
		print(f"saved audio to {path}")

	def run(self):
		rospy.spin()

	def vad_is_speech(self, audio):
		return self.vad.is_speech(audio, self.sample_rate)