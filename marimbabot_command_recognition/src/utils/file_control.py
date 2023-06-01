import wave
import os

class WAVFile:
	def __init__(self):
		self.tmp_folder = "/tmp/"
		self.sample_rate = 16000
		self.channels = 1
		self.sample_width = 2
		self.cache_size_limit = 10

	# get path from file id
	def file_id2path(self, file_id):
		return os.path.join(self.tmp_folder, f"tmp_audio_file_{file_id:04d}.wav")

	# get file id from path
	def path2file_id(self, path):
		return int(path.split("_")[-1].split(".")[0])

	# get all ids from tmp folder
	def get_all_ids(self):
		ids = []
		for file in os.listdir(self.tmp_folder):
			if 'tmp_audio_file_' in file:
				ids.append(self.path2file_id(file))
		ids.sort()
		return ids

	# get last file id
	def get_last_file_id(self):
		ids = self.get_all_ids()
		if len(ids) > 0:
			return ids[-1]
		else:
			return None

	# save audio to file
	def save_audio(self, audio, file_id):
		path = self.file_id2path(file_id)
		self.write_audio(path, audio)
		return path

	# write audio with wave module
	def write_audio(self, path, audio):
		self.clean_cache()
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

	def clean_cache(self):
		# remove old cache when cache size is over limit
		ids = self.get_all_ids()
		if len(ids) > self.cache_size_limit:
			for i in range(len(ids) - self.cache_size_limit):
				os.remove(self.file_id2path(ids[i]))

	# remove all files from tmp folder
	def remove_all_cache(self):
		for file in os.listdir(self.tmp_folder):
			if 'tmp_audio_file_' in file:
				os.remove(os.path.join(self.tmp_folder, file))
