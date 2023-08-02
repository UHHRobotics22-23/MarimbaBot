from functools import reduce
import json
import os
import struct
import cv_bridge
import rospy
from audio_common_msgs.msg import AudioDataStamped, AudioInfo
from sensor_msgs.msg import Image
from std_msgs.msg import Float32
from marimbabot_msgs.msg import NoteOnset, CQTStamped
import pretty_midi
import librosa
import crepe
import cv2
import numpy as np
import matplotlib.pyplot as plt
from matplotlib.colors import ListedColormap

def hz_to_note(hz):
	return pretty_midi.note_number_to_name(pretty_midi.hz_to_note_number(hz))

def note_to_pm_id(note_name):
	return pretty_midi.note_name_to_number(note_name)

def pm_id_to_note(pm_id):
	return pretty_midi.note_number_to_name(pm_id)

def note_to_hz(note):
	return pretty_midi.note_number_to_hz(pretty_midi.note_name_to_number(note))

def check_audio_format():
	"""
	check available audio input format, if it satisfy the requirements.
	:return:
	"""
	rospy.logdebug("Waiting for Audio Info")
	info = rospy.wait_for_message("/audio_node/audio_info", AudioInfo)
	if info.channels != 1:
		rospy.logfatal(
			"audio data has more than one channel,"
			"expecting single-channel recording"
		)
	elif info.sample_rate != 44100:
		rospy.logfatal(
			f"sample rate {info.sample_rate} is not 44100"
		)
	elif info.sample_format != "S16LE":
		rospy.logfatal(
			f"sample format '{info.sample_format}' is not S16LE"
		)
	elif info.coding_format != "wave":
		rospy.logfatal(
			f"coding '{info.coding_format}' is not raw"
		)
	else:
		rospy.logdebug("Audio compatible")
		return True
	return False

class OnsetDetection:
	def __init__(self):
		self.first_input = True
		# other parameters
		self.last_time = rospy.Time.now()
		self.last_seq_id = 0

		self.init_instrument_config()
		self.init_audio_config()
		self.init_detection_config()
		self.init_visualization_config()
		self.init_cache_file()

		if not check_audio_format():
			rospy.signal_shutdown("incompatible audio format")
			return
		# the buffer to read audio raw data from ros topic
		self.buffer = np.array([0.0] * self.sr, dtype=float)
		# warm up classifier / jit caches
		self.warm_up()

	def init_cache_file(self):
		self.onset_cache_file = "/tmp/onsets.json"
		if os.path.exists(self.onset_cache_file):
			os.remove(self.onset_cache_file)

	def init_instrument_config(self):
		# # harp
		# self.fmin_note = "C4"
		# self.fmax_note = "C6"
		# self.semitones = 62

		# # guzheng
		# self.fmin_note = "C2"
		# self.fmax_note = "C8"
		# self.semitones = 84

		# marimba
		self.fmin_note = "C4"  # C4 pm_id=60
		self.fmin_note_id = note_to_pm_id(self.fmin_note)
		self.fmax_note = "C7"  # C7 pm_id=96
		self.fmax_note_id = note_to_pm_id(self.fmax_note)
		self.semitones = 36 + 24  # 60   36 for 4-6 octives, 24 for overtones for two octives

		# convert the western notation to the corresponding frequency
		self.fmin = note_to_hz(self.fmin_note)
		self.fmax = note_to_hz(self.fmax_note)
		rospy.logdebug("Instrument configuration initialized.")

	def init_audio_config(self):
		"""
			The configuration of the audio signal
		"""
		self.sr = 44100
		self.hop_length = 512  # each hop equal to one pixel in spectrum
		self.pixels_per_sec = self.sr/self.hop_length  # careful, it is float for further precise calculation.
		rospy.logdebug(f"Audio configuration initialized.")

	def init_detection_config(self):
		"""
			confidence threshold for note classification(crepe)
		"""
		# For onset detection
		self.reference_amplitude = rospy.get_param("~reference_amplitude", np.max)
		self.confidence_threshold = 0.3  # the threshold for note classification
		self.windows_for_classification = 0.1  # using 0.1 sec data after onset time for note classification
		# preload model to not block the callback on first message
		# capacities: 'tiny', 'small', 'medium', 'large', 'full'
		self.crepe_model = "full"  # choose the crepe model type for music note classification
		rospy.logdebug(f"Loading crepe {self.crepe_model}-model...")
		crepe.core.build_and_load_model(self.crepe_model)
		rospy.logdebug(f"Crepe {self.crepe_model}-model loaded.")
		rospy.logdebug("Detection configuration initialized.")

	def init_visualization_config(self):
		"""
			config the spectrum plot
		"""
		hsv = plt.get_cmap("hsv")
		self.cmap = ListedColormap(np.vstack((
			hsv(np.linspace(0, 1, 86)),
			hsv(np.linspace(0, 1, 85)),
			hsv(np.linspace(0, 1, 85)))
		))
		self.cmap.set_bad((0, 0, 0, 1))  # make sure they are visible

		# time for analysis window
		self.window_t = 1.0
		self.window_overlap_t = 0.5  # and overlap regions between consecutive windows
		self.window_all_t = self.window_t + 2*self.window_overlap_t  # window in the middle, overlap windows at both end.

		# number of samples for analysis window
		self.window_num = int(self.sr * self.window_t)
		self.window_overlap_num = int(self.sr * self.window_overlap_t)

		# how much hop for overlap
		self.overlap_hops = int(self.window_overlap_num / self.hop_length)

		self.cv_bridge = cv_bridge.CvBridge()
		rospy.logdebug("Visualization configuration initialized.")

	def warm_up(self):
		# warm up classifier / jit caches
		_ = self.cqt()
		_ = self.onset_classification(0.0)
		self.reset()

	def start(self):
		# the spectrum for visualization
		self.pub_spectrogram = rospy.Publisher(
			"/audio/spectrogram_img", Image, queue_size=1, tcp_nodelay=True
		)
		self.pub_compute_time = rospy.Publisher(
			"/audio/compute_time", Float32, queue_size=1, tcp_nodelay=True
		)
		# signal after constant Q transform
		self.pub_cqt = rospy.Publisher(
			"/audio/cqt", CQTStamped, queue_size=100, tcp_nodelay=True
		)
		# onset signals
		self.pub_onset = rospy.Publisher(
			"/audio/onset_notes", NoteOnset, queue_size=10, tcp_nodelay=True
		)
		self.sub = rospy.Subscriber(
			"/audio_node/audio_stamped",
			AudioDataStamped,
			self.audio_process,
			queue_size=500,
			tcp_nodelay=True,
		)
		rospy.spin()

	def reset(self):
		# audio buffer
		self.buffer_time = None
		self.buffer = np.array([], dtype=float)

		# visualization
		self.spectrogram = None
		self.previous_onsets = []
		self.onsets_times = []
		self.previous_winner_onsets_hops = []
		self.previous_onsets_hops = []
		self.previous_winners = []
		self.previous_durations = []

	# the most important function for signal processing logic
	# 0.5+ sec delay for detection, detect for each 1 sec.
	def audio_process(self, msg):
		# unpack the msg
		msg_time = msg.header.stamp
		seq_id = msg.header.seq
		# the way to decode the data from ros topic
		msg_data = np.array(struct.unpack(f"{int(len(msg.audio.data) / 2)}h",bytes(msg.audio.data)),dtype=float)

		# handle bag loop graciously
		if msg_time < self.last_time:
			rospy.logdebug("detected bag loop")
			self.reset()
		# make sure the seq in order
		if seq_id > self.last_seq_id + 1 and not self.first_input:
			jump = seq_id - self.last_seq_id
			rospy.logwarn(
				f"sample drop detected: seq jumped "
				f"from {self.last_seq_id} to {seq_id} "
				f"(difference of {jump})"
			)
			end_of_buffer_time = self.buffer_time + rospy.Duration(self.buffer.shape[0] / self.sr)
			self.reset()
			self.buffer_time = end_of_buffer_time + rospy.Duration((jump - 1) * (len(msg_data) / self.sr))
		elif seq_id != self.last_seq_id + 1 and not self.first_input:
			rospy.logwarn(f"something weird happened, seq jumped from {self.last_seq_id} to {seq_id}")

		self.first_input = False
		self.last_time = msg_time
		self.last_seq_id = seq_id

		# take time from message headers and increment based on data
		if self.buffer_time is None:
			self.buffer_time = msg_time
		self.buffer = np.concatenate([
			self.buffer,
			msg_data
		])

		# make sure buffer is full, which is 1 sec new data and 1 sec old data. aka. 1 sec per update of cqt.
		# aggregate buffer until window+2*overlaps are full, like [0.5 Sec Overlap | 1 Sec window | 0.5 Sec Overlap]
		if self.buffer.shape[0] < self.window_num + 2 * self.window_overlap_num:
			return

		"""
			constant q transform with 60 half-tones from C4,
			in theory we only need notes from C4-C7, but in practice tuning
			is often too low and harmonics are needed above C6,
			therefore we use 60 semitones include 2 octaves overtone.
		"""
		cqt = self.cqt()  # cqt ndarrary  (60,173)
		self.publish_cqt(cqt)

		onset_env_cqt = librosa.onset.onset_strength(sr=self.sr, S=librosa.amplitude_to_db(cqt, ref=np.max)        )
		# detect when the onset happened within 2 sec cqt (60,173)
		onsets_cqt_time_list = librosa.onset.onset_detect(
			y=self.buffer,
			sr=self.sr,
			hop_length=self.hop_length,
			onset_envelope=onset_env_cqt,
			units="time",
			backtrack=False,
			delta=4.0,
			normalize=False,
		)

		# filter out the onset in the overlap windows, to keep the long tail inside
		# the whole windows include 0.5 sec overlap at both end, the target windows is only 1 sec at the middle.
		def in_window(o):
			# only detect the onset inside the target windows, to make sure the long tail can be included.
			return ( o >= self.window_overlap_t and o < self.window_overlap_t + self.window_t)
		onsets_in_windows = [o for o in onsets_cqt_time_list if in_window(o)]
		rospy.logdebug(f"onset_cqt:{onsets_in_windows}")

		# since the onset are extracted, then we need to pass them through the classification model to get note label.
		winners_raw_idx = []
		winner_onsets = []
		durations = []
		default_duration = 0.5
		# publish events and plot visualization
		for onset in onsets_in_windows:
			fundamental_frequency, confidence, winner_raw_idx, winner_pm_idx = self.onset_classification(onset)

			if winner_raw_idx is not None:
				# find the y-position of onset in spectrum
				winners_raw_idx.append(winner_raw_idx)  # ys
				winner_onsets.append(onset)  # xs
				durations.append(default_duration)


				# publish the onset note to topic
				t = self.buffer_time + rospy.Duration(onset)
				no = NoteOnset()
				no.header.stamp = t
				note = hz_to_note(fundamental_frequency)
				no.note = note
				no.confidence = confidence
				no.duration = default_duration
				no.loudness = default_duration
				self.pub_onset.publish(no)

				rospy.logdebug(
					f"music note detected: "
					f"time:{t} "
					f"note:{note} "
					f"confidence:{confidence:.4f} "
				)

		# update the sectrum visualization
		self.update_spectrogram(
			spec=cqt,
			onsets_cqt=onsets_in_windows,
			winners_onsets_hops=winner_onsets,
			winners_raw_idx=winners_raw_idx,
			durations=durations,
		)

		# advance buffer, keep one overlap for next processing
		self.buffer_time += rospy.Duration(self.window_t)
		self.buffer = self.buffer[-(2 * self.window_overlap_num):]

		rospy.loginfo_once("onset detection is online")
		compute_time = rospy.Time.now() - msg_time
		self.pub_compute_time.publish(compute_time.to_sec())
		if compute_time > rospy.Duration(self.window_t):
			rospy.logerr("computation took longer than processed window")

	def sec2hops(self,sec):
		return int(self.sr/self.hop_length*sec)

	def onset_classification(self, onset):
		"""
			input: onset, a float value from 0 to 1, denote the percentage position of 1 sec
			output:
				-   winner_freq: the freq of the winner signal
				-   max(buckets[winner]): the confidence
				-   winner_idx_in_spec: the idx in spec along y-axis
				-   winner_pm_idx: the note id in the pretty_midi
		"""
		# using 0.1 sec windows data for classification
		prediction_averaging_window = (
			self.windows_for_classification * self.sr
		)
		# extract the data from onset time until 0.1 sec later
		excerpt = self.buffer[int(onset * self.sr):int(onset * self.sr + prediction_averaging_window)]
		# neuron nets for onset classification
		time, freq, confidence, _ = crepe.predict(
			excerpt,
			self.sr,
			viterbi=True,
			model_capacity=self.crepe_model,
			verbose=0
		)

		# filter out the onset, which confidence lower that threshold and note beyond range(C4-C7)
		confidence_mask = confidence > self.confidence_threshold
		freq_mask = (freq >= self.fmin) & (freq <= self.fmax)
		mask = confidence_mask & freq_mask
		filtered_freq = freq[mask]
		filtered_confidence = confidence[mask]

		if len(filtered_freq) > 0:
			# rospy.logdebug(f"freq:{[hz_to_note(hz) for hz in filtered_freq]}, confidence:{filtered_confidence}")
			buckets = {}
			for f, c in zip(filtered_freq, filtered_confidence):
				note = hz_to_note(f)
				buckets[note] = buckets.get(note, []) + [c]

			def add_confidence(note):
				return reduce(lambda x, y: x + y, buckets.get(note))
			winner = max(buckets, key=lambda a: add_confidence(a))
			winner_freq = note_to_hz(winner)
			winner_pm_idx = note_to_pm_id(winner)
			winner_raw_idx_in_spec = note_to_pm_id(winner)-self.fmin_note_id
			confidence = max(buckets[winner])
			return winner_freq, confidence, winner_raw_idx_in_spec, winner_pm_idx
		else:
			return 0.0, 0.0, None, None

	# the draw of spectrum has 1 sec delay, the spectrum include 2 sec data
	def update_spectrogram(self, spec, onsets_cqt, winners_onsets_hops, winners_raw_idx, durations):
		if self.pub_spectrogram.get_num_connections() == 0:
			self.spectrogram = None
			return

		# throw away overlap, left 1 sec frame as target windows
		spec = spec[:, self.overlap_hops:-self.overlap_hops]
		log_spec = np.log(spec+1e-8)

		if self.spectrogram is None:
			self.spectrogram = log_spec
			return
		elif self.spectrogram.shape[1] > spec.shape[1]:
			# sliding windows, move the old 1 sec frame to the left.
			self.spectrogram = self.spectrogram[:, -spec.shape[1]:]

		# add the previous frame to current frame
		self.spectrogram = np.concatenate([self.spectrogram, log_spec], 1)
		# normalizes per compute over max value
		spectrogram = np.array(self.spectrogram / np.max(self.spectrogram) * 255, dtype=np.uint8)
		heatmap = cv2.applyColorMap(spectrogram, cv2.COLORMAP_JET)
		assert len(durations) == len(winners_raw_idx)

		# convert the onset time to pixel idx in the spectrum
		onsets_hops = [self.sec2hops(onset-self.window_overlap_t+self.window_t) for onset in onsets_cqt]

		WINNER_LINECOLOR = [0, 255, 0]
		for idx, t in enumerate(self.previous_winner_onsets_hops):
			t_end = t + self.sec2hops(self.previous_durations[idx])
			heatmap[self.previous_winners[idx], max(t,0):min(t_end, spectrogram.shape[1])][:] = WINNER_LINECOLOR

		winners_onsets_hops = [self.sec2hops(y - self.window_overlap_t + self.window_t) for y in winners_onsets_hops]  # shift an overlap windows for visualization
		for idx, t in enumerate(winners_onsets_hops):
			t_end = t + self.sec2hops(durations[idx])
			heatmap[winners_raw_idx[idx], max(t,0):min(t_end, spectrogram.shape[1])][:] = WINNER_LINECOLOR

		# draw onset line as white
		ONSET_LINECOLOR = [255, 255, 255]
		for t in self.previous_onsets_hops:
			heatmap[:,t] = ONSET_LINECOLOR
		for t in onsets_hops:
			heatmap[:,t] = ONSET_LINECOLOR

		self.pub_spectrogram.publish(
			self.cv_bridge.cv2_to_imgmsg(heatmap, "bgr8")
		)

		# keep the histories
		self.previous_winner_onsets_hops = []
		self.previous_winners = []
		self.previous_durations = []
		for idx, winner_onset_hop in enumerate(winners_onsets_hops):
			hop = winner_onset_hop - int(self.window_num / self.hop_length)
			if hop > 0:
				self.previous_winner_onsets_hops.append(hop)
				self.previous_winners.append(winners_raw_idx[idx])
				self.previous_durations.append(durations[idx])

		self.previous_onsets_hops = []
		for onsets_hop in onsets_hops:
			hop = onsets_hop - self.sec2hops(self.window_t)
			self.previous_onsets_hops.append(hop)

	def cqt(self):
		"""
			The function for constant Q transform
			input: self.buffer
			output: ndarrary with shape (60,173) by default
		"""
		cqt = np.abs(
			librosa.cqt(
				y=self.buffer,
				sr=self.sr,
				hop_length=self.hop_length,
				fmin=self.fmin,
				n_bins=self.semitones,
			)
		)
		return cqt

	def publish_cqt(self, cqt):
		msg = CQTStamped()
		msg.number_of_semitones = self.semitones
		msg.min_note = self.fmin_note
		msg.hop_length = rospy.Duration(self.hop_length / self.sr)
		msg.header.stamp = self.buffer_time + rospy.Duration(self.window_overlap_t)
		msg.data = cqt[:, self.overlap_hops:-self.overlap_hops].flatten(order="F")
		self.pub_cqt.publish(msg)

if __name__ == '__main__':
	rospy.init_node("detect_onset",log_level=rospy.DEBUG)
	detector = OnsetDetection()
	detector.start()
