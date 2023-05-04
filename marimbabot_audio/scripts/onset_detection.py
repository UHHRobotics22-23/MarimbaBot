from functools import reduce
import struct
import cv_bridge
import rospy
from audio_common_msgs.msg import AudioDataStamped, AudioInfo
from sensor_msgs.msg import Image
from std_msgs.msg import Float32
from marimbabot_audio.msg import NoteOnset, CQTStamped
import pretty_midi
import librosa
import crepe
import cv2
import numpy as np
import matplotlib.pyplot as plt
from matplotlib.colors import ListedColormap


'''
    Some default parameters:
        -   Sampling rate = 44100
        -   Sampleing format = S16LE

'''
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
    rospy.loginfo("Waiting for Audio Info")
    info = rospy.wait_for_message("audio_info", AudioInfo)
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
        rospy.loginfo("Audio compatible")
        return True
    return False

def unpack_data(data):
    return np.array(struct.unpack(
        f"{int(len(data) / 2)}h",
        bytes(data)
        ),
        dtype=float)


class OnsetDetection:
    def _init_instrument_config(self):
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

    def _init_audio_config(self):
        """
            The configuration of the audio signal
        """
        self.sr = 44100
        self.hop_length = 512

    def _init_detection_config(self):
        """
            confidence threshold for note classification(crepe)
        """
        self.confidence_threshold = 0.3  # the threshold for note classification
        self.windows_for_classification = 0.1  # using 0.1 sec data after onset time for note classification
        # preload model to not block the callback on first message
        # capacities: 'tiny', 'small', 'medium', 'large', 'full'
        self.crepe_model = "full"  # choose the crepe model type for music note classification
        rospy.loginfo(f"Loading crepe {self.crepe_model}-model...")
        crepe.core.build_and_load_model(self.crepe_model)
        rospy.loginfo(f"Crepe {self.crepe_model}-model loaded.")

    def _init_visualization_config(self):
        """
            config the spectrum plot of
        """
        hsv = plt.get_cmap("hsv")
        self.cmap = ListedColormap(np.vstack((
            hsv(np.linspace(0, 1, 86)),
            hsv(np.linspace(0, 1, 85)),
            hsv(np.linspace(0, 1, 85)))
        ))
        self.cmap.set_bad((0, 0, 0, 1))  # make sure they are visible

        # number of samples for analysis window
        self.window_t = 1.0
        # and overlap regions between consecutive windows
        self.window_overlap_t = 0.5
        self.window_all_t = self.window_t+2*self.window_overlap_t

        self.window = int(self.sr * self.window_t)
        self.window_overlap = int(self.sr * self.window_overlap_t)


        self.overlap_hops = int(self.window_overlap / self.hop_length)

        self.cv_bridge = cv_bridge.CvBridge()

    def warm_up(self):
        # warm up classifier / jit caches
        _ = self.cqt()
        _ = self.onset_classification(0.0)
        self.reset()

    def __init__(self):

        self._init_instrument_config()
        self._init_audio_config()
        self._init_detection_config()
        self._init_visualization_config()


        if not check_audio_format():
            rospy.signal_shutdown("incompatible audio format")
            return

        # other parameters
        self.last_time = rospy.Time.now()
        self.last_seq = 0

        # the buffer to read audio raw data from ros topic
        self.buffer = np.array([0.0] * self.sr, dtype=float)

        # warm up classifier / jit caches
        self.warm_up()

        self.first_input = True

    def start(self):
        # the spectrum for visualization
        self.pub_spectrogram = rospy.Publisher(
            "spectrogram", Image, queue_size=1, tcp_nodelay=True
        )
        self.pub_compute_time = rospy.Publisher(
            "~compute_time", Float32, queue_size=1, tcp_nodelay=True
        )
        # signal after constant Q transform
        self.pub_cqt = rospy.Publisher(
            "cqt", CQTStamped, queue_size=100, tcp_nodelay=True
        )
        # onset signals
        self.pub_onset = rospy.Publisher(
            "onsets", NoteOnset, queue_size=100, tcp_nodelay=True
        )
        self.sub = rospy.Subscriber(
            "audio_stamped",
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
        self.previous_winner_onsets_times = []
        self.previous_onsets_times = []
        self.previous_winners = []

    # the most important function for signal processing logic
    def audio_process(self, msg):
        # unpack the msg
        now = msg.header.stamp
        seq = msg.header.seq
        msg_data = unpack_data(msg.audio.data)

        # handle bag loop graciously
        if now < self.last_time:
            rospy.loginfo("detected bag loop")
            self.reset()
        # make sure the seq in order
        if seq > self.last_seq + 1 and not self.first_input:
            jump = seq - self.last_seq
            rospy.logwarn(
                f"sample drop detected: seq jumped "
                f"from {self.last_seq} to {seq} "
                f"(difference of {jump})"
            )
            end_of_buffer_time = \
                self.buffer_time + rospy.Duration(self.buffer.shape[0] / self.sr)
            self.reset()
            self.buffer_time = \
                end_of_buffer_time + rospy.Duration((jump - 1) * (len(msg_data) / self.sr))
        elif seq != self.last_seq + 1 and not self.first_input:
            rospy.logwarn(f"something weird happened, seq jumped from {self.last_seq} to {seq}")

        self.first_input = False
        self.last_time = now
        self.last_seq = seq
        self.last_seq = seq

        # take time from message headers and increment based on data
        if self.buffer_time is None:
            self.buffer_time = now
        self.buffer = np.concatenate([
            self.buffer,
            msg_data
        ])

        # aggregate buffer until window+2*overlaps are full, like [0.5 Sec Overlap | 1 Sec window | 0.5 Sec Overlap]
        if self.buffer.shape[0] < self.window + 2 * self.window_overlap:
            return

        # TODO: clearn noise
        #self.buffer = reduce_noise_power(self.buffer,sr=self.sr)



        # TODO: it would be better to do the computation below asynchronously
        """
            constant q transform with 60 half-tones from C4,
            in theory we only need notes from C4-C7, but in practice tuning
            is often too low and harmonics are needed above C6,
            therefore we use 60 semitones include 2 octaves overtone.
        """
        cqt = self.cqt()  # cqt ndarrary  (60,173)
        self.publish_cqt(cqt)


        # TODO: visualization of the loudness to figure out the rght way for duration detection
        # to extract the cqt within the target windows
        cqt_len = cqt.shape[1]
        cqt_overlap_num = int(self.window_overlap_t/self.window_all_t*cqt_len)
        _cqt = cqt[:,cqt_overlap_num:-cqt_overlap_num]
        _cqt = _cqt[0,:]

        loudness_seq = librosa.power_to_db(cqt**2)

        # rospy.loginfo(f"dB(median:{np.median(db)} max:{np.max(db)}, avg:{np.average(db)}):{db}")
        onset_env_cqt = librosa.onset.onset_strength(
            sr=self.sr, S=librosa.amplitude_to_db(cqt, ref=np.max)
        )

        # detect where the onset happened
        onsets_cqt_raw = librosa.onset.onset_detect(
            y=self.buffer,
            sr=self.sr,
            hop_length=self.hop_length,
            onset_envelope=onset_env_cqt,
            units="time",
            backtrack=False,
            # wait= 0.1*self.sr/self.hop_length,
            delta=4.0,
            normalize=False,
        )

        # the whole windows include 0.5 sec overlap at both end, the target windows is only 1 sec.
        def in_window(o):
            # make sure the onset inside the target windows
            return (
                    o >= self.window_overlap_t and
                    o < self.window_t + self.window_overlap_t
            )

        # filter out the onset in the overlap windows
        onsets_cqt = [
            o
            for o in onsets_cqt_raw
            if in_window(o)
        ]

        winners_idx = []
        winner_onsets_times = []
        durations_len = []

        # publish events and plot visualization
        for o in onsets_cqt:
            fundamental_frequency, confidence, winner_idx, winner_pm_idx = self.onset_classification(o)


            if winner_idx is not None:
                # find the y-position of onset in spectrum
                onset_time = self.onsets_to_time_in_spec(o)
                # extract the duration and loudness of the onset note
                duration, duration_len, loudness = self.extract_duration(
                    onset_time=onset_time,
                    winner_idx=winner_idx,
                    cqt=cqt,
                    loudnes_seq=loudness_seq
                )

                winners_idx.append(winner_idx)
                winner_onsets_times.append(onset_time)
                durations_len.append(duration_len)


                # publish the onset note to topic
                t = self.buffer_time + rospy.Duration(o)
                no = NoteOnset()
                no.header.stamp = t
                note = hz_to_note(fundamental_frequency)
                no.note = note
                no.confidence = confidence
                no.duration = duration
                no.loudness = loudness
                self.pub_onset.publish(no)

                rospy.logdebug(
                    f"music note detected: "
                    f"time:{t} "
                    f"note:{note} "
                    f"confidence:{confidence:.4f} "
                    f"duration:{duration:.4f} "
                    f"loudness:{loudness:.4f}"
                )

        # update the sectrum visualization
        self.update_spectrogram(
            spec=cqt,
            onsets_cqt=onsets_cqt,
            ys=winner_onsets_times,
            xs=winners_idx,
            durations_len=durations_len,
        )

        if len(onsets_cqt) == 0:
            rospy.logdebug("found no onsets")
        else:
            rospy.logdebug("found {} onsets".format(len(onsets_cqt)))

        # advance buffer, keep one overlap for next processing
        self.buffer_time += rospy.Duration(self.window_t)
        self.buffer = self.buffer[(-2 * self.window_overlap):]

        rospy.loginfo_once("onset detection is online")
        compute_time = rospy.Time.now() - now
        self.pub_compute_time.publish(compute_time.to_sec())
        if compute_time > rospy.Duration(self.window):
            rospy.logerr("computation took longer than processed window")

    def extract_duration(self, onset_time, winner_idx, cqt, loudnes_seq, frame_len=173, intensity_threshold_ratio=0.4):
        xs = winner_idx
        y = onset_time

        max_intensity = np.max(cqt[xs, y - 5:y + 10])
        loudness = np.max(loudnes_seq[xs, y - 5:y + 10])
        m = 1
        while True:
            this_intensity = np.mean(cqt[xs, y + m - 1:y + m + 1])
            if (this_intensity / (max_intensity + 0.00001) < intensity_threshold_ratio) or (m + y >= frame_len - 1):
                break
            m += 1
        duration_hop = m
        duration_t = duration_hop * self.hop_length / self.sr
        return duration_t, duration_hop, loudness

    def onsets_to_time_in_spec(self, onset):
        """
            calculate the onset index for each spectrum along y-axis(time)
        """
        onset = onset - self.window_overlap_t
        return int(self.window / self.hop_length + onset * self.sr / self.hop_length)

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
            buckets = {}
            for f, c in zip(filtered_freq, filtered_confidence):
                note = hz_to_note(f)
                buckets[note] = buckets.get(note, []) + [c]

            def add_confidence(note):
                return reduce(lambda x, y: x + y, buckets.get(note))
            winner = max(buckets, key=lambda a: add_confidence(a))
            winner_freq = note_to_hz(winner)
            winner_pm_idx = note_to_pm_id(winner)
            winner_idx_in_spec = note_to_pm_id(winner)-self.fmin_note_id

            return winner_freq, max(buckets[winner]), winner_idx_in_spec, winner_pm_idx
        else:
            return 0.0, 0.0, None, None

    def update_spectrogram(self, spec,onsets_cqt, ys, xs, durations_len):
        if self.pub_spectrogram.get_num_connections() == 0:
            self.spectrogram = None
            return
        

        onsets_time = [self.onsets_to_time_in_spec(onset) for onset in onsets_cqt]


        # throw away overlap
        spec = spec[:, self.overlap_hops:-self.overlap_hops]

        log_spec = np.log(spec+1e-8)

        if self.spectrogram is None:
            self.spectrogram = log_spec
            return
        elif self.spectrogram.shape[1] > spec.shape[1]:
            self.spectrogram = self.spectrogram[:, -spec.shape[1]:]
        self.spectrogram = np.concatenate([self.spectrogram, log_spec], 1)

        # normalizes per compute
        spectrogram = np.array(
            self.spectrogram / np.max(self.spectrogram) * 255, dtype=np.uint8
        )

        heatmap = cv2.applyColorMap(spectrogram, cv2.COLORMAP_JET)

        assert len(durations_len) == len(xs)

        WINNER_LINECOLOR = [0, 255, 0]
        for idx, t in enumerate(self.previous_winner_onsets_times):
            t_end = t + self.previous_durations_len[idx]
            if t_end < 173 and self.previous_winners[idx] < 60:
                heatmap[self.previous_winners[idx], t:t_end][:] = WINNER_LINECOLOR

        for idx, t in enumerate(ys):
            t_end = t + durations_len[idx]
            if t_end < 173 and xs[idx] < 60:
                heatmap[xs[idx], t:t_end][:] = WINNER_LINECOLOR
        
        ONSET_LINECOLOR = [255, 255, 255]
        for t in self.previous_onsets_times:
            heatmap[:,t] = ONSET_LINECOLOR
        for t in onsets_time:
            heatmap[:,t] = ONSET_LINECOLOR

        # keep the histories
        self.previous_winner_onsets_times = [winner_onset_time - int(self.window / self.hop_length) for winner_onset_time in ys]
        self.previous_winners = xs
        self.previous_durations_len = durations_len
        self.previous_onsets_times = [onset_time - int(self.window / self.hop_length) for onset_time in onsets_time]

        self.pub_spectrogram.publish(
            self.cv_bridge.cv2_to_imgmsg(heatmap, "bgr8")
        )

    def cqt(self):
        """
            The function for constant Q transform
            input: self.buffer
            output: ndarrary with shape (60,173) by default
        """
        # TODO: subtract mean background noise from CQT
        return np.abs(
            librosa.cqt(
                y=self.buffer,
                sr=self.sr,
                hop_length=self.hop_length,
                fmin=self.fmin,
                n_bins=self.semitones,
            )
        )

    def publish_cqt(self, cqt):
        msg = CQTStamped()
        msg.number_of_semitones = self.semitones
        msg.min_note = self.fmin_note
        msg.hop_length = rospy.Duration(self.hop_length / self.sr)

        msg.header.stamp = \
            self.buffer_time + rospy.Duration(self.window_overlap_t)
        msg.data = \
            cqt[:, self.overlap_hops:-self.overlap_hops].flatten(order="F")
        self.pub_cqt.publish(msg)

if __name__ == '__main__':
    rospy.init_node("detect_onset")
    detector = OnsetDetection()
    detector.start()
