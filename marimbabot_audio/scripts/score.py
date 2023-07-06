import rospy
from marimbabot_msgs.msg import HitSequence as HitSequenceMsg
from marimbabot_msgs.msg import SequenceScore as SequenceScoreMsg
from marimbabot_msgs.msg import NoteOnset as NoteOnsetMsg


class Measurement():
	def __init__(self, windowsize=1, plan_delay=0.0):
		self.windowsize = windowsize
		self.plan_delay = plan_delay
		self.note_seq_audio = []
		self.note_seq_vision = []
		# start and end time of the sequence to be measured
		self.plan_start_time = None
		self.plan_end_time = None
		self.vision_seq_id = None
		self.duration_all_note_plan = None
		self.init_ros()

	def init_ros(self):
		self.seq_sub = rospy.Subscriber(
			'/audio/hit_sequence',
			HitSequenceMsg,
			self.hit_sequence_callback,
			queue_size=10,
			tcp_nodelay=True)
		self.score_pub = rospy.Publisher(
			'/audio/sequence_score',
			SequenceScoreMsg,
			queue_size=10,
			tcp_nodelay=True)
		self.note_sub = rospy.Subscriber(
			'/audio/onset_notes',
			NoteOnsetMsg,
			self.audio_note_sequence_callback,
			queue_size=10,
			tcp_nodelay=True)

	def reset(self):
		self.note_seq_audio = []
		self.note_seq_vision = []
		self.plan_start_time = None
		self.plan_end_time = None
		self.vision_seq_id = None
		self.duration_all_note_plan = None

	def audio_note_sequence_callback(self, msg, time_tolerance=1):
		"""
			Collects the audio sequence within the time range of the target sequence with a tolerance of time_tolerance
			to ensure the sequence is as complete as possible.
		"""
		if self.plan_start_time is not None:
			if msg.header.stamp > self.plan_start_time - rospy.Duration(
					time_tolerance) and msg.header.stamp < self.plan_end_time + rospy.Duration(time_tolerance):
				self.note_seq_audio.append(msg)

	def hit_sequence_callback(self, msg):
		"""
			The time in header should be the timing of plan start aka. the timing of the first hit
		"""
		self.reset()
		self.note_seq_vision = msg.hit_sequence_elements
		self.duration_all_note_plan = rospy.Duration(0)
		for i in range(len(self.note_seq_vision)):
			self.duration_all_note_plan += self.note_seq_vision[i].tone_duration
		self.vision_seq_id = msg.sequence_id
		self.plan_start_time = msg.header.stamp + rospy.Duration(self.plan_delay)
		self.plan_end_time = self.duration_all_note_plan + self.plan_start_time

	def is_time_for_comparison(self):
		"""
			Checks if the sequence is ready for comparison
		"""
		if len(self.note_seq_audio) != 0 and rospy.Time.now() > self.plan_end_time + rospy.Duration(2):
			return True
		else:
			return False

	def compare(self, time_tolerance=1):
		"""
			Compare the target sequence with the extracted sequence, and return the recall rate.
		"""
		length = len(self.note_seq_vision)
		success_times = 0
		for hit_element_msg in self.note_seq_vision:
			plan_time_this_hit = hit_element_msg.start_time  # relative plan hit time, start from zero
			plan_time_this_hit = rospy.Duration(plan_time_this_hit.to_sec())  # relative plan hit time, start from zero
			plan_time_this_hit = plan_time_this_hit + self.plan_start_time  # plus the absolut first plan hit time, to get the absolut time of this hit
			plan_time_range_low = plan_time_this_hit - rospy.Duration(
				self.windowsize)  # the time range to search for the hit to enable a tolerance
			plan_time_range_high = plan_time_this_hit + rospy.Duration(
				self.windowsize)  # the time range to search for the hit to enable a tolerance
			# retrieve each plan note and check if it is in the time range
			for note_msg in self.note_seq_audio:
				audio_note_time = note_msg.header.stamp
				if audio_note_time > plan_time_range_low and audio_note_time < plan_time_range_high:
					# if the note is in the time range, check if it is the right note, and if it is, take it into
					# account and break the loop.
					if note_msg.note == hit_element_msg.tone_name + str(hit_element_msg.octave):
						success_times += 1
						break
		return success_times / length

	def score_calculation(self):
		"""
			Waits for the sequence to be detected and then compares the sequence with the target sequence
		"""
		rate = rospy.Rate(10)  # 10hz
		while not rospy.is_shutdown():
			if self.is_time_for_comparison():
				# print the sequence to be compared
				audio_start = self.note_seq_audio[0].header.stamp.to_sec()
				audio_print = [f"{msg.note},{msg.header.stamp.to_sec()-audio_start}" for msg in self.note_seq_audio]
				vision_print = [f"{msg.tone_name + str(msg.octave)},{msg.start_time.to_sec()}" for msg in
				                self.note_seq_vision]
				rospy.logdebug('-' * 40)
				rospy.logdebug(f'got audio len:{len(self.note_seq_audio)}  vision len:{len(self.note_seq_vision)}')
				rospy.logdebug(f'extracted audio seq: {audio_print}')
				rospy.logdebug(f'received vision seq: {vision_print}')
				if len(self.note_seq_vision) != len(self.note_seq_vision):
					rospy.logwarn('The length of the received sequence and the target sequence are different.')
				score = self.compare()
				self.reset()
				msg = SequenceScoreMsg()
				msg.sequence_id = self.vision_seq_id
				msg.score = score
				msg.header.stamp = rospy.Time.now()
				self.score_pub.publish(msg)
				rospy.logdebug(f'Publish Score: ID: {msg.sequence_id} Score(recall): {msg.score}')
			rate.sleep()


if __name__ == '__main__':
	rospy.init_node('score_calculator', log_level=rospy.DEBUG)
	measurement = Measurement(windowsize=1, plan_delay=0.0)
	measurement.score_calculation()
	rospy.spin()
