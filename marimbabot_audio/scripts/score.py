import rospy
from marimbabot_msgs.msg import HitSequence as HitSequenceMsg
from marimbabot_msgs.msg import SequenceScore as SequenceScoreMsg
from marimbabot_msgs.msg import NoteOnset as NoteOnsetMsg


class Measurement():
	def __init__(self, windowsize=1, plan_delay=0.0):
		self.windowsize = windowsize
		self.plan_delay = plan_delay
		self.audio_feedback_sequence = []
		self.plan_sequence = []
		# start and end time of the sequence to be measured
		self.plan_start_time = None
		self.plan_end_time = None
		self.plan_seq_id = None
		self.interval_all_note_plan = None
		self.init_ros()
		self.match_sign = []

	def init_ros(self):
		self.seq_sub = rospy.Subscriber(
			'/audio/hit_sequence',
			HitSequenceMsg,
			self.plan_sequence_callback,
			queue_size=10,
			tcp_nodelay=True)
		self.score_pub = rospy.Publisher(
			'/audio/sequence_score',
			SequenceScoreMsg,
			queue_size=10,
			tcp_nodelay=True)
		self.score_pub = rospy.Publisher(
			'/audio/evaluation',
			SequenceScoreMsg,
			queue_size=10,
			tcp_nodelay=True)
		self.note_sub = rospy.Subscriber(
			'/audio/onset_notes',
			NoteOnsetMsg,
			self.audio_feedback_sequence_callback,
			queue_size=10,
			tcp_nodelay=True)

	def reset(self):
		self.audio_feedback_sequence = []
		self.plan_sequence = []
		self.plan_start_time = None
		self.plan_end_time = None
		self.plan_seq_id = None
		self.interval_all_note_plan = None

	# add the msg to audio sequence
	def audio_feedback_sequence_callback(self, msg, time_tolerance=1):
		"""
			Collects the audio sequence within the time range of the target sequence with a tolerance of time_tolerance
			to ensure the sequence is as complete as possible.
		"""
		if self.plan_start_time is not None:
			if msg.header.stamp > self.plan_start_time - rospy.Duration(
					time_tolerance) and msg.header.stamp < self.plan_end_time + rospy.Duration(time_tolerance):
				self.audio_feedback_sequence.append(
					{
						'note': msg.note,
						"time": msg.header.stamp,
					}
				)

	# add the msg to plan sequence
	def plan_sequence_callback(self, msg):
		"""
			The time in header should be the timing of plan start aka. the timing of the first hit
		"""
		self.reset()
		self.plan_start_time = msg.header.stamp + rospy.Duration(self.plan_delay)
		self.interval_all_note_plan = rospy.Duration(0)
		for hit_element in msg.hit_sequence_elements:
			self.interval_all_note_plan += hit_element.tone_duration
		self.plan_end_time = self.interval_all_note_plan + self.plan_start_time
		self.plan_seq_id = msg.sequence_id
		for hit_element in msg.hit_sequence_elements:
			self.plan_sequence.append(
				{
					"note": hit_element.tone_name + str(hit_element.octave),
					"time": self.plan_start_time + rospy.Duration(hit_element.start_time.to_sec()),
					"match": None,
					"time_offset": 0,

				}
			)

	def is_time_for_comparison(self):
		"""
			Checks if the sequence is ready for comparison
		"""
		if len(self.audio_feedback_sequence) != 0 and rospy.Time.now() > self.plan_end_time + rospy.Duration(2):
			return True
		else:
			return False

	def compare(self):
		"""
			Compare the target sequence with the extracted sequence, and return the recall rate.
		"""
		for i, plan in enumerate(self.plan_sequence):
			plan_time_range_low = plan['time'] - rospy.Duration(
				self.windowsize)  # the time range to search for the hit to enable a tolerance
			plan_time_range_high = plan['time'] + rospy.Duration(
				self.windowsize)  # the time range to search for the hit to enable a tolerance
			# retrieve each plan note and check if it is in the time range
			for j, feedback in enumerate(self.audio_feedback_sequence):
				if feedback["time"] > plan_time_range_low and feedback["time"] < plan_time_range_high:
					# if the note is in the time range, check if it is the right note, and if it is, take it into
					# account and break the loop.
					if feedback["note"] == plan["note"]:
						self.plan_sequence[i]["match"] = j
						self.plan_sequence[i]["time_offset"] = (feedback["time"]-plan["time"]).to_sec()
						break

	def score_calculation(self):
		"""
			Counting the successful match times as the final score
		"""
		success = 0
		for plan in self.plan_sequence:
			if plan["match"] is not None:
				success += 1
		return success/len(self.plan_sequence)

	def pub_score(self, score):
		msg = SequenceScoreMsg()
		msg.sequence_id = self.plan_seq_id
		msg.score = score
		msg.header.stamp = rospy.Time.now()
		self.score_pub.publish(msg)
		rospy.logdebug(f'Publish Score: ID: {msg.sequence_id} Score(recall): {msg.score}')

	def update_midi(self):
		# TODO
		raise NotImplementedError()

	def run(self):
		"""
			Waits for the sequence to be detected and then compares the sequence with the target sequence
		"""
		rate = rospy.Rate(10)  # 10hz
		while not rospy.is_shutdown():
			if self.is_time_for_comparison():
				# print the sequence to be compared
				audio_start = self.audio_feedback_sequence[0].header.stamp.to_sec()
				audio_print = [f"{msg.note},{msg.header.stamp.to_sec() - audio_start}" for msg in self.audio_feedback_sequence]
				vision_print = [f"{msg.tone_name + str(msg.octave)},{msg.start_time.to_sec()}" for msg in
				                self.plan_sequence]
				rospy.logdebug('-' * 40)
				rospy.logdebug(f'got audio len:{len(self.audio_feedback_sequence)}  vision len:{len(self.plan_sequence)}')
				rospy.logdebug(f'extracted audio seq: {audio_print}')
				rospy.logdebug(f'received vision seq: {vision_print}')
				if len(self.plan_sequence) != len(self.plan_sequence):
					rospy.logwarn('The length of the received sequence and the target sequence are different.')

				self.compare()
				score = self.score_calculation()
				self.pub_score(score)
				self.update_midi()
				self.reset()
			rate.sleep()



if __name__ == '__main__':
	rospy.init_node('score_calculator', log_level=rospy.DEBUG)
	measurement = Measurement(windowsize=1, plan_delay=0.0)
	measurement.run()
	rospy.spin()
