import rospy
from marimbabot_msgs.msg import HitSequence as HitSequenceMsg
from marimbabot_msgs.msg import SequenceScore as SequenceScoreMsg
from marimbabot_msgs.msg import NoteOnset as NoteOnsetMsg


class Measurement():
	def __init__(self):
		self.note_seq_audio = []
		self.note_seq_vision = []
		# start and end time of the sequnce to be measured
		self.plan_start_time = None
		self.plan_end_time = None
		self.vision_seq_id = None
		self.note_duration = None
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
			self.note_sequence_callback,
			queue_size=10,
			tcp_nodelay=True)

	def reset(self):
		self.note_seq_audio = []
		self.note_seq_vision = []
		self.plan_start_time = None
		self.plan_end_time = None
		self.vision_seq_id = None
		self.note_duration = None

	def note_sequence_callback(self, msg, time_tolerance=1):
		if self.plan_start_time is not None:
			if msg.header.stamp > self.plan_start_time - rospy.Duration(
					time_tolerance) and msg.header.stamp < self.plan_end_time + rospy.Duration(time_tolerance):
				self.note_seq_audio.append(msg)

	def hit_sequence_callback(self, msg):
		self.reset()
		self.note_seq_vision = msg.hit_sequence_elements
		self.note_duration = self.note_seq_vision[0].tone_duration
		self.vision_seq_id = msg.sequence_id
		self.plan_start_time = msg.header.stamp
		self.plan_end_time = len(msg.hit_sequence_elements) * self.note_duration + self.plan_start_time

	def is_time_for_comparison(self):
		'''
			Checks if the sequence is ready for comparison
		'''
		if len(self.note_seq_audio) != 0 \
				and rospy.Time.now() > self.plan_end_time+rospy.Duration(2) \
				and self.note_seq_audio[-1].header.stamp > self.plan_end_time:

			self.plan_start_time = None
			self.plan_end_time = None
			return True
		else:
			return False

	def compare(self, time_tolerance=1):
		'''
			Compare the target sequence with the extracted sequence, and return the recall rate.
		'''
		length = len(self.note_seq_vision)
		success_times = 0
		for hit_element_msg in self.note_seq_vision:
			start_time_plan = hit_element_msg.start_time
			start_time_plan_min = start_time_plan - rospy.Duration(time_tolerance)
			start_time_plan_max = start_time_plan + rospy.Duration(time_tolerance)
			for note_msg in self.note_seq_audio:
				if note_msg.header.stamp > start_time_plan_min and note_msg.header.stamp < start_time_plan_max:
					if note_msg.note == hit_element_msg.tone_name + str(hit_element_msg.octave):
						success_times += 1
						break
		return success_times / length

	def score_calculatioin(self):
		'''
			Waits for the sequence to be detected and then compares the sequence with the target sequence
		'''
		rate = rospy.Rate(10)  # 10hz
		while not rospy.is_shutdown():
			if self.is_time_for_comparison():
				# print the sequence to be compared
				audio_print = [f"{msg.note},{msg.header.stamp.to_sec()}" for msg in self.note_seq_audio]
				vision_print = [f"{msg.tone_name + str(msg.octave)},{msg.start_time.to_sec()}" for msg in
				                self.note_seq_vision]
				rospy.logdebug('-------------------')
				rospy.logdebug(f'got audio len:{len(self.note_seq_audio)}  vision len:{len(self.note_seq_vision)}')
				rospy.logdebug(f'extracted audio seq: {audio_print}')
				rospy.logdebug(f'received vision seq: {vision_print}')
				if len(self.note_seq_vision) != len(self.note_seq_vision):
					rospy.logwarn('The length of the received sequence and the target sequence are different.')

				score = self.compare()
				msg = SequenceScoreMsg()
				msg.sequence_id = self.vision_seq_id
				msg.score = score
				msg.header.stamp = rospy.Time.now()
				self.score_pub.publish(msg)
				self.reset()
				rospy.logdebug(f'Publish Score: ID: {msg.sequence_id} Score(recall): {msg.score}')
			rate.sleep()


if __name__ == '__main__':
	rospy.init_node('score_calculator', log_level=rospy.DEBUG)
	measurement = Measurement()
	measurement.score_calculatioin()
	rospy.spin()
