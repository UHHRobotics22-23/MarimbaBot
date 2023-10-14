import rospy
from std_msgs.msg import Header

from marimbabot_msgs.msg import HitSequence as HitSequenceMsg
from marimbabot_msgs.msg import NoteOnset as NoteOnsetMsg
from marimbabot_msgs.msg import SequenceMatchResult as SequenceMatchResultMsg


class Comparison():
	def __init__(self, windows_t_for_compare=1):
		"""
			:param windows_t_for_compare: parameter to compare the audio sequence and plan sequence,
				if time offset is within this window, the note is considered as correct
		"""
		self.audio_feedback_sequence_to_compare = []
		self.audio_feedback_sequence = []
		self.plan_sequence = []

		self.windows_t_for_compare = windows_t_for_compare  # be used to compare the audio feedback sequence and plan sequence
		self.windows_t_for_target_extract = 1  # be used to extract the music note sequence from the audio feedback sequence

		# start and end time of the sequence to be measured
		self.plan_start_time = None
		self.plan_end_time = None
		self.plan_seq_id = None
		self.plan_duration_all = None

		# init ros content
		self.init_ros()

	def init_ros(self):
		self.seq_sub = rospy.Subscriber(
			'/audio/hit_sequence',
			HitSequenceMsg,
			self.plan_sequence_callback,
			queue_size=10,
			tcp_nodelay=True)
		self.result_pub = rospy.Publisher(
			'match_result',
			SequenceMatchResultMsg,
			queue_size=1,
			tcp_nodelay=True)
		self.note_sub = rospy.Subscriber(
			'/audio_node/onset_notes',
			NoteOnsetMsg,
			self.audio_feedback_sequence_callback,
			queue_size=10,
			tcp_nodelay=True)

	def reset(self):
		self.audio_feedback_sequence = []
		self.plan_sequence = []
		self.audio_feedback_sequence_to_compare = []
		self.plan_start_time = None
		self.plan_end_time = None
		self.plan_seq_id = None
		self.plan_duration_all = None

	# add the msg to audio sequence
	def audio_feedback_sequence_callback(self, msg):
		"""
			Collects the audio sequence from the audio feedback node
		"""
		self.audio_feedback_sequence.append(
			{
				'note': msg.note,
				"time": msg.header.stamp,
			}
		)

	# add the msg to plan sequence
	def plan_sequence_callback(self, msg):
		"""
			The time in header should be the timing of plan start aka. the timing of the first hit in plan
		"""
		# extract start time and end time of the whole plan sequence
		self.plan_start_time = msg.hit_sequence_elements[0].start_time
		self.plan_end_time = msg.hit_sequence_elements[-1].start_time

		# add each hit element to the plan sequence list
		for hit_element in msg.hit_sequence_elements:
			self.plan_sequence.append(
				{
					"note": hit_element.tone_name + str(hit_element.octave),
					"time": hit_element.start_time,  # the start time is the absolut time
					"match": -1,
					"time_offset": 0,
				}
			)

		# extract the robot's actually played sequence from the audio feedback sequence
		time_boundary_low = self.plan_start_time - rospy.Duration(self.windows_t_for_target_extract)
		time_boundary_high = self.plan_end_time + rospy.Duration(self.windows_t_for_target_extract)
		self.audio_feedback_sequence_to_compare = []

		# search from the end of the audio feedback sequence
		for element in self.audio_feedback_sequence[::-1]:
			if time_boundary_low < element["time"] < time_boundary_high:
				self.audio_feedback_sequence_to_compare.append(element)
				if element["time"] < time_boundary_low:
					break

		# compare the two sequences
		self.compare()
		# publish the recall score and avg_time_offset
		msg_result = SequenceMatchResultMsg()
		msg_result.sequence_id = msg.sequence_id
		msg_result.score = self.recall_calculation()
		msg_result.time_offset_list = [note["time_offset"] for note in self.plan_sequence]
		msg_result.is_matched_list = [note["match"] for note in self.plan_sequence]
		msg_result.avg_time_offset = sum(msg_result.time_offset_list) / len(msg_result.time_offset_list)
		msg_result.hit_sequence_elements = msg.hit_sequence_elements
		msg_result.extracted_note_onsets = []
		for note in self.audio_feedback_sequence_to_compare:
			msg_result.extracted_note_onsets.append(
				NoteOnsetMsg(
					note=note["note"],
					header=Header(stamp=note["time"]),
					duration=0.5,
					loudness=0,
				)
			)
		self.result_pub.publish(msg_result)

	def compare(self):
		"""
			Compare the target sequence with the extracted sequence, and return the recall rate.
		"""
		for i, plan in enumerate(self.plan_sequence):
			plan_time_range_low = plan['time'] - rospy.Duration(
				self.windows_t_for_compare)  # the time range to search for the hit to enable a tolerance
			plan_time_range_high = plan['time'] + rospy.Duration(
				self.windows_t_for_compare)  # the time range to search for the hit to enable a tolerance
			# retrieve each plan note and check if it is in the time range
			for j, feedback in enumerate(self.audio_feedback_sequence_to_compare):
				if feedback["time"] > plan_time_range_low and feedback["time"] < plan_time_range_high:
					# if the note is in the time range, check if it is the right note, and if it is, take it into
					# account and break the loop.
					if feedback["note"].lower() == plan["note"].lower():
						self.plan_sequence[i]["match"] = j
						self.plan_sequence[i]["time_offset"] = (feedback["time"]-plan["time"]).to_sec()
						break

	def recall_calculation(self):
		"""
			Counting the successful match times as the final score
		"""
		success = 0
		for plan in self.plan_sequence:
			if plan["match"] != -1:
				success += 1
		return success/len(self.plan_sequence)

	def run(self):
		rospy.spin()



if __name__ == '__main__':
	rospy.init_node('score_calculator')
	window_size = rospy.get_param("~window_size", 1)  # sec
	compare = Comparison(windows_t_for_compare=1)
	compare.run()

