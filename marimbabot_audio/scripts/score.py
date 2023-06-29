import rospy
from marimbabot_msgs.msg import HitSequence as HitSequenceMsg
from marimbabot_msgs.msg import SequenceScore as SequenceScoreMsg

class Measurement():
	def __init__(self):
		self.notes_seq_audio = []
		self.note_seq_vision = []

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

	def hit_sequence_callback(self, msg):
		seq_id = msg.seq_id
		seq = msg.seq
		this_satrt = rospy.Time.now()
		total_time = 0
		for hit in seq:
			total_time += hit.duration
		this_end = this_satrt + rospy.Duration(total_time)

	def extract_seq_from_audio_dectection(self):
		pass

