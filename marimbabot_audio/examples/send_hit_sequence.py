from marimbabot_msgs.msg import HitSequence as HitSequenceMsg
from marimbabot_msgs.msg import HitSequenceElement as HitSequenceElementMsg
from marimbabot_msgs.msg import SequenceScore as SequenceScoreMsg
import numpy as np
import rospy
def gererate_hit_sequence_msg(seq:list):
	msg = HitSequenceMsg()
	msg.header.stamp = rospy.Time.now()
	msg.sequence_id = 0
	msg.hit_sequence_elements = []
	time_start = rospy.Time.now()
	for idx, each in enumerate(seq):
		hit = HitSequenceElementMsg()
		'''
		string tone_name
		int32 octave
		time start_time
		duration tone_duration
		float32 loudness
		'''
		hit.tone_name = each[:-1]
		hit.octave = each[-1]
		interval = rospy.Duration(0.5)
		hit.start_time = time_start + interval* idx
		hit.tone_duration = 0.5 + np.random.uniform(-0.2,0.2)
		hit.loudness = 0.5 + np.random.uniform(-0.2,0.2)
		msg.seq.append(hit)
	return msg

def send_hit_sequnce():
	seq = ['C4', 'C#4', 'D4', 'D#4', 'E4', 'F4', 'G4', 'G#4', 'A4', 'B4', 'C5']
	rospy.init_node('send_hit_sequnce', anonymous=True)
	pub = rospy.Publisher('/audio/hit_sequence', HitSequenceMsg, queue_size=10)
	rate = rospy.Rate(0.5) # 10hz
	while not rospy.is_shutdown():
		msg = gererate_hit_sequence_msg(seq)
		pub.publish(msg)
		rate.sleep()

if __name__ == '__main__':
	rospy.init_node('send_hit_sequnce', anonymous=True)
	send_hit_sequnce()
