from marimbabot_msgs.msg import HitSequence as HitSequenceMsg
from marimbabot_msgs.msg import HitSequenceElement as HitSequenceElementMsg
from marimbabot_msgs.msg import SequenceScore as SequenceScoreMsg
from marimbabot_msgs.msg import NoteOnset as NoteOnsetMsg
import numpy as np
import rospy
def gererate_sequence_msg(seq:list,id=0,noisy=True):
	msgs_audio = []
	msgs_vision = HitSequenceMsg()
	msgs_vision.header.stamp = rospy.Time.now()
	msgs_vision.sequence_id = id
	msgs_vision.hit_sequence_elements = []
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
		# build test msg for vision
		hit.tone_name = each[:-1]
		hit.octave = int(each[-1])
		interval = rospy.Duration(0.5*idx)
		hit.start_time = time_start + interval
		hit.tone_duration = rospy.Duration(0.5)
		hit.loudness = 0.5

		if noisy:
			hit.start_time += rospy.Duration(np.random.uniform(-0.2, 0.2))
			hit.tone_duration += rospy.Duration(np.random.uniform(-0.2, 0.2))
			hit.loudness += np.random.uniform(-0.2, 0.2)

		msgs_vision.hit_sequence_elements.append(hit)

		# build test msg for audio
		msg_audio = NoteOnsetMsg()
		msg_audio.header.stamp = time_start + interval
		# msg_audio.header.stamp = time_start + interval + rospy.Duration(2)
		msg_audio.loudness = 0.8
		msg_audio.confidence = 0.8
		msg_audio.duration = 0.5
		if noisy:
			msg_audio.header.stamp += rospy.Duration(np.random.uniform(-0.2,0.2))
			msg_audio.duration += np.random.uniform(-0.2,0.2)

		msg_audio.note = each
		msgs_audio.append(msg_audio)

	return msgs_audio, msgs_vision

def send_hit_sequnce4test():
	seq = ['C4', 'C#4', 'D4', 'D#4', 'E4', 'F4', 'G4', 'G#4', 'A4', 'B4', 'C5']
	vision_pub = rospy.Publisher('/audio/hit_sequence', HitSequenceMsg, queue_size=10)
	audio_pub = rospy.Publisher('/audio/onset_notes', NoteOnsetMsg, queue_size=10)

	rate = rospy.Rate(10)
	n = 0
	while not rospy.is_shutdown():
		msgs_audio, msgs_vision = gererate_sequence_msg(seq,n)
		rospy.sleep(2)
		vision_pub.publish(msgs_vision)
		rospy.logdebug('publish vision msg')
		for each in msgs_audio:
			audio_pub.publish(each)
			rate.sleep()
		rospy.sleep(3)
		n += 1

if __name__ == '__main__':
	rospy.init_node('send_hit_sequnce', log_level=rospy.DEBUG)
	send_hit_sequnce4test()
	# rospy.spin()
