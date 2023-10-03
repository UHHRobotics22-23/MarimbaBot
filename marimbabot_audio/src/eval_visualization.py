import os

import cv_bridge
import matplotlib
import matplotlib.pyplot as plt
import numpy as np
import pretty_midi
import rospy
from sensor_msgs.msg import Image

from threading import Lock

from marimbabot_msgs.msg import HitSequenceElement
from marimbabot_msgs.msg import SequenceMatchResult as SequenceMatchResultMsg

matplotlib.use('Agg')

class Visualization():
	def __init__(self) -> None:
		self.onset_list = []
		self.cv_bridge = cv_bridge.CvBridge()
		# duration for whole canvas windows
		self.update_freq = rospy.get_param("~update_freq", default=4)
		
		self.pub_eval_img = rospy.Publisher(
			"feedback_img", Image, queue_size=1, tcp_nodelay=True
		)
		self.sub_macth_result = rospy.Subscriber(
			"/audio_node/match_result",
			SequenceMatchResultMsg,
			self.match_result_visualization,
			queue_size=1,
			tcp_nodelay=True,
		)

		self.render_lock = Lock()

	def match_result_visualization(self, msg: SequenceMatchResultMsg):
		note_width = 0.1
		rospy.logdebug(f"match_result_callback: {msg}")
		self.render_lock.acquire()
		fig = plt.figure()
		ax = fig.add_subplot(1, 1, 1)
		ax.clear()

		start_time = msg.hit_sequence_elements[0].start_time - rospy.Duration(1)
		end_time = msg.hit_sequence_elements[-1].start_time + rospy.Duration(1)

		windows_size = (end_time - start_time).to_sec()

		# Format plot
		plt.xlim(0, windows_size)
		plt.ylim(0, 36+1)
		plt.xticks(rotation=45, ha='right')
		plt.yticks(range(0, 37, 3), [pretty_midi.note_number_to_name(x + pretty_midi.note_name_to_number("C4")) for x in range(0, 37, 3)])
		plt.subplots_adjust(bottom=0.30)
		plt.title('Sequence Match Result(red: miss, green: hit, cyan: robot)')
		plt.ylabel('Notes')
		plt.xlabel('Relative time')
		# Add horizontal grid lines
		ax.yaxis.grid(True)

		#CHange the dpi
		plt.rcParams['figure.dpi'] = 300

		# draw the note of the extracted audio feedback note, within the duration of planned note
		for idx, feedback in enumerate(msg.extracted_note_onsets):
			pitch_name = feedback.note
			relative_pitch = pretty_midi.note_name_to_number(pitch_name)-pretty_midi.note_name_to_number("C4")
			relative_time = (feedback.header.stamp-start_time).to_sec()
			if relative_time>0:
				ax.bar(x=relative_time, height=1, width=note_width, bottom=relative_pitch - 0.5, align="edge", color='c')
				#ax.text(relative_time, relative_pitch - 0.5, pitch_name, fontsize=10, color='black', fontweight='bold')

		# draw the planned hit sequence
		hit_element: HitSequenceElement
		for idx, hit_element in enumerate(msg.hit_sequence_elements):
			pitch_name = hit_element.tone_name + str(hit_element.octave)
			relative_pitch = pretty_midi.note_name_to_number(pitch_name)-pretty_midi.note_name_to_number("C4")
			relative_time = (hit_element.start_time - start_time).to_sec()
			matched_id = msg.is_matched_list[idx]
			if matched_id!= -1:
				ax.bar(x=relative_time, height=1, width=note_width, bottom=relative_pitch - 0.5, align="edge", color='green')
			else:
				ax.bar(x=relative_time, height=1, width=note_width, bottom=relative_pitch - 0.5, align="edge", color='red')
			#ax.text(relative_time, relative_pitch - 0.5, pitch_name, fontsize=10, color='black',fontweight='bold')
		

		fig.savefig('/tmp/tmp_vis2.png')  # save the figure, to update the canvas.
		data = np.frombuffer(fig.canvas.tostring_rgb(), dtype=np.uint8)
		fig = data.reshape(fig.canvas.get_width_height()[::-1] + (3,))
		self.pub_eval_img.publish(
			self.cv_bridge.cv2_to_imgmsg(fig[:,:,::-1], "bgr8")
		)

		ax.clear()
		self.render_lock.release()

if __name__ == "__main__":
	rospy.init_node("evalviz")
	Visualization()
	rospy.spin()




