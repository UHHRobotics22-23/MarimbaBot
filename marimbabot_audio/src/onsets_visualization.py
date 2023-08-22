import rospy
from marimbabot_msgs.msg import NoteOnset
from sensor_msgs.msg import Image
from marimbabot_msgs.msg import SequenceMatchResult as SequenceMatchResultMsg
import cv_bridge
import numpy as np
import matplotlib
import matplotlib.pyplot as plt
import pretty_midi
import os

matplotlib.use('Agg')

class Visualization():
	def __init__(self) -> None:
		self.onset_list = []
		self.cv_bridge = cv_bridge.CvBridge()
		# duration for whole canvas windows
		self.midi_fig_windows_size = rospy.get_param("~canvas_size_t", default=5)
		self.update_freq = rospy.get_param("~update_freq", default=4)
		self.fig = plt.figure()
		self.ax = self.fig.add_subplot(1, 1, 1)
		self.midi_fig = None
		self.sub = rospy.Subscriber(
			"/audio_node/onset_notes",
			NoteOnset,
			self.onset_event,
			queue_size=100,
			tcp_nodelay=True,
		)
		self.pub_onset_img = rospy.Publisher(
			"live_midi_img", Image, queue_size=1, tcp_nodelay=True
		)
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

	def run(self):
		rate = rospy.Rate(self.update_freq)
		while not rospy.is_shutdown():
			self.onset_visualization()
			rate.sleep()
		rospy.spin()

	# plot the image to ros topic
	def onset_visualization(self):
		fig = plt.figure()
		ax = fig.add_subplot(1, 1, 1)

		# remove the note outside of time windows
		now = rospy.Time.now()
		start_time_in_fig = now - rospy.Duration(self.midi_fig_windows_size)
		onsets_in_windows = []
		for note in self.onset_list:
			if note["time"] > start_time_in_fig:
				onsets_in_windows.append(note)
		self.onset_list = onsets_in_windows

		pitchs = []
		note_names = []
		for idx, note in enumerate(onsets_in_windows):
			relative_time = (note["time"] - start_time_in_fig).to_sec()
			relative_pitch = pretty_midi.note_name_to_number(note["name"])-pretty_midi.note_name_to_number("C4")
			ax.bar(x=relative_time, height=1, width=0.5, bottom=relative_pitch - 0.5, align="edge", color='green')
			ax.text(relative_time, relative_pitch - 0.5, note["name"], fontsize=14, color='black', fontweight='bold')
			pitchs.append(relative_pitch)
			note_names.append(note["name"])
		
		if len(onsets_in_windows) > 0:
			rospy.logdebug(f"onset_visualization: {onsets_in_windows}")

		# Format plot
		plt.xlim(0, self.midi_fig_windows_size)
		plt.ylim(0, 36+1)
		plt.xticks(rotation=45, ha='right')
		plt.yticks(pitchs, note_names)
		plt.subplots_adjust(bottom=0.30)
		plt.title('marimba note over Time')
		plt.ylabel('Notes')
		plt.xlabel('Time')

		fig.savefig('/tmp/.tmp_vis.png')  # save the figure, to update the canvas.
		try:
			os.remove('/tmp/.tmp_vis.png')
		except:
			pass
		data = np.frombuffer(fig.canvas.tostring_rgb(), dtype=np.uint8)
		fig = data.reshape(fig.canvas.get_width_height()[::-1] + (3,))
		self.pub_onset_img.publish(
			self.cv_bridge.cv2_to_imgmsg(fig, "bgr8")
		)
		ax.clear()
		

	def onset_event(self, msg: NoteOnset):
		note = {
			"name": msg.note,
			"time": msg.header.stamp,
			"duration": msg.confidence,
		}
		rospy.logdebug(f"onset_event: {note}")
		self.onset_list.append(note)

	def match_result_visualization(self, msg: SequenceMatchResultMsg):
		note_width = 0.1
		rospy.logdebug(f"match_result_callback: {msg}")
		fig = plt.figure()
		ax = fig.add_subplot(1, 1, 1)
		ax.clear()
		start_time = msg.hit_sequence_elements[0].header.stamp - rospy.Duration(1)
		end_time = msg.hit_sequence_elements[-1].header.stamp + rospy.Duration(1)

		windows_size = (end_time - start_time).to_sec()
		# draw the planned hit sequence
		for idx, hit_element in enumerate(msg.hit_sequence_elements):
			pitch_name = hit_element.tone_name + str(hit_element.octave)
			relative_pitch = pretty_midi.note_name_to_number(pitch_name)-pretty_midi.note_name_to_number("C4")
			relative_time = (hit_element.header.stamp - start_time).to_sec()
			matched_id = hit_element.is_matched_list[idx]
			if matched_id!= -1:
				ax.bar(x=relative_time, height=1, width=note_width, bottom=relative_pitch - 0.5, align="edge", color='green')
			else:
				ax.bar(x=relative_time, height=1, width=note_width, bottom=relative_pitch - 0.5, align="edge", color='red')
			ax.text(relative_time, relative_pitch - 0.5, pitch_name, fontsize=14, color='black',fontweight='bold')
		# draw the note of the extracted audio feedback note, within the duration of planned note
		for idx, feedback in enumerate(msg.extracted_note_onsets):
			pitch_name = feedback.note
			relative_pitch = pretty_midi.note_name_to_number(pitch_name)-pretty_midi.note_name_to_number("C4")
			relative_time = (feedback.header.stamp-start_time).to_sec()
			if relative_time>0:
				ax.bar(x=relative_time, height=1, width=note_width, bottom=relative_pitch - 0.5, align="edge", color='c')
				ax.text(relative_time, relative_pitch - 0.5, pitch_name, fontsize=14, color='black', fontweight='bold')

		# Format plot
		plt.xlim(0, windows_size)
		plt.ylim(0, 36+1)
		plt.xticks(rotation=45, ha='right')
		# plt.yticks(relative_pitches, pitch_names)
		plt.subplots_adjust(bottom=0.30)
		plt.title('Sequence Match Result(red: miss, green: hit, cyan: robot)')
		plt.ylabel('Notes')
		plt.xlabel('Relative time')
		fig.savefig('/tmp/tmp_vis2.png')  # save the figure, to update the canvas.
		data = np.frombuffer(fig.canvas.tostring_rgb(), dtype=np.uint8)
		fig = data.reshape(fig.canvas.get_width_height()[::-1] + (3,))
		self.pub_eval_img.publish(
			self.cv_bridge.cv2_to_imgmsg(fig, "bgr8")
		)
		ax.clear()


if __name__ == "__main__":
	rospy.init_node("onset2midi",log_level=rospy.DEBUG)
	real_time_vis = Visualization()
	real_time_vis.run()




