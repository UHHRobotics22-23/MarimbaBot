import os
import datetime
import rospy
from marimbabot_msgs.msg import NoteOnset, CQTStamped
from sensor_msgs.msg import Image
import cv_bridge
import numpy as np
import matplotlib
import matplotlib.pyplot as plt
import pretty_midi

matplotlib.use('Agg')


class Visualization():
	def __init__(self) -> None:
		self.onset_list = []
		self.plot_windows = 10  # sec

		self.cv_bridge = cv_bridge.CvBridge()

		self.midi = None
		self.time_first = 0  # recorder

		self.midi_fig_update_duration = 5  # 0.5 sec duration for each update
		self.time_last_draw = 0  # a recorder
		self.midi_fig_windows_size = 10  # 2 sec duration for whole windows

		self.windows_start_time = 0  # record the start time of each figure windows
		self.windows_end_time = 0  # record the end time of each figure windows

		self.reset()
		self.cv_bridge = cv_bridge.CvBridge()
		self.sub = rospy.Subscriber(
			"/audio/onset_notes",
			NoteOnset,
			self.onset_event,
			queue_size=500,
			tcp_nodelay=True,
		)
		self.pub_midi = rospy.Publisher(
			"/audio/midi_img", Image, queue_size=1, tcp_nodelay=True
		)
		self.fig = plt.figure()
		self.ax = self.fig.add_subplot(1, 1, 1)
		self.midi_fig = None

	def run(self):
		rate = rospy.Rate(4)
		while not rospy.is_shutdown():
			self.plot_to_img()
			rate.sleep()
		rospy.spin()

	# plot the image to ros topic
	def plot_to_img(self):
		assert self.fig is not None
		assert self.ax is not None
		self.ax.clear()

		# remove the note outside of time windows
		now = rospy.Time.now()
		start_time_in_fig = now - rospy.Duration(self.midi_fig_windows_size)
		onsets_in_windows = []
		for note in self.onset_list:
			if note["time"] > start_time_in_fig:
				onsets_in_windows.append(note)
		self.onset_list = onsets_in_windows

		if len(onsets_in_windows) > 0:
			rospy.logdebug(f"draw {onsets_in_windows}")

		pitchs = []
		note_names = []
		for idx, note in enumerate(onsets_in_windows):
			relative_time = (note["time"] - start_time_in_fig).to_sec()
			relative_pitch = pretty_midi.note_name_to_number(note["name"])-pretty_midi.note_name_to_number("C4")
			self.ax.bar(x=relative_time, height=1, width=note["duration"], bottom=relative_pitch - 0.5, align="edge", color='black')
			pitchs.append(relative_pitch)
			note_names.append(note["name"])

		# Format plot
		plt.xlim(0, self.midi_fig_windows_size)
		plt.ylim(0, 37)
		plt.xticks(rotation=45, ha='right')
		plt.yticks(pitchs, note_names)
		plt.subplots_adjust(bottom=0.30)
		plt.title('marimba note over Time')
		plt.ylabel('Notes')
		plt.xlabel('Time')

		self.fig.savefig('/tmp/tmp.png')  # save the figure, to update the canvas.
		data = np.frombuffer(self.fig.canvas.tostring_rgb(), dtype=np.uint8)
		data = data.reshape(self.fig.canvas.get_width_height()[::-1] + (3,))
		self.midi_fig = data
		self.pub_midi.publish(
			self.cv_bridge.cv2_to_imgmsg(self.midi_fig, "bgr8")
		)


	def reset(self):
		self.onset_list = []

	def onset_event(self, msg: NoteOnset):
		note = {
			"name": msg.note,
			"time": msg.header.stamp,
			"duration": msg.confidence,
		}
		rospy.logdebug(f"onset_event: {note}")
		self.onset_list.append(note)

if __name__ == "__main__":
	rospy.init_node("onset2midi",log_level=rospy.DEBUG)
	visualizer = Visualization()
	visualizer.run()




