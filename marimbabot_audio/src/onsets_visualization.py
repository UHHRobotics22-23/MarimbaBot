import os

import cv_bridge
import matplotlib
import matplotlib.pyplot as plt
import numpy as np
import pretty_midi
import rospy
from sensor_msgs.msg import Image

from threading import Lock

from marimbabot_msgs.msg import NoteOnset

matplotlib.use('Agg')

class Visualization():
	def __init__(self) -> None:
		self.onset_list = []
		self.cv_bridge = cv_bridge.CvBridge()
		# duration for whole canvas windows
		self.midi_fig_windows_size = rospy.get_param("~canvas_size_t", default=5)
		self.update_freq = rospy.get_param("~update_freq", default=4)

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

		self.render_lock = Lock()

		# Create timer that calls the callback every 0.1 seconds
		self.timer = rospy.Timer(rospy.Duration(1/self.update_freq), self.onset_visualization)

	# plot the image to ros topic
	def onset_visualization(self, _):
		self.render_lock.acquire()
		fig = plt.figure()
		ax = fig.add_subplot(1, 1, 1)
		ax.clear()

		# Format plot
		plt.xlim(0, self.midi_fig_windows_size)
		plt.ylim(0, 36+1)
		plt.xticks(rotation=45, ha='right')
		plt.yticks(range(0, 37, 6), [pretty_midi.note_number_to_name(x + pretty_midi.note_name_to_number("C4")) for x in range(0, 37, 6)])
		plt.subplots_adjust(bottom=0.30)
		plt.title('marimba note over Time')
		plt.ylabel('Notes')
		plt.xlabel('Time')

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


		fig.savefig('/tmp/.tmp_vis.png')  # save the figure, to update the canvas.
		try:
			os.remove('/tmp/.tmp_vis.png')
		except:
			pass
		data = np.frombuffer(fig.canvas.tostring_rgb(), dtype=np.uint8)
		fig = data.reshape(fig.canvas.get_width_height()[::-1] + (3,))
		self.pub_onset_img.publish(
			self.cv_bridge.cv2_to_imgmsg(fig[:,:,::-1], "bgr8")
		)
		ax.clear()
		self.render_lock.release()
		

	def onset_event(self, msg: NoteOnset):
		note = {
			"name": msg.note,
			"time": msg.header.stamp,
			"duration": msg.confidence,
		}
		self.onset_list.append(note)

if __name__ == "__main__":
	rospy.init_node("onsetviz")
	Visualization()
	rospy.spin()




