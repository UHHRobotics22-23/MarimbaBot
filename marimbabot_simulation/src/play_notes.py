# Plays sounds for the notes published to sim_notes

#!/usr/bin/env python

import os
import subprocess

import librosa
import rospy
import sounddevice as sd
from std_msgs.msg import String

NOTES = [
  "a4",
  "a5",
  "a6",
  "ais4",
  "ais5",
  "ais6",
  "b4",
  "b5",
  "b6",
  "c4",
  "c5",
  "c6",
  "c7",
  "cis4",
  "cis5",
  "cis6",
  "d4",
  "d5",
  "d6",
  "dis4",
  "dis5",
  "dis6",
  "e4",
  "e5",
  "e6",
  "f4",
  "f5",
  "f6",
  "fis4",
  "fis5",
  "fis6",
  "g4",
  "g5",
  "g6",
  "gis4",
  "gis5",
  "gis6",
]


def find_file_directory(filename):
    home_directory = os.path.expanduser("~")
    command = f"find {home_directory} -type f -name {filename} -exec dirname {{}} \\; -quit 2>/dev/null"
    try:
        output = subprocess.check_output(command, shell=True, universal_newlines=True)
        directory = output.strip()
        if directory:
            return directory
        else:
            return None
    except subprocess.CalledProcessError:
        return None


audio_dir = find_file_directory("note_ais4.wav")
print(f"Found audio dir: {audio_dir}", flush=True)


note2audio = {}
for note in NOTES:
    fp = f"{audio_dir}/note_{note}.wav"
    try:
        note2audio[note] = librosa.load(fp, sr=None)
    except:
        note2audio[note] = librosa.load(f"{audio_dir}/note_a4.wav", sr=None)
print("Loaded notes")


def note_callback(message):
    note = str(message.data).strip().lower()
    print(f"Playing note: {note}")
    sd.play(*note2audio[note])


def listener():
    rospy.init_node("play_notes")
    rospy.Subscriber("sim_notes", String, callback=note_callback, queue_size=1)
    rospy.spin()


if __name__ == "__main__":
    rospy.loginfo("Starting listener")
    listener()

