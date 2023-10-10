# Detects contacts with bars in Gazebo

#!/usr/bin/env python

import os
import subprocess
import datetime
from pathlib import Path

import rospy
import pretty_midi
from sensor_msgs.msg import JointState
from std_msgs.msg import String


def find_file_directory(filename):
    home_directory = os.path.expanduser("~")
    command = f"find {home_directory} -type f -xtype f -name {filename} -exec dirname {{}} \\; -quit 2>/dev/null"
    try:
        output = subprocess.check_output(command, shell=True, universal_newlines=True)
        directory = output.strip()
        if directory:
            return directory
        else:
            return None
    except subprocess.CalledProcessError:
        return None


# velocity threshold to count as a bar hit
VEL_THRESHOLD = 0.005

# time between contacts to register as a separate one
TIME_THRESHOLD = 0.1  # s

# duration of a note (assume fixed for now)
NOTE_DURATION = 0.5  # s

# names of bars we are interested in
# corresponding joints are bar_<note>/joint
# e.g. "bar_a4" <-> "bar_a4/joint"
BAR_JOINT_NAMES = [
    "bar_a4/joint",
    "bar_a5/joint",
    "bar_a6/joint",
    "bar_ais4/joint",
    "bar_ais5/joint",
    "bar_ais6/joint",
    "bar_b4/joint",
    "bar_b5/joint",
    "bar_b6/joint",
    "bar_c4/joint",
    "bar_c5/joint",
    "bar_c6/joint",
    "bar_c7/joint",
    "bar_cis4/joint",
    "bar_cis5/joint",
    "bar_cis6/joint",
    "bar_d4/joint",
    "bar_d5/joint",
    "bar_d6/joint",
    "bar_dis4/joint",
    "bar_dis5/joint",
    "bar_dis6/joint",
    "bar_e4/joint",
    "bar_e5/joint",
    "bar_e6/joint",
    "bar_f4/joint",
    "bar_f5/joint",
    "bar_f6/joint",
    "bar_fis4/joint",
    "bar_fis5/joint",
    "bar_fis6/joint",
    "bar_g4/joint",
    "bar_g5/joint",
    "bar_g6/joint",
    "bar_gis4/joint",
    "bar_gis5/joint",
    "bar_gis6/joint",
]

joint2note = {joint: joint[4 : joint.index("/")] for joint in BAR_JOINT_NAMES}

pub = None
play_start_time = None
last_contact_timestamps = {joint: 0 for joint in BAR_JOINT_NAMES}
# holds tuples (note, start_time)
played_notes = []


def save_midi(notes):
    """Save notes to a MIDI file using pretty_midi."""

    src_dir = find_file_directory("detect_bar_contact.py")
    midi_dir = Path(src_dir).parent / "midi"
    midi_dir.mkdir(exist_ok=True)
    midi_path = midi_dir / f"{datetime.datetime.now().strftime('%Y-%m-%d_%H-%M-%S')}.midi"

    midi = pretty_midi.PrettyMIDI()
    instrument_program = pretty_midi.instrument_name_to_program("Marimba")
    instrument = pretty_midi.Instrument(program=instrument_program)

    for note, start in notes:
        note_number = pretty_midi.note_name_to_number(note.replace("is", "#").upper())
        note = pretty_midi.Note(
            velocity=100,
            pitch=note_number,
            start=start,
            end=start + NOTE_DURATION,
        )
        instrument.notes.append(note)

    midi.instruments.append(instrument)
    midi.write(str(midi_path.resolve()))
    rospy.loginfo(f"Saved MIDI file to {midi_path}")


def joint_states_callback(message):
    for i, name in enumerate(message.name):
        if name not in joint2note:
            continue

        # ignore joint if it has an ongoing contact
        if message.header.stamp.secs - last_contact_timestamps[name] < TIME_THRESHOLD:
            continue

        vel = message.velocity[i]
        if vel > VEL_THRESHOLD:
            global play_start_time
            if play_start_time is None:
                play_start_time = message.header.stamp.secs
            rospy.loginfo(f"Detected contact with {joint2note[name]} vel={vel:.5f}")
            pub.publish(joint2note[name])
            last_contact_timestamps[name] = message.header.stamp.secs
            played_notes.append((joint2note[name], message.header.stamp.secs - play_start_time))


def listener():
    rospy.init_node("detect_bar_contact")
    global pub
    pub = rospy.Publisher("sim_notes", String, queue_size=1)
    rospy.Subscriber("joint_states", JointState, joint_states_callback, queue_size=1)
    rospy.spin()


if __name__ == "__main__":
    rospy.on_shutdown(lambda: save_midi(played_notes))
    listener()
