#!/usr/bin/env python3

"""
A set of melodies that can be played on the marimbabot for testing purposes.
"""

import actionlib
import rospy
from marimbabot_msgs.msg import (HitSequenceAction, HitSequenceElement,
                                 HitSequenceGoal)

MELODIES = {
    'chromatic_scale':
        [
                      ('C#', 4), ('D', 4), ('D#', 4), ('E', 4), ('F', 4), ('F#', 4), ('G', 4), ('G#', 4), ('A', 4), ('A#', 4), ('B', 4),
            ('C', 5), ('C#', 5), ('D', 5), ('D#', 5), ('E', 5), ('F', 5), ('F#', 5), ('G', 5), ('G#', 5), ('A', 5), ('A#', 5), ('B', 5),
            ('C', 6), ('C#', 6), ('D', 6), ('D#', 6), ('E', 6), ('F', 6), ('F#', 6), ('G', 6), ('G#', 6), ('A', 6), ('A#', 6), ('B', 6),
            ('C', 7)
        ],
    'brother_john':
        [
            # f,g,a,f,f,g,a,f,a,a#,c,a,a#,c
            ('F', 4), ('G', 4), ('A', 4), ('F', 4), ('F', 4), ('G', 4), ('A', 4), ('F', 4), ('A', 4), ('A#', 4), ('C', 5), ('A', 4), ('A#', 4), ('C', 5)
        ],
    'all_my_little_ducklings':
        [
            # c,d,e,f,g,g,a,a,a,a,g,a,a,a,a,g,f,f,f,f,e,e,g,g,g,g,c
            ('C', 4), ('D', 4), ('E', 4), ('F', 4), ('G', 4), ('G', 4), ('A', 4), ('A', 4), ('A', 4), ('A', 4), ('G', 4), ('A', 4), ('A', 4), ('A', 4), ('A', 4), ('G', 4), ('F', 4), ('F', 4), ('F', 4), ('F', 4), ('E', 4), ('E', 4), ('G', 4), ('G', 4), ('G', 4), ('G', 4), ('C', 4)
        ],
    'repeat_low_end':
        [
            ('D', 4), ('D', 4), ('D', 4), ('D', 4), ('D', 4), ('D', 4), ('D', 4), ('D', 4), ('D', 4), ('D', 4), ('D', 4), ('D', 4)
        ],
    'repeat_high_end':
        [
            ('C', 7), ('C', 7), ('C', 7), ('C', 7), ('C', 7), ('C', 7), ('C', 7), ('C', 7), ('C', 7), ('C', 7), ('C', 7), ('C', 7)
        ],
    'feather_pattern':
            # An alternating pattern with two notes where we start in the middle move both notes further apart
        [
            ('F', 5), ('G', 5),
            ('E', 5), ('A', 5),
            ('D', 5), ('B', 5),
            ('C', 5), ('C', 6),
            ('B', 4), ('D', 6),
            ('A', 4), ('E', 6),
            ('G', 4), ('F', 6),
            ('F', 4), ('G', 6),
            ('E', 4), ('A', 6),
            ('D', 4), ('B', 6),
        ],
    'all_white_keys':
        [
            ('D', 4), ('E', 4), ('F', 4), ('G', 4), ('A', 4), ('B', 4), ('C', 5), ('D', 5), ('E', 5), ('F', 5), ('G', 5), ('A', 5), ('B', 5), ('C', 6), ('D', 6), ('E', 6), ('F', 6), ('G', 6), ('A', 6), ('B', 6), ('C', 7)
        ],
    'all_black_keys':
        [
            ('C#', 4), ('D#', 4), ('F#', 4), ('G#', 4), ('A#', 4), ('C#', 5), ('D#', 5), ('F#', 5), ('G#', 5), ('A#', 5), ('C#', 6), ('D#', 6), ('F#', 6), ('G#', 6), ('A#', 6)
        ],
}


class DummyMotionClient:
    def __init__(self):
        self.client = actionlib.SimpleActionClient('hit_sequence', HitSequenceAction)

        self.client.wait_for_server()
        self.main_loop()

    def main_loop(self):
        """
        Wait for keyboard input to select and send a melody to the server.
        """
        while not rospy.is_shutdown():
            print("Available melodies:")
            print("===================")
            for i, melody in enumerate(MELODIES.keys()):
                print(f"{i}: {melody}")
            print("===================")
            user_input = input("Enter a melody index or press q to quit.\nYou can specify an octave offset by appending a signed number to the melody index (e.g. 3+1):")
            if user_input == 'q':
                break
            if '+' in user_input:
                melody_index, octave_offset = user_input.split('+')
                melody_index = int(melody_index)
                octave_offset = int(octave_offset)
            elif '-' in user_input:
                melody_index, octave_offset = user_input.split('-')
                melody_index = int(melody_index)
                octave_offset = -int(octave_offset)
            else:
                melody_index = int(user_input)
                octave_offset = 0
            melody = MELODIES[list(MELODIES.keys())[melody_index]]
            goal = HitSequenceGoal()
            start_time = rospy.Time(0)
            duration = rospy.Duration(0.5)
            for note, octave in melody:
                start_time += duration
                goal.hit_sequence_elements.append(
                    HitSequenceElement(
                        tone_name=note.strip().lower().replace('#', 'is'),
                        tone_duration=duration,
                        octave=octave+octave_offset,
                        loudness=1.0,
                        start_time=start_time))
            self.client.send_goal(goal)
            self.client.wait_for_result()
            result = self.client.get_result()
            if result.success:
                print("Success!")
            else:
                print(f"Failure: {['None', 'PLANNING_FAILED', 'EXECUTION_FAILED'][result.error_code]}")

if __name__ == '__main__':
    rospy.init_node('sound_pad', anonymous=True)
    DummyMotionClient()
