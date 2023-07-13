#!/usr/bin/env python3
"""
A dummy action client to call the motions action server.
Usage:
Run the dummy client with `rosrun marimbabot_planning dummy_client.py`.
"""

import re

import actionlib
import rospy

from marimbabot_msgs.msg import (HitSequenceAction, HitSequenceElement,
                                 HitSequenceGoal)


class DummyMotionClient:
    def __init__(self):
        self.client = actionlib.SimpleActionClient('hit_sequence', HitSequenceAction)
        self.client.wait_for_server()
        self.main_loop()

    def main_loop(self):
        """
        Wait for keyboard input, concatenate the input into a string, and send it to the server once the user presses enter.
        """
        octave = int(input("Enter an octave (e.g. 4): "))
        while not rospy.is_shutdown():
            # get input from user and create goal
            input_str = input("Enter a sequence of hits (e.g. 'C, <C#, D>, C#, A'): ")
            goal = HitSequenceGoal()
            start_time = rospy.Time(0)
            duration = rospy.Duration(0.5)

            # create regex for splitting the notes including only the notes from the scale (C, D, E, F, G, A, B)
            # e.g. 'C, <C#, D>, C#, A' => ['C', '<C#, D>', 'C#', 'A']
            # LIMITED TO A CHORD OF 2 NOTES !!!
            regex_pattern = r'([CDEFGAB][#b]?|[<][CDEFGAB][#b]?,\s?[CDEFGAB][#b]?[>])'
            notes_or_accords = re.findall(regex_pattern, input_str)

            # iterate over the notes and send them to the server
            for note in notes_or_accords:
                start_time += duration
                if note.startswith('<') and note.endswith('>'):
                    # Handle chord
                    chord_notes = note[1:-1].split(',')
                    start_time += duration
                    for chord_note in chord_notes:
                        goal.hit_sequence_elements.append(
                            HitSequenceElement(
                                tone_name=chord_note.strip().lower().replace('#', 'is'),
                                tone_duration=duration,
                                octave=octave,
                                loudness=1.0,
                                start_time=start_time))
                else:
                    # Handle single note
                    goal.hit_sequence_elements.append(
                    HitSequenceElement(
                        tone_name=note.strip().lower().replace('#', 'is'),
                        tone_duration=duration,
                        octave=octave,
                        loudness=1.0,
                        start_time=start_time))
            # send goal to server
            self.client.send_goal(goal)
            self.client.wait_for_result()
            result = self.client.get_result()
            if result.success:
                print("Success!")
            else:
                print(f"Failure: {['None', 'PLANNING_FAILED', 'EXECUTION_FAILED'][result.error_code]}")

if __name__ == '__main__':
    rospy.init_node('dummy_motion_client', anonymous=True)
    DummyMotionClient()
