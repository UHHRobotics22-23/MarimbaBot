#!/usr/bin/env python3
"""
A dummy action client to call the motions action server.
"""

import rospy
import actionlib
from marimbabot_msgs.msg import HitSequenceAction, HitSequenceGoal, HitSequenceElement


class DummyMotionClient:
    def __init__(self):
        self.client = actionlib.SimpleActionClient('hit_sequence', HitSequenceAction)
        self.client.wait_for_server()
        self.main_loop()

    def main_loop(self):
        """
        Wait for keyboard input, concatenate the input into a string, and send it to the server once the user presses enter.
        """
        octave = int(input("Enter an octave (e.g. 2, 4): "))
        loudness = float(input("Enter a loudness (e.g. 0.2, 1.0): "))
        duration = float((input("Enter a duration (e.g. 0.5 for a quarter note and 120 bpm): ")))
        tempo = float(input("Enter a tempo (e.g. 60, 120): "))

        while not rospy.is_shutdown():
            input_str = input("Enter a sequence of hits with corresponding duration (e.g. 'C2-0.5, C-0.8, A4-0.2'): ")
            goal = HitSequenceGoal()
            start_time = rospy.Time(0)
            #duration = rospy.Duration(0.5)
            for note in input_str.split(','):
                full_tone= note.strip().upper().replace('#', 's')
                tone_name = full_tone.split('-')[0]
                dur = full_tone.split('-')[1]
                duration = rospy.Duration(60/tempo*float(dur))
                start_time += duration
                print("Tone name: ", tone_name, "Duration: ", duration, "Start time: ", start_time)
                goal.hit_sequence_elements.append(
                    HitSequenceElement(
                        tone_name=tone_name,
                        tone_duration=duration,
                        octave=octave,
                        loudness=loudness,
                        start_time=start_time))
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