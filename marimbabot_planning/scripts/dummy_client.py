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
        octave = int(input("Enter an octave (e.g. 4): "))
        while not rospy.is_shutdown():
            input_str = input("Enter a sequence of hits (e.g. 'C, C#, A'): ")
            goal = HitSequenceGoal()
            start_time = rospy.Time(0)
            duration = rospy.Duration(0.5)
            for note in input_str.split(','):
                start_time += duration
                goal.hit_sequence_elements.append(
                    HitSequenceElement(
                        tone_name=note.strip().upper().replace('#', 's'),
                        tone_duration=duration,
                        octave=octave,
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
    rospy.init_node('dummy_motion_client')
    DummyMotionClient()