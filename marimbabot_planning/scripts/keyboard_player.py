#!/usr/bin/env python3

"""
A keyboard controller for testing the marimbabot.
"""

import fcntl
import os
import sys
import termios

import actionlib
import rospy

from marimbabot_msgs.msg import (HitSequenceAction, HitSequenceElement,
                                 HitSequenceGoal)

NOTES = ['C', 'C#', 'D', 'D#', 'E', 'F', 'F#', 'G', 'G#', 'A', 'A#', 'B']

class KeyboardClient:
    def __init__(self):
        self.client = actionlib.SimpleActionClient('hit_sequence', HitSequenceAction)

        self.client.wait_for_server()

        fd = sys.stdin.fileno()

        oldterm = termios.tcgetattr(fd)
        newattr = termios.tcgetattr(fd)
        newattr[3] = newattr[3] & ~termios.ICANON & ~termios.ECHO
        termios.tcsetattr(fd, termios.TCSANOW, newattr)

        oldflags = fcntl.fcntl(fd, fcntl.F_GETFL)
        fcntl.fcntl(fd, fcntl.F_SETFL, oldflags | os.O_NONBLOCK)


        try:
            while 1:
                try:
                    c = sys.stdin.read(1)
                    if c:
                        print("Got character", repr(c))
                        # Check for escape
                        if c == '\x1b':
                            break
                        chars = ['q2w3er5t6z7u', 'ysxdcvgbhnjm']
                        for i, char_set in enumerate(chars):
                            if c in char_set:
                                note = NOTES[char_set.index(c)]
                                self.play_note(note, i + 4)
                except IOError: pass
        finally:
            termios.tcsetattr(fd, termios.TCSAFLUSH, oldterm)
            fcntl.fcntl(fd, fcntl.F_SETFL, oldflags)


    def play_note(self, note: str, octave: int):
        print(f"Playing {note} in octave {octave}")
        goal = HitSequenceGoal()
        goal.hit_sequence_elements.append(
            HitSequenceElement(
                tone_name=note.strip().upper().replace('#', 's'),
                tone_duration=rospy.Duration(0.5),
                octave=octave,
                loudness=1.0,
                start_time=rospy.Time(0)))
        self.client.send_goal(goal)
        self.client.wait_for_result()
        result = self.client.get_result()
        if result.success:
            print("Success!")
        else:
            print(f"Failure: {['None', 'PLANNING_FAILED', 'EXECUTION_FAILED'][result.error_code]}")

if __name__ == '__main__':
    rospy.init_node('keyboard', anonymous=True)
    KeyboardClient()