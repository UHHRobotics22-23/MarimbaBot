# This node is a dummy node that will be used to test the behavior tree
# It emulates voice commands and publishes to the behavior node
# It emulates vision information and publishes to the behavior node

import rospy
from std_msgs.msg import String


class DummyBehaviorNode:
    def __init__(self):
        self.command_pub = rospy.Publisher('speech_node/command', String, queue_size=10)
        self.recognized_notes_pub = rospy.Publisher('vision_node/recognized_notes', String, queue_size=10)
        self.main_loop()

    def main_loop(self):
        """
        Wait for keyboard input, concatenate the input into a string, and send it to the server once the user presses enter.
        """
        while not rospy.is_shutdown():
            # get input from user and create goal
            input_str = input("Enter a command (e.g. 'marimbabot read'): ")
            recognized_notes_str = input("Enter recognized notes (default 'c4 d4 e4'): ", )
            recognized_notes_str = recognized_notes_str or 'c4 d4 e4'

            self.command_pub.publish(input_str)
            self.recognized_notes_pub.publish(recognized_notes_str)