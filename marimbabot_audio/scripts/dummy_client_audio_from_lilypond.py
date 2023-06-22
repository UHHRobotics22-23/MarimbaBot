import rospy
import actionlib
from marimbabot_msgs.msg import (LilypondAudioAction, LilypondAudioGoal)
from std_msgs.msg import String


def run_dummy():
    sentence = "c4 d4 e4"
    # send lilypond string to action server 'lilypond_audio_generation'
    client = actionlib.SimpleActionClient('audio_from_lilypond', LilypondAudioAction)
    # Waits until the action server has started up and started
    # listening for goals.
    client.wait_for_server()

    # Creates a goal to send to the action server.
    goal = LilypondAudioGoal(lilypond_string=String(data=sentence))
    # Sends the goal to the action server.
    client.send_goal(goal)

    # Waits for the server to finish performing the action.
    # Includes that the audio file is generated and was played
    client.wait_for_result()

    # Prints out the result of executing the action
    print(f"Result from audio_from_lilypond action server: {client.get_result()}")


if __name__ == '__main__':
    rospy.init_node('dummy_motion_client', anonymous=True)
    run_dummy()
