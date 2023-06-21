import rospy
import actionlib
from marimbabot_msgs.msg import LilypondAudioAction, LilypondAudioGoal
rospy.publish('lilypond_to_audio', self.sentence)
# send lilypond string to action server 'lilypond_audio_generation'
client = actionlib.SimpleActionClient('lilypond_audio_generation', LilypondAudioAction)

# Waits until the action server has started up and started
# listening for goals.
client.wait_for_server()

# Creates a goal to send to the action server.
goal = LilypondAudioGoal(lilypond_string="c4 e g2")

# Sends the goal to the action server.
client.send_goal(goal)

# Waits for the server to finish performing the action.
# Includes that the audio file is generated and was played
client.wait_for_result()

# Prints out the result of executing the action
rospy.logdebug(f"Result from audio_from_lilypond action server: {client.get_result()}")
