from marimbabot_command_recognition.msg import Command as CommandMsg
from marimbabot_command_recognition.msg import Speech as SpeechMsg
import rospy


class CommandExtract:
	def __init__(self):
		self.command_pub = rospy.Publisher('/command', CommandMsg, queue_size=100, tcp_nodelay=True)
		self.speech_sub = rospy.Subscriber('/speech', SpeechMsg, self.speech_callback, queue_size=10, tcp_nodelay=True)

	def speech_callback(self, speech_msg):
		text = speech_msg.speech
		rospy.logdebug(f"receive speech: {text}")
		command_msg = CommandMsg()
		command_msg.command = text
		command_msg.command_id = speech_msg.sentence_id
		self.command_pub.publish(command_msg)

	def extract(self, text:str):
		return text

	def run(self):
		rospy.spin()

if __name__ == '__main__':
	rospy.init_node('command_extract', log_level=rospy.DEBUG)
	command_extract = CommandExtract()
	command_extract.run()