from marimbabot_speech.msg import Command as CommandMsg
from marimbabot_speech.msg import Speech as SpeechMsg
import rospy
from utils.command import get_commands

class CommandExtract:
	def __init__(self):
		self.command_pub = rospy.Publisher('/speech_node/command', CommandMsg, queue_size=100, tcp_nodelay=True)
		self.speech_sub = rospy.Subscriber('/speech_node/speech', SpeechMsg, self.speech_callback, queue_size=10, tcp_nodelay=True)
		self.command_list = get_commands()
		self.hotword = "marimbabot"

	def hotword_spotting(self, text):
		if self.hotword in text:
			rospy.logdebug(f"hotword spotted")
			return True

	def speech_callback(self, speech_msg):
		if speech_msg.is_finished:
			text = speech_msg.speech.lower()
			rospy.logdebug(f"receive speech: {text}")
			if self.hotword_spotting(text):
				# if command in the same sentence of the spotting keyword, extract it
				command = self.extract(text)
				if command is not None:
					self.pub_command(command)
					rospy.logdebug(f"pub command: {command}")

	def pub_command(self, command):
		command_msg = CommandMsg()
		command_msg.command = command
		command_msg.header.stamp = rospy.Time.now()
		self.command_pub.publish(command_msg)
		rospy.logdebug(f"publish command: {command}")

	def extract(self, text:str):
		for command in self.command_list:
			if command in text:
				return command
		return None

	def run(self):
		rospy.spin()

if __name__ == '__main__':
	rospy.init_node('speech_t2c_node', log_level=rospy.DEBUG)
	command_extract = CommandExtract()
	command_extract.run()