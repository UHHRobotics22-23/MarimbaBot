from marimbabot_command_recognition.msg import Command as CommandMsg
from marimbabot_command_recognition.msg import Speech as SpeechMsg
import rospy

class CommandExtract:
	def __init__(self):
		self.command_pub = rospy.Publisher('/command', CommandMsg, queue_size=100, tcp_nodelay=True)
		self.speech_sub = rospy.Subscriber('/speech', SpeechMsg, self.speech_callback, queue_size=10, tcp_nodelay=True)
		self.command_id = 0
		self.waiting_for_command = False
		self.command_list = [
			"read",
			"play",
			"stop",
			"repeat",
			"loop",
			"concatenate",
			"slower",
			"faster",
			"louder",
			"softer"
		]


	def say(self,text):
		rospy.loginfo(text)
		# TODO: implement this function to let robot say "Hello, I am here".

	def keyword_spotting(self,text):
		if "hello" not in text:
			return False
		possible_kw_detection = [
			"marimba",
			"marinda",
			"marumba",
			"my rainbow",
			"my rain bow",
			"mernba",
		]
		for kw in possible_kw_detection:
			if kw in text:
				return True

	def speech_callback(self, speech_msg):
		if speech_msg.is_finished:
			text = speech_msg.speech.lower()
			rospy.logdebug(f"receive speech: {text}")
			if self.keyword_spotting(text) or self.waiting_for_command:
				rospy.logdebug("keyword is spotted")
				# if command in t he same sentence of the spotting keyword, extract it
				command = self.extract(text)
				if command is not None:
					self.say(f"ok,I will {command}")
					self.waiting_for_command = False
					self.pub_command(command)
					rospy.logdebug(f"send command: {command}")
				# else, detect the command in the next sentence
				else:
					rospy.logdebug("no command is extracted")
					if self.waiting_for_command:
						self.say('sorry, I cannot understand you, please say again')
					else:
						self.say('what can I do for you?')
						self.waiting_for_command = True


	def pub_command(self, command):
		command_msg = CommandMsg()
		command_msg.command = command
		command_msg.command_id = self.command_id
		self.command_pub.publish(command_msg)
		self.command_id += 1

	def extract(self, text:str):
		for keyword in self.command_list:
			if keyword in text:
				return keyword
		return None

	def run(self):
		rospy.spin()

if __name__ == '__main__':
	rospy.init_node('command_extract', log_level=rospy.INFO)
	command_extract = CommandExtract()
	command_extract.run()