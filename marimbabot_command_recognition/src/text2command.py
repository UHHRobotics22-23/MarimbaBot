from marimbabot_command_recognition.msg import Command as CommandMsg
from marimbabot_command_recognition.msg import Speech as SpeechMsg
from marimbabot_command_recognition.srv import Speak, SpeakResponse
import rospy

class CommandExtract:
	def __init__(self):
		self.command_pub = rospy.Publisher('/command_node/command', CommandMsg, queue_size=100, tcp_nodelay=True)
		self.speech_sub = rospy.Subscriber('/command_node/speech', SpeechMsg, self.speech_callback, queue_size=10, tcp_nodelay=True)
		rospy.wait_for_service('/speech_node/tts')
		self.speaker = rospy.ServiceProxy('/speech_node/tts', Speak)
		self.command_id = 0
		self.waiting_for_command = False
		self.waiting_counter_limit = 3
		self.waiting_counter = 0
		self.time_filter = rospy.Time.now()
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
		rospy.logdebug(f"say: {text}")
		resp = self.speaker(text)
		self.time_filter = rospy.Time.now()
		return resp

	def keyword_spotting(self,text):
		if "hello" not in text and "hi" not in text:
			return False

		possible_kw_detection = [
			"robot",
		]
		for kw in possible_kw_detection:
			if kw in text:
				rospy.logdebug(f"keyword is detected: {kw}")
				return True

	def speech_callback(self, speech_msg):
		this_time = speech_msg.header.stamp
		if this_time < self.time_filter:
			rospy.logdebug("speech is too old")
			return
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
						self.waiting_counter += 1
						if self.waiting_counter >= self.waiting_counter_limit:
							self.waiting_for_command = False
							self.waiting_counter = 0
					else:
						self.say('what can I do for you?')
						self.waiting_for_command = True


	def pub_command(self, command):
		command_msg = CommandMsg()
		command_msg.command = command
		command_msg.command_id = self.command_id
		command_msg.header.stamp = rospy.Time.now()
		self.command_pub.publish(command_msg)
		rospy.logdebug(f"publish command: {command}")
		self.command_id += 1

	def extract(self, text:str):
		for keyword in self.command_list:
			if keyword in text:
				return keyword
		return None

	def run(self):
		rospy.spin()

if __name__ == '__main__':
	rospy.init_node('command_extract', log_level=rospy.DEBUG)
	command_extract = CommandExtract()
	command_extract.run()