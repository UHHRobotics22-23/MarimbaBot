import re
import json
import rospy
from marimbabot_msgs.msg import Command as CommandMsg
from marimbabot_msgs.msg import Speech as SpeechMsg

def examples_test():
	# examples for testing the command extractor, just temporary put it here as a playground,
	# will be moved to the test folder later
	simple_command_list = [
		"start playing",  # start playing
		"stop playing",  # stop playing
		"play",  # start playing
		"stop",  # stop playing
		"start",  # start playing
		"play slower",  # play slower by 20 bpm
		"play faster",  # play faster by 20 bpm
		"play louder",  # play louder by (?)%
		"play softer",  # play softer by (?)%
		"repeat",  # repeat the song for (?) times
		"loop",  # repeat the song for (?) times
		"read",  # read the song from camera
		"concatenate",  # concatenate the song with the previous one
		"save as test song",  # read the song and save it as test song
		"load test song",  # load the test song and play it
	]
	complicated_command_list = [
		"play in 120 bpm",
		"play in forte",
		"play faster by 60 bpm",
		"play slower by 40 bpm",
		"play slower by 120 bpm",
		"play louder by 20%",
		"play softer by 40%",
		# "preview",
		# "preview slower by 30 bpm",
		# "preview in 80 bpm",
		# "preview in forte",
	]
	command_extractor = CommandExtraction()
	command_list = simple_command_list + complicated_command_list
	for command_text in command_list:
		command_dict = command_extractor.extract(command_text)
		print(f"{command_text}:{command_dict}")

def dummy_clinet_command_subscriber():
	# dummy client for command subscriber, for testing and instruction purpose
	def command_callback(msg):
		command_dict = json.loads(msg.command)
		print(command_dict)
	rospy.init_node('command_subscriber', log_level=rospy.DEBUG)
	rospy.Subscriber('/speech_node/command', CommandMsg, command_callback, queue_size=10, tcp_nodelay=True)
	rospy.spin()


class CommandExtraction():
	"""
	Extract command from text, and publish it to the command topic.
	"""
	def __init__(self):
		self.independent_action_list = [
			"stop",
			"start",
			"repeat",
			"loop",
			"read",
			"concatenate",
			"save",
			"load",
		]
		self.dependent_action_list = [
			"play",
			"preview",
		]
		self.speed_setup_list = [
			"faster",
			"slower",
		]
		self.volume_setup_list = [
			"louder",
			"softer",
		]
		self.command_pub = rospy.Publisher('/speech_node/command', CommandMsg, queue_size=100, tcp_nodelay=True)
		self.speech_sub = rospy.Subscriber('/speech_node/speech', SpeechMsg, self.speech_callback, queue_size=10, tcp_nodelay=True)

	def speech_callback(self, req):
		text = req.text
		if req.no_speech_prob > 0.5:
			rospy.logwarn("no_speech_prob > 0.5, but is passed the wad, something weird happened, ignore this speech")
			return
		command = self.extract(text)
		rospy.logdebug(f"command extracted: {command}")
		command_str = json.dumps(command)
		msg = CommandMsg()
		msg.command = command_str
		msg.header.stamp = rospy.Time.now()
		self.command_pub.publish(msg)


	def extract_bpm(self,input_text):
		pattern = re.compile(r" ([0-9]+) bpm", re.IGNORECASE)
		result = pattern.findall(input_text)
		if len(result) > 0:
			return result[0]
		else:
			return ""

	def extract_percentage(self,input_text):
		pattern = re.compile(r" ([0-9]+)%", re.IGNORECASE)
		result = pattern.findall(input_text)
		if len(result) > 0:
			return result[0]
		else:
			return ""

	def extract(self, text: str):
		text = text.lower()
		text = text.replace("plays", "play")
		text = text.replace("starts", "start")

		# one of those keywords occurs means the command is clear enough, there is no ambiguous combination with other words,
		# and it can be to be extracted.
		for command in self.independent_action_list:
			if command in text:
				return {
					"action" : command,
					"parameters" : ""
				}

		if "play" in text:
			if "in" in text:
				if "bpm" in text:
					bpm = self.extract_bpm(text)
					return {
						"action"    : "setup speed",
						"parameters": bpm
					}
				elif "forte" in text:
					# TODO: consider other words to describe volume if needed
					return {
						"action"    : "setup volume",
						"parameters": "forte"
					}



			for word in self.speed_setup_list:
				if word in text:
					bpm = self.extract_bpm(text)
					if word == "faster":
						return {
							"action"    : "increase speed",
							"parameters": f"{bpm}"
						}
					elif word == "slower":
						return {
							"action"    : "decrease speed",
							"parameters": f"{bpm}"
						}
			for word in self.volume_setup_list:
				if word in text:
					percent = self.extract_percentage(text)
					if word == "louder":
						return {
							"action"    : "increase volume",
							"parameters": f"{percent}"
						}
					elif word == "softer":
						return {
							"action"    : "decrease volume",
							"parameters": f"{percent}"
						}
			# if only "play" appears, that means start playing
			return {
				"action"    : "start",
				"parameters": ""
			}

if __name__ == '__main__':
	rospy.init_node('command_extractor',log_level=rospy.DEBUG)
	command_extractor = CommandExtraction()
	rospy.spin()
