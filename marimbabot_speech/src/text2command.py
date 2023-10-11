import re

import rospy

from marimbabot_msgs.msg import Command as CommandMsg
from marimbabot_msgs.msg import Speech as SpeechMsg


def examples_test(command_extractor):
	# examples for testing the command extractor, just temporary put it here as a playground,
	# will be moved to the test folder later
	simple_command_list = [
		"start playing",  # start playing
		"stop playing",  # stop playing
		"play",  # start playing
		"stop",  # stop playing
		"play slower",  # play slower by 20 bpm
		"play faster",  # play faster by 20 bpm
		"play louder",  # play louder by (?)%
		"play softer",  # play softer by (?)%
		"play in loop",  # play in loop
		"preview in loop",  # preview in loop
		"read",  # read the song from camera
	]
	complicated_command_list = [
		"play in 120 bpm",
		"play faster by 60 bpm",
		"play slower by 40 bpm",
		"play louder by 2 steps",
		"play softer by 4 steps",
		"preview in 120 bpm",
		"preview faster by 20 bpm",
		"preview slower by 30 bpm",
		"preview louder by 2 steps",
		"preview softer by 4 steps",
	]
	command_list = simple_command_list + complicated_command_list
	for idx, command_text in enumerate(command_list):
		command_dict = command_extractor.extract(command_text)
		command_extractor.publish_command(command_dict)
		print(f" the {idx}th command: {command_dict}")


class CommandExtraction():
	"""
	Extract command from text, and publish it to the command topic.
	"""
	def __init__(self):
		self.template = {
			"behavior"  : "",
			"action"    : "",
			"parameters": ""
		}
		self.independent_behavior_list = [
			"stop",
			"read",
		]
		#"loop", # repeat the song for (?) times (default infinite)
		self.dependent_behavior_list = [
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
		self.no_speech_prob_threshold = rospy.get_param('~no_speech_prob_threshold', 0.5)
	def fill_into_command(self, behavior="", action="", parameters=""):
		if behavior != "":
			self.template["behavior"] = behavior
		if action != "":
			self.template["action"] = action
		if parameters != "":
			self.template["parameters"] = parameters

	def speech_callback(self, req):
		text = req.text
		if req.no_speech_prob > self.no_speech_prob_threshold:
			rospy.logwarn(f"whisper's no_speech_prob > {self.no_speech_prob_threshold}, even is passed the wad tool, so abandon this text: {text}")
			return
		command = self.extract(text)
		rospy.logdebug(f"command extracted: {command}")
		self.publish_command(command)

	def publish_command(self, command):
		# command_str = json.dumps(command)
		msg = CommandMsg()
		msg.behavior = command["behavior"]
		msg.action = command["action"]
		msg.parameter = command["parameters"]
		msg.header.stamp = rospy.Time.now()
		self.command_pub.publish(msg)


	def extract_bpm(self,input_text):
		pattern = re.compile(r" ([0-9]+) bpm", re.IGNORECASE)
		result = pattern.findall(input_text)
		if len(result) > 0:
			return result[0]
		else:
			return ""

	def extract_step(self, input_text):
		pattern = re.compile(r" ([0-9]+) step", re.IGNORECASE)
		result = pattern.findall(input_text)
		if len(result) > 0:
			return result[0]
		else:
			return ""

	def reset(self):
		self.template = {
			"behavior"  : "",
			"action"    : "",
			"parameters": ""
		}

	def extract(self, text: str):
		self.reset()
		text = text.lower()
		text = text.replace("plays", "play")
		text = text.replace("starts", "start")

		# one of those keywords occurs means the command is clear enough, there is no ambiguous combination with other words,
		# and it can be to be extracted.
		for behavior in self.independent_behavior_list:
			if behavior in text:
				self.fill_into_command(behavior=behavior)
				return self.template

		if "loop" in text:
			if "play" in text:
				self.fill_into_command(behavior="play", action="loop")
				return self.template
			elif "preview" in text:
				self.fill_into_command(behavior="preview", action="loop")
				return self.template
			
		for behavior in ["play", "preview"]:
			if behavior in text:
				if "in" in text:
					if "bpm" in text:
						bpm = self.extract_bpm(text)
						self.fill_into_command(behavior=behavior, action="setup speed", parameters=str(bpm))
						return self.template
					elif "step" in text:
						step = self.extract_step(text)
						self.fill_into_command(behavior=behavior, action="setup volume", parameters=str(step))
						return self.template

				for word in self.speed_setup_list:
					if word in text:
						bpm = self.extract_bpm(text)
						if word == "faster":
							self.fill_into_command(behavior=behavior, action="increase speed", parameters=str(bpm))
							return self.template
						elif word == "slower":
							self.fill_into_command(behavior=behavior, action="decrease speed", parameters=str(bpm))
							return self.template

				for word in self.volume_setup_list:
					if word in text:
						step = self.extract_step(text)
						if word == "louder":
							self.fill_into_command(behavior=behavior, action="increase volume", parameters=str(step))
							return self.template
						elif word == "softer":
							self.fill_into_command(behavior=behavior, action="decrease volume", parameters=str(step))
							return self.template

				# if only "play" appears, that means start playing
				self.fill_into_command(behavior=behavior)
				return self.template
		return self.template

if __name__ == '__main__':
	rospy.init_node('command_extractor',log_level=rospy.DEBUG)
	command_extractor = CommandExtraction()
	rospy.spin()
