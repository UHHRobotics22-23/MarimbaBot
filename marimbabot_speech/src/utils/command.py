import rospy

def get_commands():
	file_path = rospy.get_param('/speech_stt_node/commands_path')
	with open(file_path, 'r+') as f:
		command_lines = f.readlines()[1:]
	return [line.lower().strip() for line in command_lines]