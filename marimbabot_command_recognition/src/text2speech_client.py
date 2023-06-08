from marimbabot_command_recognition.srv import Speak, SpeakResponse
import rospy

# only for show how to call the service of text2speech service
if __name__ == '__main__':
	text = "hello, nice to see you. It's a good day."
	rospy.wait_for_service('/speech_node/tts')
	try:
		speak = rospy.ServiceProxy('/speech_node/tts', Speak)
		resp1 = speak(text)
		print(resp1)
	except rospy.ServiceException as e:
		print(f"Service call failed: {e}")