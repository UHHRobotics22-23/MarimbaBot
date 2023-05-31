import rospy
import whisper
from file_control import WAVFile
from marimbabot_command_recognition.msg import Command as CommandMsg
from marimbabot_command_recognition.msg import Speech as SpeechMsg

class SpeechRecognition:
	def __init__(self):
		self.model = whisper.load_model('')
		self.file_controller = WAVFile()
		self.pub = rospy.Publisher('/speech', SpeechMsg, queue_size=10)

	def recognize(self):
		file_path = self.file_controller.get_last_file_path()
		# load audio and pad/trim it to fit 30 seconds
		audio = whisper.load_audio(file_path)
		audio = whisper.pad_or_trim(audio)

		# make log-Mel spectrogram and move to the same device as the model
		mel = whisper.log_mel_spectrogram(audio).to(self.model.device)

		# detect the spoken language
		_, probs = self.model.detect_language(mel)
		print(f"Detected language: {max(probs, key=probs.get)}")

		# decode the audio
		options = whisper.DecodingOptions(language='en')
		result = whisper.decode(self.model, mel, options)
		rospy.logdebug(f"detected text: {result.text}")
		return result.text