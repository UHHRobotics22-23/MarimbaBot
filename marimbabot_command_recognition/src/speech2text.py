import rospy
import whisper
from utils.file_control import WAVFile
from marimbabot_command_recognition.msg import Speech as SpeechMsg
from marimbabot_command_recognition.msg import TmpFile as TmpFileMsg

class STT:
	def __init__(self):
		# 'tiny.en', 'tiny', 'base.en', 'base', 'small.en', 'small', 'medium.en', 'medium', 'large-v1', 'large-v2', 'large'
		self.model = whisper.load_model('medium.en')
		self.file_controller = WAVFile()
		self.speech_pub = rospy.Publisher('/command_node/speech', SpeechMsg, queue_size=100, tcp_nodelay=True)
		self.tmp_sub = rospy.Subscriber('/command_node/audio_tmp', TmpFileMsg, self.tmp_callback, queue_size=10, tcp_nodelay=True)
		self.recognize_freq = 2  # Hz
		self.recognize_rate = rospy.Rate(self.recognize_freq)

	def tmp_callback(self, tmp_file_msg):
		file_path = tmp_file_msg.file_path
		rospy.logdebug(f"receive tmp file: {file_path}")
		text = self.recognize(file_path)
		rospy.logdebug(f"publish speech: {text}")
		speech_msg = SpeechMsg()
		speech_msg.speech = text
		speech_msg.sentence_id = tmp_file_msg.sentence_id
		speech_msg.is_finished = tmp_file_msg.is_finished
		self.speech_pub.publish(speech_msg)


	def recognize(self, file_path:str):
		# load audio and pad/trim it to fit 30 seconds
		audio = whisper.load_audio(file_path)
		audio = whisper.pad_or_trim(audio)

		# make log-Mel spectrogram and move to the same device as the model
		mel = whisper.log_mel_spectrogram(audio).to(self.model.device)

		# decode the audio
		options = whisper.DecodingOptions(fp16=False, language='en')
		result = whisper.decode(self.model, mel, options)
		return result.text

	def run(self):
		rospy.spin()

if __name__ == '__main__':
	rospy.init_node('speech_recognition', log_level=rospy.INFO)
	speech_recognition = STT()
	speech_recognition.run()