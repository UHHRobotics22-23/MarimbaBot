import librosa
import pretty_midi
from librosa import display
from matplotlib import pyplot as plt
import numpy as np

def manual_defined_midi():
	# find the program id for marimba
	instrument_name = "Marimba"
	program_id = pretty_midi.instrument_name_to_program(instrument_name)

	# add the instrument into the midi
	pm = pretty_midi.PrettyMIDI(initial_tempo=80)
	inst = pretty_midi.Instrument(program=program_id, is_drum=False, name='tams marimba')
	pm.instruments.append(inst)

	# Let's add a few notes to our instrument
	velocity = 100
	for pitch, start, end in zip([60, 62, 64, 67, 69, 73, 75], [0.2, 0.6, 1.0, 1.5, 2, 2.5, 3],
	                             [1.1, 1.7, 2.3, 2, 2.5, 3, 3.5]):
		inst.notes.append(pretty_midi.Note(velocity, pitch, start, end))

	# TODO: should we use pitch bend.
	# A pitch bend is a music effect on where one note will slide to another note.

	# We'll just do a 1-semitone pitch ramp up
	# n_steps = 512
	# bend_range = 8192//2
	# for time, pitch in zip(np.linspace(1.5, 2.3, n_steps),
	#                        range(0, bend_range, bend_range//n_steps)):
	#     inst.pitch_bends.append(pretty_midi.PitchBend(pitch, time))
	pm.write('./out.mid')







def plot_marimba_roll(pm, fs=22520):
	# Use librosa's specshow function for displaying the piano roll
	all_pitchs = []
	for instr in pm.instruments:
		for note in instr.notes:
			all_pitchs.append(note.pitch)
	start_pitch = min(all_pitchs)
	end_pitch = max(all_pitchs) + 1

	display.specshow(pm.get_piano_roll(fs)[start_pitch:end_pitch],
	                 hop_length=1, sr=fs, x_axis='time', y_axis='cqt_note',
	                 fmin=pretty_midi.note_number_to_hz(start_pitch))


# see: https://www.daniweb.com/programming/software-development/code/216976/play-a-midi-music-file-using-pygame

# sudo pip install pygame

# on ubuntu
# sudo apt-get install python-pygame

import pygame
def _play_music(music_file):
	"""
    stream music with mixer.music module in blocking manner
    this will stream the sound from disk while playing
    """
	clock = pygame.time.Clock()
	try:
		pygame.mixer.music.load(music_file)
		print("Music file %s loaded!" % music_file)
	except pygame.error:
		print("File %s not found! (%s)" % (music_file, pygame.get_error()))
		return
	pygame.mixer.music.play()
	while pygame.mixer.music.get_busy():
		# check if playback has finished
		clock.tick(30)


# pick a midi music file you have ...
# (if not in working folder use full path)
def play_midi(midi_file):
	freq = 22520  # audio CD quality
	bitsize = -16  # unsigned 16 bit
	channels = 2  # 1 is mono, 2 is stereo
	buffer = 2048  # number of samples
	pygame.mixer.init(freq, bitsize, channels, buffer)

	# optional volume 0 to 1.0
	pygame.mixer.music.set_volume(0.8)
	try:
		_play_music(midi_file)
	except KeyboardInterrupt:
		# if user hits Ctrl/C then exit
		# (works only in console mode)
		pygame.mixer.music.fadeout(1000)
		pygame.mixer.music.stop()
		raise SystemExit



if __name__ == '__main__':
	manual_defined_midi()
	# all_freq = []
	# for pitch in ['C','D','E','F','G']:
	# 	for octive in ['4','5','6']:
	# 		note_name = pitch+octive
	# 		freq = pretty_midi.note_number_to_hz(pretty_midi.note_name_to_number(note_name))
	# 		print(f"name:{note_name}  freq:{freq}")
	# 		all_freq.append(freq)
	# librosa.cqt()
	# midi_file = '/home/wang/workspace/sound2midi/src/music_perception/playground/out.mid'
	# pm = pretty_midi.PrettyMIDI(midi_file)
	# pm.instruments[0].program = pretty_midi.instrument_name_to_program("Marimba")
	# plt.figure(figsize=(8, 4))
	# plot_marimba_roll(pm)
	# plt.show()
	# play_midi(midi_file)


	# for n in range(100):
	# 	name = pretty_midi.note_number_to_name(note_number=n)
	# 	freq = pretty_midi.note_number_to_hz(n)
	# 	print(f"id:{n}, name:{name}  hz:{freq}")