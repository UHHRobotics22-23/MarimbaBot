# Loading a file on disk using PrettyMidi, and show
mid_path = "/home/wang/workspace/master_porject_2023/src/robot_project/marimbabot_audio/playground/out.mid"


import pretty_midi
import matplotlib.pyplot as plt
from librosa import display

def plot_piano_roll(pm, start_pitch, end_pitch, fs=100):
    # Use librosa's specshow function for displaying the piano roll
    display.specshow(pm.get_piano_roll(fs)[start_pitch:end_pitch],
                             hop_length=1, sr=fs, x_axis='time', y_axis='cqt_note',
                             fmin=pretty_midi.note_number_to_hz(start_pitch))

pm = pretty_midi.PrettyMIDI(mid_path)
plt.figure(figsize=(12, 4))
plot_piano_roll(pm, 24, 84)
plt.show()