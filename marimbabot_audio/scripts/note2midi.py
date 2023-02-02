import rospy
from marimbabot_audio.msg import NoteOnset
import pretty_midi
from librosa import display
import os
import matplotlib.pyplot as plt
import numpy as np
import cv_bridge
from sensor_msgs.msg import Image
from copy import deepcopy
import cv2
from time import time
import matplotlib.animation as animation
import datetime
import matplotlib
# matplotlib.use('agg')
'''
Introduction:


'''

class MyPrettyMiDi():
    def __init__(self,instruments_name_list:list=["Marimba"],file_path:str=None):
        # if file_path existed, then load midi file from current path, else create a new midi file.
        self.file_path = file_path
        assert file_path is not None, "the midi file path can not be None."
        if  os.path.exists(file_path):
                self.pm = pretty_midi.PrettyMIDI(midi_file=file_path)
        else:
            self.instruments_name_list = instruments_name_list
            # init midi file
            self.pm = pretty_midi.PrettyMIDI(initial_tempo=100)
            self.instruments = self._init_instruments()        
    
    def _init_instruments(self):
        for instrument in self.instruments_name_list:
            # get the intrument id from name
            program_id = pretty_midi.instrument_name_to_program(instrument_name=instrument)
            # build instrument from it's id
            intru = pretty_midi.Instrument(program=program_id, is_drum=False, name=instrument)
            self.pm.instruments.append(intru)

    
    def append_note(self, instrument_name:str, onset_note:str, start_time, end_time):
        '''
            Add the note into specified instrument in the midi file.
            instrument_name: str, e.g. "Marimba"
            onset_note: str, e.g. "C#4"
            start_time: float, in second, e.g. 1.3
            end_time: floar, sec, e.g. 1.7
        '''
        note_number = pretty_midi.note_name_to_number(note_name=onset_note)
        pm_note = pretty_midi.Note(velocity=100, pitch=note_number, start=start_time, end=end_time)
        for idx, instrument in enumerate(self.pm.instruments):
            if instrument.name == instrument_name:
                self.pm.instruments[idx].notes.append(pm_note)
                # print(f'add note to marimba, we have {len(instrument.notes)} now.')
    
    def save(self):
        self.pm.write(self.file_path)



class Onset2Midi():
    def __init__(self,file_path:str="/home/wang/workspace/master_porject_2023/midi_files") -> None:
        self.cv_bridge = cv_bridge.CvBridge()
        now = datetime.datetime.now()
        if not os.path.exists(file_path):
            os.mkdir(file_path)
        self.file_path = os.path.join(file_path,f"{now.year}_{now.month}_{now.day}_{now.hour}_{now.minute}.mid")
        self.midi = None
        # self.midi_save_duration = 5*60 #  save the mid file per 5 minutes
        # self.midi_last_save_time = 0  # reocorder for save the mid files
        self.time_first = 0  # recorder

        self.midi_fig_update_duration = 5  # 0.5 sec duration for each update
        self.time_last_draw = 0  # a recorder
        self.midi_fig_windows_size = 10  # 2 sec duration for whole windows

        self.windows_start_time = 0  # record the start time of each figure windows
        self.windows_end_time = 0  # record the end time of each figure windows


        self.reset()
        self.sub = rospy.Subscriber(
            "/onsets",
            NoteOnset,
            self.onset_event,
            queue_size=500,
            tcp_nodelay=True,
        )

        self.pub_midi = rospy.Publisher(
            "midi", Image, queue_size=1, tcp_nodelay=True
        )
        self._init_plot()

    def reset(self):
        if os.path.exists(self.file_path):
            os.remove(self.file_path)

        
    def onset_event(self, msg: NoteOnset):
        
        # parse the msg
        start_time = msg.header.stamp.secs + msg.header.stamp.nsecs/1000000000  # the system second, not start from zero.
        seq_id = msg.header.seq
        note = msg.note
        confidence = msg.confidence
        duration = 0.5

        # now = datetime.datetime.now()
        # if self.midi_last_save_time == 0:
        #     file_path = os.path.join(self.file_path,f"{now.year}_{now.month}_{now.day}_{now.hour}_{now.minute}.mid")
        #     self

        # load midi file
        self.midi = MyPrettyMiDi(file_path=self.file_path)

        # make sure the time conuter start from zero
        if self.time_first == 0:
            self.time_first = start_time
        note_start_time = start_time-self.time_first
        note_end_time = note_start_time+duration

        # append the note to the midi file under default instrument marimba
        
        self.midi.append_note(
            instrument_name="Marimba",
            onset_note=note,
            start_time=note_start_time,
            end_time=note_end_time
        )
        # save to file
        self.midi.save()
        print(f"onset added :  onset_note:{note}  start_time:{start_time} end_time:{start_time+duration}")   


        # self.windows_end_time = note_end_time
        # self.windows_start_time = self.windows_end_time-self.midi_fig_windows_size
        # if self.windows_start_time < 0:
        #     self.windows_start_time = 0
        
        # if note_end_time - self.midi_fig_update_duration >= self.time_last_draw:
        #     # print('try to draw midi')
        #     self.update_midi_fig()
        #     self.time_last_draw = note_end_time

        
        
    # def update_midi_fig(self):
    #     fs = 44100
    #     start_time = self.windows_start_time

    #     if self.pub_midi.get_num_connections() == 0:
    #         self.midi_fig = None
    #         return

    #     # pm = pretty_midi.PrettyMIDI(midi_file=self.file_path)
    #     pm = deepcopy(self.midi.pm)
    #     # extract the notes inside of the windows
    #     notes_len = len(pm.instruments[0].notes)
    #     start_note_idx = notes_len
    #     for note_id in range(notes_len-1,-1,-1):
    #         if pm.instruments[0].notes[note_id].start >= start_time:
                
    #             pm.instruments[0].notes[note_id].start -= start_time
    #             pm.instruments[0].notes[note_id].end -= start_time
    #             start_note_idx = note_id
    #         else:
    #             break
    #     pm.instruments[0].notes = pm.instruments[0].notes[start_note_idx:]

    #     # draw the midi to numpy ndarrary         
    #     fig = plt.figure(figsize=(8, 4))
    #     # Use librosa's specshow function for displaying the marimba roll
    #     all_pitchs = []
    #     for instr in pm.instruments:
    #         for note in instr.notes:
    #             all_pitchs.append(note.pitch)
    #     start_pitch = min(all_pitchs)
    #     end_pitch = max(all_pitchs) + 1
    #     t0 = time()
    #     display.specshow(pm.get_piano_roll(fs)[start_pitch:end_pitch],
    #                     hop_length=1, sr=fs, x_axis='off', y_axis='cqt_note',
    #                     fmin=pretty_midi.note_number_to_hz(start_pitch))
    #     print(f"display.specshow:{time()-t0}")
    #     # plt.savefig('./tmp.png')

    #     # convert the matplotlib figure to ndarray, which is the same data type as cv2
    #     t0 = time()
    #     fig.canvas.draw()
    #     plt.savefig('./tmp.png')
    #     img = cv2.imread('./tmp.png')
    #     img = np.frombuffer(fig.canvas.tostring_rgb(), dtype=np.uint8)
    #     img = img.reshape(fig.canvas.get_width_height()[::-1] + (3,))
    #     print(f"save and load image:{time()-t0}")
    #     # publish the image through cv_bridge
    #     print("draw a img")
    #     self.pub_midi.publish(
    #         self.cv_bridge.cv2_to_imgmsg(img, "bgr8")
    #         )
    #     plt.close()



    ## For visualization
    # This function is called periodically from FuncAnimation
    def animate(self, i):
        limit = 20
        if os.path.exists(self.file_path):
            notes = MyPrettyMiDi(file_path=self.file_path).pm.instruments[0].notes[-limit:]
            pitchs = [note.pitch for note in notes]
            start_time = [note.start for note in notes]
            durations = [note.duration for note in notes]
            end_time = [note.end for note in notes]
            note_names = [pretty_midi.note_number_to_name(note.pitch) for note in notes]

            # Limit x and y lists to 20 items
            xs = start_time
            ys = pitchs

            # C4 = 60  C7 = 96

            # Draw x and y lists
            self.ax.clear()
            for idx in range(len(xs)):
                color = pretty_midi
                self.ax.bar(x=xs[idx], height=1, width=durations[idx], bottom=ys[idx]-0.5, align="edge",color='b')

            # Format plot
            plt.xticks(rotation=45, ha='right')
            plt.yticks(pitchs, note_names)
            plt.subplots_adjust(bottom=0.30)
            plt.title('marimba note over Time')
            plt.ylabel('Notes')
            plt.xlabel('Time')
        else:
            return


    def _init_plot(self):
        # Create figure for plotting
        self.fig = plt.figure()
        self.ax = self.fig.add_subplot(1, 1, 1)

        # Set up plot to call animate() function periodically
        ani = animation.FuncAnimation(self.fig, self.animate, fargs=(), interval=1000)
        plt.show()
    


    

if __name__=="__main__":
    # TODO problem 1: ini the /onsets, the msg only contain the start time as sec, it could be more percise to include nano-sec.


    rospy.init_node("onset2midi")
    onset2midi = Onset2Midi()
    rospy.spin()



    