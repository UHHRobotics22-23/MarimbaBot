import rospy
from std_msgs.msg import String

import tempfile
from abjad import Block, LilyPondFile, Score, Staff, Voice
from abjad.persist import as_midi
import pretty_midi as pm

# create lilypond file from sentence
def create_lilypond_file(notes):
    # generate abjad staff
    voice = Voice(notes)
    staff = Staff([voice])
    score = Score([staff], name="Score")

    # generate abjad score block and midi block as described here https://github.com/Abjad/abjad/issues/1352#issuecomment-904852122
    score_block = Block(name="score")
    score_block.items.append(score)
    midi_block = Block(name="midi")
    score_block.items.append(midi_block)

    # generate lilypond file
    header_block = Block(name="header")
    header_block.tagline = "#ff"
    lilypond_file = LilyPondFile(
        items=[
            header_block,
            score_block        
            ]
    )

    return lilypond_file

def lilypond_to_midi(notes):
    lilypond_file = create_lilypond_file(notes.data)

    # create midi file from lilypond
    midi_file_path, abjad_formatting_time, lilypond_rendering_time, success = as_midi(lilypond_file, 'test_midi.mid')
    rospy.loginfo(f"created midi file {midi_file_path}, success: {success},\
                    abjad formatting time: {abjad_formatting_time}, lilypond rendering time: {lilypond_rendering_time}")

    return midi_file_path

def callback_read_notes(notes, pub):
    midi_path = lilypond_to_midi(notes)

    midi_stream = pm.PrettyMIDI(midi_path)

    # get sequence of note names
    # note_names = midi_stream.get_pitch_names()
    # timings = midi_stream.get_beats()
    note_sequence = []
    for note in midi_stream.instruments[0].notes:
        info = {}
        name = pm.note_number_to_name(note.pitch)
        info['note'] = ''.join([x for x in name if not x.isdigit()])
        info['octave'] = ''.join([x for x in name if x.isdigit()])
        info['start'] = note.start
        info['duration'] = note.get_duration()
        info['loudness'] = note.velocity/127.0
        note_sequence.append(info)

    pub.publish(str(note_sequence))

def listener():
    pub = rospy.Publisher('~midi_path', String, queue_size=10) 

    # listens to the currently read note sequence
    sub = rospy.Subscriber('behavior_node/read_notes', String, callback_read_notes, callback_args=(pub))

if __name__ == '__main__':

    rospy.init_node('behavior_node')

    listener()

    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()