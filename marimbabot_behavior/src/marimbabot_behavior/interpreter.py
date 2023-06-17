import rospy
from std_msgs.msg import String
from marimbabot_msgs.msg import HitSequenceGoal, HitSequenceElement

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
    lilypond_file = create_lilypond_file(notes)

    # create midi file from lilypond
    midi_file_path, abjad_formatting_time, lilypond_rendering_time, success = as_midi(lilypond_file, 'test_midi.mid')
    rospy.loginfo(f"created midi file {midi_file_path}, success: {success},\
                    abjad formatting time: {abjad_formatting_time}, lilypond rendering time: {lilypond_rendering_time}")

    return midi_file_path

def read_notes(notes) -> HitSequenceGoal:
    midi_path = lilypond_to_midi(notes)

    midi_stream = pm.PrettyMIDI(midi_path)

    # get sequence of note names
    # note_names = midi_stream.get_pitch_names()
    # timings = midi_stream.get_beats()

    goal = HitSequenceGoal()

    for note in midi_stream.instruments[0].notes:
        name = pm.note_number_to_name(note.pitch)
        goal.hit_sequence_elements.append(
            HitSequenceElement(
                tone_name = ''.join([x for x in name.replace('#', 'is') if not x.isdigit()]).lower(), # TODO regex parse
                octave = int(name[-1]),
                start_time = rospy.Time(note.start),
                tone_duration = rospy.Duration(note.get_duration()),
                loudness = note.velocity/127.0))

    return goal
    # Sends the goal to the action server.
    client.send_goal(note_sequence)