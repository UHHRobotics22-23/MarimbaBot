#!/usr/bin/env python

import tempfile

import rospy
from abjad import Block, LilyPondFile, Score, Staff, Voice
from abjad.persist import as_midi
from midi2audio import FluidSynth
from sound_play.msg import SoundRequest
from std_msgs.msg import String


# create lilypond file from sentence
def create_lilypond_file(sentence):
    # generate abjad staff
    voice = Voice(sentence)
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

# create audio from lilypond
def create_audio_from_lilypond(sentence):
    lilypond_file  = create_lilypond_file(sentence)

    # store lilypond as midi temporarily
    # store temporarily due to parser limitations concerning '\midi' https://abjad.github.io/api/abjad/parsers/parser.html 
    temp_ = tempfile.TemporaryFile()

    # create midi file from lilypond
    midi_file_path, abjad_formatting_time, lilypond_rendering_time, success = as_midi(lilypond_file, str(temp_.name))
    rospy.logdebug(f"created midi file {midi_file_path}, success: {success},\
                    abjad formatting time: {abjad_formatting_time}, lilypond rendering time: {lilypond_rendering_time}")

    # create audio from midi
    temp_ = tempfile.TemporaryFile()
    audio_filename = str(temp_.name) + ".wav"

    # create audio file from midi
    FluidSynth().midi_to_audio(midi_file_path, audio_filename)

    return audio_filename

# callback for vision node, includes audio publisher
def callback_vision_results(data: String, audio_publisher):
    rospy.logdebug("received recognized sentence")
    
    # create audio from lilypond
    audio_filename = create_audio_from_lilypond(data.data)
    rospy.logdebug(f"created audio file {audio_filename}")

    # send audio to audio node (sound_play package)
    # http://docs.ros.org/en/api/sound_play/html/msg/SoundRequest.html
    sound_request = SoundRequest()
    sound_request.sound = -2
    sound_request.command = 1
    sound_request.volume = 0.5
    sound_request.arg = audio_filename

    # publish audio file to audio node (sound_play package)
    audio_publisher.publish(sound_request)

# create audio from lilypond publisher
# send audio to audio node (sound_play package)
def listener():
    # initialize node
    rospy.init_node('~audio_generation', anonymous=True)

    # subscribe to vision node and publish to audio node (sound_play package)
    pub = rospy.Publisher('robotsound', SoundRequest, queue_size=50)
    rospy.Subscriber("vision_node/recognized_notes", String, callback_vision_results, callback_args=(pub))

    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()

if __name__ == '__main__':
    # start listener node
    listener()