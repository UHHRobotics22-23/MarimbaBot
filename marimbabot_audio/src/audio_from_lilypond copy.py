#!/usr/bin/env python

import tempfile

from abjad import Block, LilyPondFile, Score, Staff, Voice
from abjad.persist import as_midi
from midi2audio import FluidSynth


# create lilypond file from sentence
def create_lilypond_file(sentence):
    # generate abjad staff
    voice = Voice(sentence)
    staff = Staff([voice])
    score = Score([staff], name="Score")

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
    midi_filename = str(temp_.name) + ".midi"
    midi_file_path, abjad_formatting_time, lilypond_rendering_time, success = as_midi(lilypond_file, midi_filename)
    print(midi_file_path)
    rospy.logdebug(f"was successful: {success}")

    # create audio from midi
    # it is hardcoded for now, could be changed to a temporary file as well
    temp_ = tempfile.TemporaryFile()
    audio_filename = str(temp_.name) + ".wav"

    FluidSynth().midi_to_audio(midi_file_path, audio_filename)

    return audio_filename

if __name__ == '__main__':
    # import abjad

    # string = "d'8 f' a' d'' f'' gs'4 r8 e' gs' b' e'' gs'' a'4"
    # voice = abjad.Voice(string, name="RH_Voice")
    # staff = abjad.Staff([voice], name="RH_Staff")
    # score = abjad.Score([staff], name="Score")

    # score_block = abjad.Block(name="score")
    # score_block.items.append(score)
    # midi_block = abjad.Block(name="midi")
    # score_block.items.append(midi_block)
    # lilypond_file = abjad.LilyPondFile(items=[score_block])
    # abjad.persist.as_midi(lilypond_file, "bar")

    create_audio_from_lilypond("c'4 d'4 e'4 f'4 g'4 a'4 b'4 c''4")
