#!/usr/bin/env python

import tempfile
import time
import wave

import actionlib
import rospy
from abjad import Block, LilyPondFile, Score, Staff, Voice
from abjad.persist import as_midi
from midi2audio import FluidSynth
from sound_play.msg import SoundRequest
from std_msgs.msg import String

from marimbabot_msgs.msg import (LilypondAudioAction, LilypondAudioFeedback,
                                 LilypondAudioResult)

"""
Action server for audio generation from lilypond string
"""
class AudioFromLilypond(object):
    # create messages that are used to publish feedback/result
    _feedback = LilypondAudioFeedback()
    _result = LilypondAudioResult()

    def __init__(self, name):
        # publish to audio node (sound_play package)
        self.pub = rospy.Publisher('robotsound', SoundRequest, queue_size=50)

        self._action_name = name
        self._as = actionlib.SimpleActionServer(self._action_name, LilypondAudioAction, execute_cb=self.execute_cb, auto_start = False)
        self._as.start()
      
    def execute_cb(self, goal):
        # helper variables
        success = True
        self._feedback.in_progress = True
        
        # publish info to the console for the user
        rospy.logdebug("Starting " + self._action_name + " with order " + str(goal.lilypond_string)) 
        
        # start executing the action
        audio_filename = ""
        try:
            # create audio from lilypond
            audio_filename = self.create_and_publish_audio(goal.lilypond_string, self.pub)
            # get audio length
            durations_seconds = self.get_audio_length(audio_filename)
            # wait for audio to finish playing
            time.sleep(durations_seconds)
        except:
            # if an error happens during audio generation, set success to false
            success = False
            rospy.logerr("Error during audio generation inside audio_from_lilypond action server")

        # set feedback and result
        self._feedback.in_progress = False
        self._result.success = success
        if success:
            rospy.loginfo('%s: Succeeded' % self._action_name)
            self._as.set_succeeded(self._result)
        else:
            rospy.logerr('%s: Failed' % self._action_name)
            self._as.set_aborted(self._result)
        

    def get_audio_length(self, file_path):
        with wave.open(file_path, 'rb') as audio_file:
            framerate = audio_file.getframerate()
            frames = audio_file.getnframes()
            duration = frames / float(framerate)
            return duration

    # create lilypond file from sentence
    def create_lilypond_file(self, sentence):
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
    def create_audio_from_lilypond(self, sentence):
        lilypond_file  = self.create_lilypond_file(sentence)

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
    def create_and_publish_audio(self, data: String, audio_publisher):
        rospy.logdebug("received recognized sentence")
        
        # create audio from lilypond
        audio_filename = self.create_audio_from_lilypond(data.data)
        rospy.logdebug(f"created audio file {audio_filename}")

        # send audio to audio node (sound_play package)
        # http://docs.ros.org/en/api/sound_play/html/msg/SoundRequest.html
        sound_request = SoundRequest()
        sound_request.sound = -2
        sound_request.command = 1
        sound_request.volume = 10.0
        sound_request.arg = audio_filename

        # publish audio file to audio node (sound_play package)
        audio_publisher.publish(sound_request)

        return audio_filename

if __name__ == '__main__':
    # initialize node
    rospy.init_node("audio_from_lilypond_node", anonymous=False)

    server = AudioFromLilypond("audio_from_lilypond")

    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()
