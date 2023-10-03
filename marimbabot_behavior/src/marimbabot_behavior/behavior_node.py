#!/usr/bin/env python3

import re
import threading

import actionlib
import rospy
from abjad.exceptions import LilyPondParserError
from std_msgs.msg import String

from marimbabot_behavior.interpreter import read_notes
from marimbabot_msgs.msg import (Command, HitSequence, HitSequenceAction,
                                 HitSequenceElement, LilypondAudioAction,
                                 LilypondAudioGoal)


class ActionDecider:
    def __init__(self):
        # the currently active note sequence
        # will be updated with the latest note_sequence from vision_node/recognized_notes after the 'read' command was issued
        self.note_sequence = None

        # the currently active hit sequence
        # will be updated together with note_sequence after the 'read' command was issued
        self.hit_sequence = None

        # publishes a response for the synthesized speech
        self.response_pub = rospy.Publisher('~response', String, queue_size=10)

        # listens to the recognized commands from speech node
        self.command_sub = rospy.Subscriber('speech_node/command', Command, self.callback_command)

        # publisher for audio/hit_sequence
        self.hit_sequence_pub = rospy.Publisher('audio/hit_sequence', HitSequence, queue_size=10)
        # sequence_id counter
        self.sequence_id_counter = 0

        # action client to send the hit sequence to the planning action server
        self.planning_client = actionlib.SimpleActionClient('hit_sequence', HitSequenceAction)

        # action client to send the note_sequence to the lilypond_audio action server
        self.lilypond_audio_client = actionlib.SimpleActionClient('audio_from_lilypond', LilypondAudioAction)

        # Waits until the action server has started
        while not self.planning_client.wait_for_server(timeout=rospy.Duration(2)) and not rospy.is_shutdown():
            rospy.loginfo('Waiting for the planning action server to come up')

        # Waits until the action server has started
        while not self.lilypond_audio_client.wait_for_server(timeout=rospy.Duration(2)) and not rospy.is_shutdown():
            rospy.loginfo('Waiting for the audio_from_lilypond action server to come up')

    # checks if the read note sequence includes a repeat symbol and updates the note sequence accordingly
    def check_for_repeat(self):
        if '\\repeat volta 2' in self.note_sequence:
            # get the index of the first note
            note_sequence_list = self.note_sequence.split(' ')
            first_note_index = None
            for i, x in enumerate(note_sequence_list):
                if re.match(r'[a-g]\'*[0-9]+(\>)?(\.)?', x):
                    first_note_index = i
                    break
            self.note_sequence = ' '.join(note_sequence_list[3:] + note_sequence_list[first_note_index:])
            rospy.loginfo(f"updated notes: {self.note_sequence}")

    # converts the note_sequence to a hit sequence
    def update_hit_sequence(self):
        try:
            self.hit_sequence = read_notes(self.note_sequence)
        except LilyPondParserError:
            rospy.logwarn('Lilypond string not valid.')
            self.response_pub.publish('Lilypond string not valid.')

    # sets the tempo of the current sequence and updates the hit sequence
    def assign_tempo(self, value=60):
        if value < 20 or value > 120:
            rospy.logwarn('Tempo can only be set between 20 and 120 BPM.')
            self.response_pub.publish('Tempo can only be set between 20 and 120 BPM.')
            return 'fail'
        
        # if there is already a tempo symbol in the sequence, replace it with the new tempo value
        if '\\tempo' in self.note_sequence:
            self.note_sequence = re.sub(r'\\tempo 4=[0-9]+', '\\\\tempo 4={}'.format(value), self.note_sequence)
        else:
            self.note_sequence = '\\tempo 4={} '.format(value) + self.note_sequence
        rospy.loginfo(f"updated notes: {self.note_sequence}")
        self.update_hit_sequence()
        return 'success'

    # changes the tempo of the current sequence and updates the hit sequence
    def change_tempo(self, faster=True, value=20):
        tempo = re.findall(r'\\tempo 4=[0-9]+', self.note_sequence)
        bpm = int(tempo[0].split('=')[-1]) if len(tempo) > 0 else 60
        new_bpm = bpm + value if faster else bpm - value
        if new_bpm < 20 or new_bpm > 120:
            message = 'Tempo can only be increased by {} BPM'.format(120-bpm) if faster else 'Tempo can only be decreased by {} BPM.'.format(bpm-20)
            rospy.logwarn(message)
            self.response_pub.publish(message)
            return 'fail' 
        else:
            return self.assign_tempo(new_bpm)

    def assign_volume(self, value='\\mp', override=False):
        dynamics = ['\\ppp', '\\pp', '\\p', '\\mp', '\\mf', '\\f', '\\ff', '\\fff']

        if value not in dynamics:
            rospy.logwarn('Volume not valid.')
            self.response_pub.publish('Volume not valid.')
            return 'fail'

        sequence_list = self.note_sequence.split(' ')

        # get the index of the first note
        dynamic_index = None
        for i, x in enumerate(sequence_list):
            if re.match(r'[a-g]\'*[0-9]+(\>)?(\.)?', x):
                dynamic_index = i+1
                break
        
        if not dynamic_index:
            rospy.logwarn('Cannot assign volume if there are no notes.')
            self.response_pub.publish('Cannot assign volume if there are no notes.')
            return 'fail'

        # check if note has an accent
        if dynamic_index+2 < len(sequence_list) and sequence_list[dynamic_index] == '-':
            dynamic_index += 2

        # add (or update) the dynamic symbol after the first note
        if dynamic_index < len(sequence_list) and sequence_list[dynamic_index] in dynamics:
            if override:
                rospy.loginfo('Changing the volume of the first note to {}.'.format(value))
                sequence_list[dynamic_index] = value
        else:
            rospy.loginfo('Assigning the volume of the first note to {}.'.format(value))
            sequence_list = sequence_list[:dynamic_index] + [value] + sequence_list[dynamic_index:]
        self.note_sequence = ' '.join(sequence_list)

        rospy.loginfo(f"updated notes: {self.note_sequence}")
        self.update_hit_sequence()
        return 'success'

    # changes the volume of the current sequence and updates the hit sequence
    def change_volume(self, louder=True, value=1):
        dynamics = ['\\ppp', '\\pp', '\\p', '\\mp', '\\mf', '\\f', '\\ff', '\\fff']
        sequence_list = self.note_sequence.split(' ')
        sequence_dynamics = [(i,x) for i, x in enumerate(sequence_list) if x in dynamics]

        # if there are already dynamic symbols in the sequence, swap them with the next louder/softer dynamic symbol
        if len(sequence_dynamics) > 0:
            # check if the volume can be increased/decreased by the specified value for all dynamic symbols
            if (louder and any(dynamics.index(x[1])+value > 7 for x in sequence_dynamics)) or (not louder and any(dynamics.index(x[1])-value < 0 for x in sequence_dynamics)):
                if louder:
                    max_steps = min(7-dynamics.index(x[1]) for x in sequence_dynamics)
                    if max_steps == 0:
                        rospy.logwarn('Volume can not be increased any further.')
                        self.response_pub.publish('Volume can not be increased any further.')
                    else:
                        rospy.logwarn('Volume can only be increased by {}.'.format(min(7-dynamics.index(x[1]) for x in sequence_dynamics)))
                        self.response_pub.publish('Volume can only be increased by {}.'.format(min(7-dynamics.index(x[1]) for x in sequence_dynamics)))
                else:
                    max_steps = min(dynamics.index(x[1]) for x in sequence_dynamics)
                    if max_steps == 0:
                        rospy.logwarn('Volume can not be decreased any further.')
                        self.response_pub.publish('Volume can not be decreased any further.')
                    else:
                        rospy.logwarn('Volume can only be decreased by {}.'.format(max(dynamics.index(x[1]) for x in sequence_dynamics)))
                        self.response_pub.publish('Volume can only be decreased by {}.'.format(max(dynamics.index(x[1]) for x in sequence_dynamics)))
                return 'fail'
            
            # change the volume of all dynamic symbols in the sequence
            rospy.loginfo('Increasing each dynamic symbol by {}.'.format(value) if louder else 'Decreasing each dynamic symbol by {}.'.format(value))

            for i, x in sequence_dynamics:
                new_dynamic = dynamics[min(dynamics.index(x)+value, 7)] if louder else dynamics[max(dynamics.index(x)-value, 0)]
                sequence_list[i] = new_dynamic
            self.note_sequence = ' '.join(sequence_list)
            
            rospy.loginfo(f"updated notes: {self.note_sequence}")
            self.update_hit_sequence()
            return 'success'
            
    """
    Callback function for the feedback from the planning action server.
    Forwards the feedback to the audio node.
    """
    def planning_feedback_cb(self, feedback_msg):
        rospy.logdebug(f"Feedback from planning action server: {feedback_msg}")
        # the feedback message is a ROS Time value that contain the absolute time when the first note of the sequence was hit
        # Calculate the relative time for each note in order create a HitSequence message
        absolute_start_time: rospy.Time = feedback_msg.first_note_hit_time
        hit_sequence_msg = HitSequence()
        hit_sequence_msg.header.stamp = rospy.Time.now()
        hit_sequence_msg.sequence_id = self.sequence_id_counter
        
        # create a list of HitSequenceElement messages
        hit_sequence_elements : list(HitSequenceElement) = []

        # iterate over the notes in the self.hit_sequence and create a HitSequenceElement message for each note
        for elem in self.hit_sequence.notes:
            hit_sequence_element = HitSequenceElement()
            hit_sequence_element.tone_name = elem.tone_name
            hit_sequence_element.octave = elem.octave
            # add the relative start time to the absolute start time
            hit_sequence_element.start_time = absolute_start_time + rospy.Duration(elem.start_time)
            hit_sequence_element.tone_duration = elem.tone_duration
            hit_sequence_element.loudness = elem.loudness

            hit_sequence_elements.append(hit_sequence_element)

        # add the list of HitSequenceElement messages to the HitSequence message
        hit_sequence_msg.hit_sequence_elements = hit_sequence_elements

        # increment the sequence_id counter
        self.sequence_id_counter += 1

        # send it to audio '/audio/hit_sequence' topic
        self.hit_sequence_pub.publish(hit_sequence_msg)
        
    # communicates with the planning action server to play the hit sequence on the marimba
    def play(self, loop=False):
        def planning_client_thread():
            rospy.loginfo(f"playing notes: {self.note_sequence}")

            # Sends the goal to the action server.
            self.planning_client.send_goal(self.hit_sequence, feedback_cb=self.planning_feedback_cb)
            # Waits for the server to finish performing the action.
            # Includes that the audio file is generated and was played
            self.planning_client.wait_for_result()
            # Prints out the result of executing the action
            rospy.logdebug(f"Result from planning action server: {self.planning_client.get_result()}")
            # Check if we have a success
            if not self.planning_client.get_result().success:
                self.response_pub.publish('The sequence could not be played.')
            else:
                while loop:
                    rospy.loginfo(f"playing notes: {self.note_sequence}")
                    self.planning_client.send_goal(self.hit_sequence, feedback_cb=self.planning_feedback_cb)
                    self.planning_client.wait_for_result()
                    if self.event.is_set():
                        break

        # start thread to not block the main thread if the action server is not currently active
        if self.planning_client.simple_state == actionlib.SimpleGoalState.DONE:
            threading.Thread(target=planning_client_thread).start()
        else:    
            rospy.logwarn('The robot is already playing.')
            self.response_pub.publish('The robot is already playing.')

    # communicates with the lilypond_audio action server to play the audio preview
    def preview(self, loop=False):
        def audio_from_lilypond_client_thread():
            rospy.loginfo(f"playing audio preview of notes: {self.note_sequence}")
            # Sends the goal to the action server.
            self.lilypond_audio_client.send_goal(LilypondAudioGoal(lilypond_string=String(data = self.note_sequence)))
            # Waits for the server to finish performing the action.
            # Includes that the audio file is generated and was played
            self.lilypond_audio_client.wait_for_result()
            # Prints out the result of executing the action
            rospy.logdebug(f"Result from audio_from_lilypond action server: {self.lilypond_audio_client.get_result()}")
            # Check if we have a success
            if not self.lilypond_audio_client.get_result().success:
                self.response_pub.publish('The audio preview could not be played.')

            else:            
                # if loop is True, the audio preview is played in a loop until the 'stop' command is issued
                while loop:
                    rospy.loginfo(f"playing audio preview of notes: {self.note_sequence}")
                    self.lilypond_audio_client.send_goal(LilypondAudioGoal(lilypond_string=String(data = self.note_sequence)))
                    self.lilypond_audio_client.wait_for_result()
                    if self.event.is_set():
                        break

        # start thread to not block the main thread if the action server is not currently active
        if self.lilypond_audio_client.simple_state == actionlib.SimpleGoalState.DONE:
            threading.Thread(target=audio_from_lilypond_client_thread).start()
        else:
            rospy.logwarn('The audio preview is already playing.')
            self.response_pub.publish('The audio preview is already playing.')

    def callback_command(self, command_msg):
        # command = command_msg.command.lower()
        rospy.loginfo(f"received command: {command_msg}")

        # read notes on the whiteboard
        if command_msg.behavior == "read":
            rospy.loginfo('reading notes')
            # update the note_sequence variable with the latest note sequence from vision_node/recognized_notes to signal that notes have been read
            try:
                # wait for the vision node to publish a recognized note sequence
                self.note_sequence = rospy.wait_for_message('vision_node/recognized_notes', String, timeout=5).data 
                rospy.loginfo(f"recognized notes: {self.note_sequence}")
                self.response_pub.publish('Notes recognized.')
                self.check_for_repeat()
                self.update_hit_sequence()
            except rospy.ROSException:
                rospy.logwarn('No notes recognized.')
                self.response_pub.publish('No notes recognized.')

        else:
            # check if a note sequence has been read via the 'read' command
            if not self.note_sequence:
                rospy.logwarn('No notes to play. Say read to read notes.')
                self.response_pub.publish('No notes to play. Say reed to read notes.')
                return

            # set the tempo of the current sequence (to specified bpm value, default = 60)
            if command_msg.action == "setup speed":
                value = 60 if command_msg.parameter == "" else int(command_msg.parameter)
                result = self.assign_tempo(value)
                if result == "fail":
                    return
                
            # increase or decrease the tempo of the current sequence (by specified bpm value, default = 20)
            elif command_msg.action == "increase speed" or command_msg.action == "decrease speed":
                value = 20 if command_msg.parameter == "" else int(command_msg.parameter)
                result = self.change_tempo(faster=True if 'increase' in command_msg.action else False, value=value)
                if result == "fail":
                    return
                
            # set the volume of the current sequence (to specified dynamic symbol, default = mp)
            # TODO

            # increase or decrease the volume of the current sequence (by specified steps, default = 1)
            elif command_msg.action == "increase volume" or command_msg.action == "decrease volume":
                value = 1 if command_msg.parameter == "" else int(command_msg.parameter)
                result = self.change_volume(louder=(True if 'increase' in command_msg.action else False), value=value)
                if result == "fail":
                    return
                

            # play the notes on the marimba using the UR5 or generate an audio preview
            if command_msg.behavior == "play":
                # create event to stop the play if the 'stop' command is issued
                self.event = threading.Event()
                if self.planning_client.simple_state == actionlib.SimpleGoalState.DONE:
                    self.play(loop = True if command_msg.action == "loop" else False)
                else:
                    rospy.logwarn('The robot is already playing.')
                    self.response_pub.publish('The robot is already playing.')

            elif command_msg.behavior == "preview":
                # create event to stop the preview if the 'stop' command is issued
                self.event = threading.Event()
                if self.lilypond_audio_client.simple_state == actionlib.SimpleGoalState.DONE:
                    self.preview(loop = True if command_msg.action == "loop" else False)
                else:
                    rospy.logwarn('The audio preview is already playing.')
                    self.response_pub.publish('The audio preview is already playing.')

            elif command_msg.behavior == "stop":
                rospy.loginfo('Aborting play.')
                # stop any running threads by setting the event
                self.event.set()

                # check if the planning action server is currently active
                if self.planning_client.simple_state == actionlib.SimpleGoalState.ACTIVE:
                    # cancel the goal of the planning action server
                    self.planning_client.cancel_goal()

                # check if the lilypond_audio action server is currently active
                if self.lilypond_audio_client.simple_state == actionlib.SimpleGoalState.ACTIVE:
                    # cancel the goal of the lilypond_audio action server
                    self.lilypond_audio_client.cancel_goal()

                # send an empty message to the soundplay node to stop any audio preview (or TTS)
                self.response_pub.publish('')

            else:
                rospy.logwarn('Command not handled: {}'.format(command_msg))
                self.response_pub.publish('Command not recognized.')

if __name__ == '__main__':

    rospy.init_node('behavior_node')

    ActionDecider()

    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()