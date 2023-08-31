#!/usr/bin/env python3

import re
import threading

import actionlib
import rospy
from abjad.exceptions import LilyPondParserError
from std_msgs.msg import String

from marimbabot_behavior.interpreter import read_notes
from marimbabot_msgs.msg import (Command, HitSequence, HitSequenceAction,
                                 LilypondAudioAction, LilypondAudioGoal)


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

    # converts the note_sequence to a hit sequence
    def update_hit_sequence(self):
        try:
            self.hit_sequence = read_notes(self.note_sequence)
        except LilyPondParserError:
            rospy.logwarn('Lilypond string not valid.')
            self.response_pub.publish('Lilypond string not valid.')

    # sets the tempo of the current sequence and updates the hit sequence
    def assign_tempo(self, value=60):
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
            rospy.logwarn('Tempo can only be increased by {} bpm'.format(120-bpm) if faster else 'Tempo can only be decreased by {} bpm.'.format(bpm)-20)
            self.response_pub.publish('Tempo can only be increased by {} B P M'.format(120-bpm) if faster else 'Tempo can only be decreased by {} B P M.'.format(bpm)-20)
            return 'fail' 
        else:
            return self.assign_tempo(new_bpm)

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
                        rospy.logwarn('Volume can only be increased by {} steps.'.format(min(7-dynamics.index(x[1]) for x in sequence_dynamics)))
                        self.response_pub.publish('Volume can only be increased by {} steps.'.format(min(7-dynamics.index(x[1]) for x in sequence_dynamics)))
                else:
                    max_steps = min(dynamics.index(x[1]) for x in sequence_dynamics)
                    if max_steps == 0:
                        rospy.logwarn('Volume can not be decreased any further.')
                        self.response_pub.publish('Volume can not be decreased any further.')
                    else:
                        rospy.logwarn('Volume can only be decreased by {} steps.'.format(max(dynamics.index(x[1]) for x in sequence_dynamics)))
                        self.response_pub.publish('Volume can only be decreased by {} steps.'.format(max(dynamics.index(x[1]) for x in sequence_dynamics)))
                return 'fail'
            
            # change the volume of all dynamic symbols in the sequence
            for i, x in sequence_dynamics:
                new_dynamic = dynamics[min(dynamics.index(x)+value, 7)] if louder else dynamics[max(dynamics.index(x)-value, 0)]
                sequence_list[i] = new_dynamic
            self.note_sequence = ' '.join(sequence_list)
            
            rospy.loginfo(f"updated notes: {self.note_sequence}")
            self.update_hit_sequence()
            return 'success'

        # if there are no dynamic symbols in the sequence, add the next louder/softer dynamic symbol at the beginning of the sequence (default: mp)
        else:
            volume = ' {} '.format(dynamics[4+value] if louder else dynamics[4-value])
            # get the position after the first note
            dynamic_index = re.search(r'[^\\][a-g]\'*[0-9]+', self.note_sequence).end()
            
            # insert volume symbol after first note
            self.note_sequence = self.note_sequence[:dynamic_index] + volume + self.note_sequence[dynamic_index+1:]
            rospy.loginfo(f"updated notes: {self.note_sequence}")
            self.update_hit_sequence()
            return 'success'
            
    """
    Callback function for the feedback from the planning action server.
    Forwards the feedback to the audio node.
    """
    def planning_feedback_cb(self, feedback_msg):
        rospy.logdebug(f"Feedback from planning action server: {feedback_msg}")
        # send it to audio '/audio/hit_sequence' topic
        hit_sequence_msg = HitSequence()
        hit_sequence_msg.header.stamp = rospy.Time.now()
        hit_sequence_msg.sequence_id = self.sequence_id_counter
        hit_sequence_msg.hit_sequence_elements = feedback_msg

        self.sequence_id_counter += 1
        self.hit_sequence_pub.publish(hit_sequence_msg)
        

    # communicates with the planning action server to play the hit sequence on the marimba
    def play(self):
        rospy.loginfo(f"playing notes: {self.note_sequence}")
        rospy.loginfo(f"goal_hit_sequence: {self.hit_sequence}")

        def planning_client_thread():
             # Waits until the action server has started up and started
            self.planning_client.wait_for_server()
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

        # start thread to not block the main thread if the action server is not currently active
        if self.planning_client.simple_state == actionlib.SimpleGoalState.DONE:
            threading.Thread(target=planning_client_thread).start()
        else:    
            rospy.logwarn('The motion is busy.')
            self.response_pub.publish('The motion is busy.')

    # communicates with the lilypond_audio action server to play the audio preview
    def preview(self):
        rospy.loginfo(f"playing audio preview of notes: {self.note_sequence}")

        def audio_from_lilypond_client_thread():
            # Waits until the action server has started up and started
            self.lilypond_audio_client.wait_for_server()
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

        # start thread to not block the main thread if the action server is not currently active
        if self.lilypond_audio_client.simple_state == actionlib.SimpleGoalState.DONE:
            threading.Thread(target=audio_from_lilypond_client_thread).start()
        else:
            rospy.logwarn('Preview is busy.')
            self.response_pub.publish('Preview is busy.')

    def callback_command(self, command_msg):
        command = command_msg.command.lower()
        rospy.loginfo(f"received command: {command}")

        # read notes on the whiteboard
        if command == 'marimbabot read':
            rospy.loginfo('reading notes')
            # update the note_sequence variable with the latest note sequence from vision_node/recognized_notes to signal that notes have been read
            try:
                # wait for the vision node to publish a recognized note sequence
                self.note_sequence = rospy.wait_for_message('vision_node/recognized_notes', String, timeout=5).data 
                rospy.loginfo(f"recognized notes: {self.note_sequence}")
                self.response_pub.publish('Notes recognized.')
                self.update_hit_sequence()
            except rospy.ROSException:
                rospy.logwarn('No notes recognized.')
                self.response_pub.publish('No notes recognized.')

        # play the notes on the marimba using the UR5 or generate an audio preview
        elif re.match(r'marimbabot start (playing|preview)', command):
            # if a note sequence has been read via the 'read' command and the corresponding hit sequence is valid, the hit sequence is send to the planning action server
            if self.hit_sequence:
                if 'playing' in command:
                    self.play()
                else:
                    self.preview()
            else:
                rospy.logwarn('No notes to play. Say reed to read notes.')
                self.response_pub.publish('No notes to play. Say reed to read notes.')
        
        # play in specified tempo
        elif re.match(r'marimbabot (play|preview) in [0-9]+ bpm', command):
            # check if a note sequence has been read via the 'read' command
            if self.note_sequence:
                value = int(command.split(' ')[-2])
                result = self.assign_tempo(value)
                if result == 'success':
                    if 'play' in command:
                        self.play()
                    else:
                        self.preview()
            else:
                rospy.logwarn('No notes to play. Say reed to read notes.')
                self.response_pub.publish('No notes to play. Say reed to read notes.')

        # play faster or slower than the current tempo (by specified bpm value, default = 20)
        elif re.match(r'marimbabot (play|preview) (faster|slower)( by [0-9]+ bpm)?', command):
            # check if a note sequence has been read via the 'read' command
            if self.note_sequence:
                value = int(command.split(' ')[-2]) if 'by' in command else 20
                result = self.change_tempo(faster=True if 'faster' in command else False, value=value)
                if result == 'success':
                    if 'play' in command:
                        self.play()
                    else:
                        self.preview()
            else:
                rospy.logwarn('No notes to play. Say reed to read notes.')
                self.response_pub.publish('No notes to play. Say reed to read notes.')

        elif re.match(r'marimbabot (play|preview) (louder|softer)( by [0-9]+ steps)?', command):
            # check if a note sequence has been read via the 'read' command
            if self.note_sequence:
                value = int(command.split(' ')[-2]) if 'by' in command else 1
                result = self.change_volume(louder=(True if 'louder' in command else False), value=value)
                if result == 'success':
                    if 'play' in command:
                        self.play()
                    else:
                        self.preview()
            else:
                rospy.logwarn('No notes to play. Say reed to read notes.')
                self.response_pub.publish('No notes to play. Say reed to read notes.')


        # stop preview
        elif command == 'marimbabot stop preview':
            rospy.loginfo('Aborting preview.')
            self.response_pub.publish('')

        # # TODO: handle ROS exceptions from the planning side (e.g. planning failed, execution failed, ...)

        # # TODO: add more cases (loop, repeat, save as <name_of_piece>, play <name_of_piece>, ...)

        else:
            rospy.logwarn('Command not recognized.')
            self.response_pub.publish('Command not recognized.')

if __name__ == '__main__':

    rospy.init_node('behavior_node')

    ActionDecider()

    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()