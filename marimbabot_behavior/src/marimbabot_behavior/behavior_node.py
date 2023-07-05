#!/usr/bin/env python3

import actionlib
import threading
import rospy
from abjad.exceptions import LilyPondParserError
from std_msgs.msg import String

from marimbabot_behavior.interpreter import read_notes
from marimbabot_msgs.msg import LilypondAudioAction, LilypondAudioGoal
from marimbabot_msgs.msg import HitSequenceAction
import re
from marimbabot_msgs.msg import Command

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
        if '\\tempo' in self.note_sequence:
            self.note_sequence = re.sub(r'\\tempo 4 = [0-9]+', '\\\\tempo 4 = {}'.format(value), self.note_sequence)
        else:
            self.note_sequence = '\\tempo 4 = {} '.format(value) + self.note_sequence
        
        rospy.logdebug(f'Tempo set to {value} bpm')
        rospy.loginfo(f"updated notes: {self.note_sequence}")
        self.update_hit_sequence()

    # changes the tempo of the current sequence and updates the hit sequence
    def change_tempo(self, faster=True, value=20):
        tempo = re.findall(r'\\tempo 4 = [0-9]+', self.note_sequence)
        if len(tempo) > 0:
            tempo = int(tempo[0].split(' ')[-1])
            assign_tempo(tempo + value if faster else tempo - value)

    # communicates with the planning action server to play the hit sequence on the marimba
    def play(self):
        rospy.loginfo(f"playing notes: {self.note_sequence}")
        rospy.loginfo(f"goal_hit_sequence: {self.hit_sequence}")

        # preparing the hit sequence message for the audio node
            # hit_sequence_msg = HitSequence()
            # hit_sequence_msg.header.stamp = rospy.Time.now()
            # hit_sequence_msg.seq_id = self.id_counter
            # hit_sequence_msg.seq = self.hit_sequenc

        def planning_client_thread():
             # Waits until the action server has started up and started
            self.planning_client.wait_for_server()
            # Sends the goal to the action server.
            self.planning_client.send_goal(self.hit_sequence)
            # Waits for the server to finish performing the action.
            # Includes that the audio file is generated and was played
            self.planning_clinet.wait_for_result()
            # Prints out the result of executing the action
            rospy.logdebug(f"Result from planning action server: {self.planning_client.get_result()}")
            # Check if we have a success
            if not self.planning_clinet.get_result().success:
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
        command = command_msg.command
        rospy.loginfo(f"received command: {command}")

        # read notes on the whiteboard
        if command == 'Marimbabot read':
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

        # play the notes on the marimba using the UR5
        elif command == 'Marimbabot start playing':
            # if a note sequence has been read via the 'read' command and the corresponding hit sequence is valid, the hit sequence is send to the planning action server
            if self.hit_sequence:
                self.play()
            else:
                rospy.logwarn('No notes to play. Say reed to read notes.')
                self.response_pub.publish('No notes to play. Say reed to read notes.')
        
        # play in specified tempo
        elif re.match(r'Marimbabot play in [0-9]+ bpm', command):
            # check if a note sequence has been read via the 'read' command
            if self.note_sequence:
                value = int(command.split(' ')[-2])
                self.assign_tempo(value)
                self.play()
            else:
                rospy.logwarn('No notes to play. Say reed to read notes.')
                self.response_pub.publish('No notes to play. Say reed to read notes.')

        # play faster or slower than the current tempo (by specified bpm value, default = 20)
        elif re.match(r'Marimbabot play (faster|slower)( by [0-9]+ bpm)?', command):
            # check if a note sequence has been read via the 'read' command
            if self.note_sequence:
                value = int(command.split(' ')[-2]) if 'by' in command else 20
                self.change_tempo(faster=True if 'faster' in command else False, value=value)
                self.play()
            else:
                rospy.logwarn('No notes to play. Say reed to read notes.')
                self.response_pub.publish('No notes to play. Say reed to read notes.')

        # preview using sound interpreted by computer (MIDI)
        elif re.match(r'Marimbabot preview( in [0-9]+ bpm)?', command):
            # if a note sequence has been read via the 'read' command
            if self.note_sequence:
                if 'bpm' in command:
                    value = int(command.split(' ')[-2])
                    self.assign_tempo(value)
                self.preview()
            else:
                rospy.logwarn('No notes to preview. Say reed to read notes.')
                self.response_pub.publish('No notes to preview. Say reed to read notes.')

        # preview faster or slower than the current tempo (by specified bpm value, default = 20)
        elif re.match(r'Marimbabot preview (faster|slower)( by [0-9]+)?', command):
            # check if a note sequence has been read via the 'read' command
            if self.note_sequence:
                value = int(command.split(' ')[-1]) if 'by' in command else 20
                self.change_tempo(faster=True if 'faster' in command else False, value=value)
                self.preview()
            else:
                rospy.logwarn('No notes to play. Say reed to read notes.')
                self.response_pub.publish('No notes to play. Say reed to read notes.')

        # stop preview
        elif command == 'Marimbabot stop preview':
            rospy.loginfo('Stopping preview.')
            self.response_pub.publish('')


        # TODO: handle ROS exceptions from the planning side (e.g. planning failed, execution failed, ...)

        # TODO: add more cases (loop, repeat, faster, slower, save as <name_of_piece>, play <name_of_piece>, ...)

        else:
            rospy.logwarn('Command not recognized.')
            self.response_pub.publish('Command not recognized.')

if __name__ == '__main__':

    rospy.init_node('behavior_node')

    ActionDecider()

    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()