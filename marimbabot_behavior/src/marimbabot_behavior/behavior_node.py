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
        # will be updated with the latest sentence from vision_node/recognized_sentence after the 'read' command was issued
        self.sentence = None

        # the currently active hit sequence
        # will be updated together with sentence after the 'read' command was issued
        self.hit_sequence = None

        # publishes a response for the synthesized speech
        self.response_pub = rospy.Publisher('~response', String, queue_size=10)

        # listens to the recognized commands from speech node
        self.command_sub = rospy.Subscriber('speech_node/command', Command, self.callback_command)

        # action client to send the hit sequence to the planning action server
        self.planning_client = actionlib.SimpleActionClient('hit_sequence', HitSequenceAction)

        # action client to send the sentence to the lilypond_audio action server
        self.lilypond_audio_client = actionlib.SimpleActionClient('audio_from_lilypond', LilypondAudioAction)

    def update_hit_sequence(self):
        try:
            self.hit_sequence = read_notes(self.sentence)
        except LilyPondParserError:
            rospy.logwarn('Lilypond string not valid. Make sure the note are valid and readable.')
            self.response_pub.publish('Lilypond string not valid. Make sure the note sequence is valid.')

    def play(self):
        rospy.loginfo(f"playing notes: {self.sentence}")
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
                self.response_pub.publish('The sequence could not be played. Try again later.')

        # start thread to not block the main thread if the action server is not currently active
        if self.planning_client.simple_state == actionlib.SimpleGoalState.DONE:
            threading.Thread(target=planning_client_thread).start()
        else:    
            rospy.logwarn('The motion is busy. Try again later.')
            self.response_pub.publish('The motion is busy. Try again later.')

    def preview(self):
        rospy.loginfo(f"playing audio preview of notes: {self.sentence}")

        def audio_from_lilypond_client_thread():
            # Waits until the action server has started up and started
            self.lilypond_audio_client.wait_for_server()
            # Sends the goal to the action server.
            self.lilypond_audio_client.send_goal(LilypondAudioGoal(lilypond_string=String(data = self.sentence)))
            # Waits for the server to finish performing the action.
            # Includes that the audio file is generated and was played
            self.lilypond_audio_client.wait_for_result()
            # Prints out the result of executing the action
            rospy.logdebug(f"Result from audio_from_lilypond action server: {self.lilypond_audio_client.get_result()}")
            # Check if we have a success
            if not self.lilypond_audio_client.get_result().success:
                self.response_pub.publish('The audio preview could not be played. Try again later.')

        # start thread to not block the main thread if the action server is not currently active
        if self.lilypond_audio_client.simple_state == actionlib.SimpleGoalState.DONE:
            threading.Thread(target=audio_from_lilypond_client_thread).start()
        else:
            rospy.logwarn('Preview is busy. Try again later.')
            self.response_pub.publish('Preview is busy. Try again later.')

    def change_tempo(self, faster=True):
        tempo = re.findall('\\\\tempo 4 = [0-9]+', self.sentence)
        if len(tempo) > 0:
            tempo = tempo[0].split(' ')[-1]
            self.sentence = re.sub('\\\\tempo 4 = [0-9]+', '\\\\tempo 4 = {}'.format(str(int(tempo) + 20)), self.sentence) if faster else re.sub('\\\\tempo 4 = [0-9]+', '\\\\tempo 4 = {}'.format(str(int(tempo) - 20)), self.sentence)
        else:
            self.sentence = '\\tempo 4 = 80 ' + self.sentence if faster else '\\tempo 4 = 40 ' + self.sentence

        rospy.loginfo(f"updated notes: {self.sentence}")
        self.update_hit_sequence()
        rospy.logwarn('Tempo increased.' if faster else 'Tempo decreased.')
        self.response_pub.publish('Tempo increased.' if faster else 'Tempo decreased.')

    def callback_command(self, command_msg):
        command = command_msg.command
        rospy.loginfo(f"received command: {command}")

        # read notes on the whiteboard
        if command == 'Marimbabot read':
            rospy.loginfo('reading notes')
            # update the sentence variable with the latest sentence from vision_node/recognized_sentence to signal that notes have been read
            try:
                # wait for the vision node to publish a recognized sentence
                self.sentence = rospy.wait_for_message('vision_node/recognized_notes', String, timeout=5).data 
                rospy.loginfo(f"recognized notes: {self.sentence}")
                self.response_pub.publish('Notes recognized.')
                self.update_hit_sequence()
            except rospy.ROSException:
                rospy.logwarn('No notes recognized. Make sure the notes are readable and visible to the camera.')
                self.response_pub.publish('No notes recognized. Make sure the notes are readable and visible to the camera.')

        elif command == 'Marimbabot start playing':
            # if a note sequence has been read via the 'read' command and the corresponding hit sequence is valid, the hit sequence is send to the planning action server
            if self.hit_sequence:
               self.play()
            else:
                rospy.logwarn('No notes to play. Say reed to read notes.')
                self.response_pub.publish('No notes to play. Say reed to read notes.')
                
        # pre play, use sound interpreted by computer
        elif command == 'Marimbabot preview':
            # if a note sequence has been read via the 'read' command
            if self.sentence:
                self.preview()
            else:
                rospy.logwarn('No notes to preview. Say reed to read notes.')
                self.response_pub.publish('No notes to preview. Say reed to read notes.')

        elif command == 'Marimbabot stop preview':
            rospy.logwarn('Stopping preview.')
            self.response_pub.publish('')

        # increase/decrease the tempo of the active notes
        elif command == 'Marimbabot play faster' or command == 'Marimbabot play slower':
            # check if a note sequence has been read via the 'read' command
            if self.sentence:
                self.change_tempo(faster=True if command == 'Marimbabot play faster' else False)
                self.play()
            else:
                rospy.logwarn('No notes to play. Say reed to read notes.')
                self.response_pub.publish('No notes to play. Say reed to read notes.')

        elif command == 'Marimbabot preview faster' or command == 'Marimbabot preview slower':
            # check if a note sequence has been read via the 'read' command
            if self.sentence:
                self.change_tempo(faster=True if command == 'Marimbabot preview faster' else False)
                self.preview()
            else:
                rospy.logwarn('No notes to play. Say reed to read notes.')
                self.response_pub.publish('No notes to play. Say reed to read notes.')

        # TODO: handle ROS exceptions from the planning side (e.g. planning failed, execution failed, ...)

        # TODO: elif command == 'stop':
        
        # TODO: add more cases (loop, repeat, faster, slower, save as <name_of_piece>, play <name_of_piece>, ...)

        else:
            self.response_pub.publish('Command not recognized.')

if __name__ == '__main__':

    rospy.init_node('behavior_node')

    ActionDecider()

    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()