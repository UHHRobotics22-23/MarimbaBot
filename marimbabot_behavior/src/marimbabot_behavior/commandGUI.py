#!/usr/bin/env python3

from tkinter import *

import rospy
from std_msgs.msg import String
from sensor_msgs.msg import Image as ROSImage
from PIL import ImageTk, Image
import numpy as np

from marimbabot_msgs.msg import Command

from cv_bridge import CvBridge, CvBridgeError

# Song Library
library = {'Frère Jacques': "c''4 d''4 e''4 c''4 c''4 d''4 e''4 c''4 e''4 f''4 g''2 e''4 f''4 g''2 g''8 a''8 g''8 f''8 e''4 c''4 g''8 a''8 g''8 f''8 e''4 c''4 c''4 g'4 c''2 c''4 g'4 c''2",
            'All my ducklings': "c''4 d''4 e''4 f''4 g''2 g''2 a''4 a''4 a''4 a''4 g''1 a''4 a''4 a''4 a''4 g''1 f''4 f''4 f''4 f''4 e''2 e''2 g''4 g''4 g''4 g''4 c''4",
            'Scale': "c'4 d'4 e'4 f'4 g'4 a'4 b'4 c''4 b'4 a'4 g'4 f'4 e'4 d'4 c'4",
            'Double Scale': "c'4 d'4 e'4 f'4 g'4 a'4 b'4 c''4 d''4 e''4 f''4 g''4 a''4 b''4 c'''4 b''4 a''4 g''4 f''4 e''4 d''4 c''4 b'4 a'4 g'4 f'4 e'4 d'4 c'4",
            'Full Scale': "c'4 cs'4 d'4 ds'4 e'4 f'4 fs'4 g'4 gs'4 a'4 as'4 b'4 c''4 cs''4 d''4 ds''4 e''4 f''4 fs''4 g''4 gs''4 a''4 as''4 b''4 c'''4",
            'Chord Testing': "c'4 <c' d'>4 <c' e'>4 <c' f'>4 <c' g'>4 <c' a'>4 <c' b'>4 <c' c''>4 d'4 <d' e'>4 <d' f'>4 <d' g'>4 <d' a'>4 <d' b'>4 <d' c''>4 <d' d''>4 e'4 <e' f'>4 <e' g'>4 <e' a'>4 <e' b'>4 <e' c''>4 <e' d''>4 <e' e''>4 f'4 <f' g'>4 <f' a'>4 <f' b'>4 <f' c''>4 <f' d''>4 <f' e''>4 <f' f''>4 g'4 <g' a'>4 <g' b'>4 <g' c''>4 <g' d''>4 <g' e''>4 <g' f''>4 <g' g''>4 a'4 <a' b'>4 <a' c''>4 <a' d''>4 <a' e''>4 <a' f''>4 <a' g''>4 <a' a''>4 b'4 <b' c''>4 <b' d''>4 <b' e''>4 <b' f''>4 <b' g''>4 <b' a''>4 <b' b''>4 c''4 <c'' d''>4 <c'' e''>4 <c'' f''>4 <c'' g''>4 <c'' a''>4 <c'' b''>4 <c'' c'''>4"}

# sets the input note sequence to the selected song from the song library
def set_song(*args):
    entry.delete(0, END)
    song = library[selected_song.get()]
    entry.insert(0, song)
    
# sets the input note sequence to the notes recognized by the camera
def read_notes():
    try:
        notes = rospy.wait_for_message('vision_node/recognized_notes', String, timeout=3).data
    except rospy.ROSException:
        notes = "No notes detected"
    entry.delete(0, END)
    entry.insert(0, notes)

# publishes the input note sequence to the note sequence topic for the behavior node
def confirm_notes():
    notes = entry.get()
    note_sequence_pub.publish(notes)

# enables or disables the action options based on the selected behavior
def set_action_options(*args):
    behavior = selected_behavior.get()
    if behavior == 'play' or behavior == 'preview':
        action.config(state=NORMAL)
    else:
        selected_action.set("Action Selection")
        parameter.delete(0, END)
        action.config(state=DISABLED)
        parameter.config(state=DISABLED)

# sets or disables the parameter options based on the selected action
def set_paramerter_options(*args):
    action = selected_action.get()
    if action == 'increase volume' or action == 'decrease volume':
        parameter.config(state=NORMAL)
        by.config(text="by")
        metric.config(text="steps")
    elif action == 'increase speed' or action == 'decrease speed':
        parameter.config(state=NORMAL)
        by.config(text="by")
        metric.config(text="bpm")
    else:
        parameter.delete(0, END)
        parameter.config(state=DISABLED)
        by.config(text="")
        metric.config(text="")

# publishes the selected command to the command topic for the behavior node
def command_pub():
    command = Command()
    command.behavior = selected_behavior.get()
    command.action = selected_action.get()
    command.parameter = parameter.get()
    command_pub.publish(command)
    parameter.delete(0, END)

# initialize GUI
root = Tk()
root.title("Marimbabot Command GUI")

# Input Note Sequence
Label(root, text="Input Note Sequence").grid(row=0, column=1, pady=4)
defaultText = StringVar()
defaultText.set("Please read from the camera feed, select from the song library, or enter a custom note sequence.")
entry = Entry(root, width=80, textvariable=defaultText)
entry.grid(row=1, column=0, columnspan=3, pady=10)

# Confirm Button
Button(root, text='Confirm', command=confirm_notes, bg='#6fbbd3', fg='white').grid(row=1, column=4, pady=4)

# Read Button
Button(root, text='Read from camera', command=read_notes).grid(row=2, column=0, pady=4)

# Song Library
selected_song = StringVar()
selected_song.set("Song Library")
drop = OptionMenu(root, selected_song, *library.keys())
drop.grid(row=2, column=1, pady=4)
selected_song.trace("w", set_song)

# Active Note Sequence
Label(root, text="Active Note Sequence").grid(row=4, column=1, pady=4)
current_sequence = Label(root, text="", bg="white", width=80, wraplength=630, anchor="w", justify=LEFT)
current_sequence.grid(row=5, column=0, columnspan=3, pady=10)

# Command Builder
Label(root, text="Command Builder").grid(row=6, column=1)

action_options = ['no action', 'louder', 'softer', 'faster', 'Slower', 'loop']
active_action_options = StringVar()

# Behavior Selection
selected_behavior = StringVar()
selected_behavior.set("Command Selection")
behavior = OptionMenu(root, selected_behavior, *['play', 'preview', 'stop'])
behavior.grid(row=7, column=0, pady=4)
behavior.config(state=DISABLED)

# Action Selection
selected_action = StringVar()
selected_action.set("Action Selection")
action = OptionMenu(root, selected_action, *['no action', 'increase volume', 'decrease volume', 'increase speed', 'decrease speed', 'loop'])
action.grid(row=7, column=1, pady=4)
action.config(state=DISABLED)

# Parameter Selection
parameter = Entry(root, width=10)
parameter.grid(row=7, column=2,  pady=4)
parameter.config(state=DISABLED)

by = Label(root, text="")
by.grid(row=7, column=2, sticky=W, pady=4)

metric = Label(root, text="")
metric.grid(row=7, column=2, sticky=E, pady=4)

# Trace Selections
selected_behavior.trace("w", set_action_options)
selected_action.trace("w", set_paramerter_options)

# GO Button
Button(root, text='GO', command=command_pub, bg='#6fbbd3', fg='white').grid(row=7, column=4, pady=4)

# updates the active note sequence based on the note sequence published by the behavior node
def note_sequence_callback(msg):
    current_sequence.config(text=msg.data)

# enables the Command Builder when the behavior node has recognized the input note sequence
def response_callback(msg):
    if msg.data == "Notes recognized.":
        behavior.config(state=NORMAL)
    elif msg.data == "No notes recognized.":
        action.config(state=DISABLED)

# Initialize ROS node
rospy.init_node('command_gui')

# Set up ROS subscribers and publishers
note_sequence_sub = rospy.Subscriber('behavior_node/note_sequence', String, note_sequence_callback)
response_sub = rospy.Subscriber('behavior_node/response', String, response_callback)
note_sequence_pub = rospy.Publisher('command_gui/note_sequence', String, queue_size=10)
command_pub = rospy.Publisher('speech_node/command', Command, queue_size=10)

mainloop()
rospy.spin()