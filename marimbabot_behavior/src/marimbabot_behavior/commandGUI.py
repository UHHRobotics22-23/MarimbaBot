#!/usr/bin/env python3

from tkinter import *

import rospy
from std_msgs.msg import String
from sensor_msgs.msg import Image as ROSImage
from PIL import ImageTk, Image
import numpy as np

from marimbabot_msgs.msg import Command

from cv_bridge import CvBridge, CvBridgeError

def clear_entry():
    entry.delete(0, END)

def set_song(*args):
    library = {'Frère Jacques': "c''4 d''4 e''4 c''4 c''4 d''4 e''4 c''4 e''4 f''4 g''2 e''4 f''4 g''2 g''8 a''8 g''8 f''8 e''2 c''2 g''8 a''8 g''8 f''8 e''2 c''2 c''4 g'4 c''2 c''4 g'4 c''2",
               'All my ducklings': "c''4 d''4 e''4 f''4 g''2 g''2 a''4 a''4 a''4 a''4 g''1 a''4 a''4 a''4 a''4 g''1 f''4 f''4 f''4 f''4 e''2 e''2 g''4 g''4 g''4 g''4 c''4",
               'Scale': "c''4 d''4 e''4 f''4 g''4 a''4 b''4 c''4 b''4 a''4 g''4 f''4 e''4 d''4 c''4"}
    
    clear_entry()
    song = library[selected_song.get()]
    entry.insert(0, song)
    
def read_notes():
    try:
        notes = rospy.wait_for_message('vision_node/recognized_notes', String, timeout=5).data
    except rospy.ROSException:
        notes = "No notes detected"
    clear_entry()
    entry.insert(0, notes)
    # command = Command()
    # command.behavior = "read"
    # command_pub.publish(command)

def confirm_notes():
    notes = entry.get()
    note_sequence_pub.publish(notes)

def set_action_options(*args):
    behavior = selected_behavior.get()
    if behavior == 'play' or behavior == 'preview':
        action.config(state=NORMAL)
    else:
        selected_action.set("Action Selection")
        parameter.delete(0, END)
        action.config(state=DISABLED)
        parameter.config(state=DISABLED)

def set_paramerter_options(*args):
    action = selected_action.get()
    if action == 'increase volume' or action == 'decrease volume':
        parameter.config(state=NORMAL)
        metric.config(text="steps")
    elif action == 'increase speed' or action == 'decrease speed':
        parameter.config(state=NORMAL)
        metric.config(text="bpm")
    else:
        parameter.delete(0, END)
        parameter.config(state=DISABLED)
        metric.config(text="")

def command_pub():
    command = Command()
    command.behavior = selected_behavior.get()
    command.action = selected_action.get()
    command.parameter = parameter.get()
    command_pub.publish(command)

# create GUI
root = Tk()
root.title("Marimbabot Command GUI")

# Camera Feed
Label(root, text="Camera Feed").grid(row=0, column=1)
camera = Canvas(root, bg = 'white', height=100, width=100)
camera.grid(row=1, column=0, columnspan=3, rowspan=3, pady=10)
Button(root, text='Read', command=read_notes).grid(row=4, column=1, pady=4)

# Input Note Sequence
Label(root, text="Input Note Sequence").grid(row=5, column=1)
defaultText = StringVar()
defaultText.set("Please read from the camera feed, select from the song library, or enter a custom note sequence.")
entry = Entry(root, width=80, textvariable=defaultText)
entry.grid(row=6, column=0, columnspan=3, pady=10)
# Button(root, text='Clear', command=clear_entry).grid(row=1, column=6, sticky=W, pady=4)

# Song Library
selected_song = StringVar()
selected_song.set("Song Library")
drop = OptionMenu(root, selected_song, *['Frère Jacques', 'All my ducklings', 'Scale'])
drop.grid(row=6, column=4, pady=4)
selected_song.trace("w", set_song)

# Confirm Button
Button(root, text='Confirm', command=confirm_notes).grid(row=7, column=1, pady=4)

# Active Note Sequence
Label(root, text="Active Note Sequence").grid(row=9, column=1)
current_sequence = Label(root, text="", bg="white", width=80)
current_sequence.grid(row=10, column=0, columnspan=3, pady=10)

# Command Builder
Label(root, text="Command Builder").grid(row=11, column=1)

action_options = ['no action', 'louder', 'softer', 'faster', 'Slower', 'loop']
active_action_options = StringVar()

# Behavior Selection
selected_behavior = StringVar()
selected_behavior.set("Command Selection")
behavior = OptionMenu(root, selected_behavior, *['play', 'preview', 'read', 'stop'])
behavior.grid(row=12, column=0, pady=4)
behavior.config(state=DISABLED)

# Action Selection
selected_action = StringVar()
selected_action.set("Action Selection")
action = OptionMenu(root, selected_action, *['', 'increase volume', 'decrease volume', 'increase speed', 'decrease speed', 'loop'])
action.grid(row=12, column=1, pady=4)
action.config(state=DISABLED)

# Parameter Selection
Label(root, text="by").grid(row=12, column=2, sticky=W, pady=4)
parameter = Entry(root, width=10)
parameter.grid(row=12, column=2,  pady=4)
metric = Label(root, text="").grid(row=12, column=3, sticky=W, pady=4)
parameter.config(state=DISABLED)

# Trace Selections
selected_behavior.trace("w", set_action_options)
selected_action.trace("w", set_paramerter_options)

# GO
Button(root, text='GO', command=command_pub).grid(row=12, column=4, pady=4)





# Button(root, text='Quit', command=root.quit).grid(row=4, column=0, sticky=W, pady=4)

bridge = CvBridge()

def note_sequence_callback(msg):
    current_sequence.config(text=msg.data)

def camera_callback(data):
    cv_img = bridge.imgmsg_to_cv2(data, desired_encoding='passthrough')
    img = ImageTk.PhotoImage(image = Image.fromarray(cv_img))
    camera.create_image(0,0, image=img)

def show_entry_fields():
   print("First Name: %s\nLast Name: %s" % (entry.cget('text'), current_sequence.get()))

def response_callback(msg):
    if msg.data == "Notes recognized.":
        behavior.config(state=NORMAL)
    elif msg.data == "No notes recognized.":
        action.config(state=DISABLED)

rospy.init_node('command_gui')

note_sequence_sub = rospy.Subscriber('behavior_node/note_sequence', String, note_sequence_callback)

response_sub = rospy.Subscriber('behavior_node/response', String, response_callback)

# camera_sub = rospy.Subscriber("cv_camera_node/image_raw", ROSImage, camera_callback, queue_size=1)

note_sequence_pub = rospy.Publisher('command_gui/note_sequence', String, queue_size=10)

command_pub = rospy.Publisher('speech_node/command', Command, queue_size=10)

mainloop( )
rospy.spin()