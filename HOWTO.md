# Howto - Demo

### Build the project
The project has to be built and setup according to the instructions in the [main readme](./README.md)

### Sensor Setup
Additionally to the UR5, the following devices have to be connected and configured before launching the project:
1. Logitech StreamCam (packages marimbabot_vision and marimbabot_speech)
2. Scarlett 2i2 USB Audio Interface (package marimbabot_audio)

#### Audio Feedback Microphone
For setting up the microphone regarding the audio feedback, we used a microphone and a sound card. **There is one thing to be careful of, you need to open the signal amplification at the soundcard, something even the red light is on, but it doesn't mean it really works, so better to reopen it, or check it by observing the system volume at the configuration of ubuntu system.**

Adjust the <i>device</i> parameter for the <i>note_audio_capture</i> node in the [launch file](marimbabot_audio/launch/audio_feedback.launch) of the package marimbabot_audio.

#### Note Reading Camera
The Logitech StreamCam is used for the detection of notes on the whiteboard.
Mount the camera on a tripod and point it to the whiteboard. Connect to the computer using USB.

Magnetic notes and note lines are provided in the hardware box.
Attach the note lines to the whiteboard before adding notes.

Change the parameter <i>device</i> of the node <i>audio_capture</i> in the [launch file](marimbabot_speech/launch/command_recognition.launch) of the package marimbabot_speech.

and modify the <i>device_id</i> parameter in the [configuration file](marimbabot_vision/config/cv_camera.yaml) of the package marimbabot_vision.

### Power Up the Mallet Holder
The mallet joint is powered by a 7.4v lipo battery.
The connector is on the back.
Plugging the battery in is sufficient.
To check if the system is powered, test if the servo is holding its position.

### Connect the  Mallet Holder to the Wifi
After powering on, the holder emits a wifi network.
By default, it is called '**Marimbabot_Mallet**' with the password '**12345678**'.
Connect the computer running the ROS application to the wifi.

### Place the Mallet Holder in the Gripper
The mallet holder is held from the top.
The shape of the top part fits the gripper base plate and should allow for only one, repeatable holding position.
For grabbing the holder use the predefined gripper positions in RViz.
First, run the main bringup to initialize the necessary drivers.

```bash
roslaunch marimbabot_bringup marimbabot_ur5_bringup.launch
```

Then RViz can then be started

```bash
rviz
```

First, select the `arm` planning group in the MoveIt Motion Planning RViz plugin and move the robot to the `marimbabot_home` goal state. 
This makes inserting the mallet holder easier.
Afterward, select the `gripper` planning group and move the gripper to the `basic_open` goal state.
Now you can insert the holder and move it to the `basic_closed` goal state.

**Note: When restarting the robot / ros the gripper sometimes opens and closes, be ready to catch it if necessary.**

### Calibration

While we could calibrate the marimba position by adapting the URDF description the following approach is often preferred due to its speed and simplicity.
The marimba in our environment is expected to be in a specific position and orientation.
We can command the robot to hit where it expects the middle of the highest and lowest keys and manually adjust the instrument's position accordingly.
The `repeat_low_end` and `repeat_high_end` commands of the following script can be used to hit these keys several times.

```bash
rosrun marimbabot_planning sound_pad.py
```

Repeating this process a few times with both ends of the instrument leads to a reasonably good positioning.

In the end, one might execute a `chromatic_scale` using the same script to verify all notes are hit correctly.

### Run a demo
In order to run the whole project on the real robot, one has to run two launch files. Start them if you haven't done it already due to an earlier step. First, the launch file that sets up the robot and its hardware:

```bash
roslaunch marimbabot_bringup marimbabot_ur5_bringup.launch
```

Second, the launch file that brings up the launch file for each package:

```bash
roslaunch marimbabot_bringup marimbabot.launch
```

### (Optional) rqt/rviz visualizations
There are several visualizations, basically, it just publishes the image to the ROS topic, you can observe the visualization with rviz or rqt by opening an image window. Following is the list of topics:
- **/audio_node/spectrogram_img**: the spectrum of the music notes detection
- **/audio_node/live_midi_img**: the live midi image of the music notes detection
- **/audio_node/feedback_img**: the final evaluation image of ground-truth and robot playing
- **cv_camera_node/image_raw**: the image captured by the camera, showing the whiteboard
- **detection_visualization**: the recognized music notation

### Usage

Now that everything is set up we are able to use the MarimbaBot.

The robot's wakeword is `Hi Marimbabot!`. 
The webcam mic is used for the speech detection, so don't move too far away from the whiteboard and microphone. Also don't stand in front of the whiteboard while the robot is reading the notes.

To play some notes of the whiteboard follow these steps:

- Say `Hi Marimbabot!`
- Wait for the activation sound
- Say `Read the notes`
- Wait for confirmation via TTS
- Say `Hi Marimbabot!`
- Wait for the activation sound
- Say `Play the music/notes`

Some other instructions that you might want to try are:

- Play faster/slower (by n bpm)
- Play louder/softer (by n steps)
- Play in a loop
- Stop playing
- Preview the piece

