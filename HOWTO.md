# Howto

## Demo

### 1. Build the project
After cloning the repository from github ([https://github.com/UHHRobotics22-23/MarimbaBot](https://github.com/UHHRobotics22-23/MarimbaBot)) the ros project has to be built according to the instructions in the [main readme](./README.md)

### 2. Setup the configuration
Additionally to the UR5, the following devices have to be connected and configured before launching the project:
1. Logitech StreamCam (packages marimbabot_vision and marimbabot_speech)
2. Scarlett 2i2 USB Audio Interface (package marimbabot_audio)
3. Wifi connection to the MalletHolder (package marimbabot_hardware)

##### Logitech StreamCam (required for packages marimbabot_vision and marimbabot_speech):
Change the parameter <i>device</i> of the node <i>audio_capture</i> in the [launch file](marimbabot_speech/launch/command_recognition.launch) of the package marimbabot_speech:

```bash
marimbabot_speech/launch/command_recognition.launch
```

and modify the <i>device_id</i> parameter in the [configuration file](marimbabot_vision/config/cv_camera.yaml) of the package marimbabot_vision:

```bash
marimbabot_vision/config/cv_camera.yaml
```

##### Scarlett 2i2 USB Audio Interface (required for package marimbabot_audio):

Adjust the <i>device</i> parameter for the <i>note_audio_capture</i> node in the [launch file](marimbabot_audio/launch/audio_feedback.launch) of the package marimbabot_audio:

```bash
marimbabot_audio/launch/audio_feedback.launch
```

### 3. Run a demo environment
In order to run the whole project on the real robot, one has to run two launch files. First, the launch file that sets up the robot and its hardware:

```bash
roslaunch marimbabot_bringup marimbabot_ur5_bringup.launch
```

Second, the launch file that brings up the launch file for each package:

```bash
roslaunch marimbabot_bringup marimbabot.launch
```

### 4. (Optional) rqt visualizations
*Honestly, I don't know how to do this - Tom*

## Hardware

### 1. Power Up the Hardware
The hardware is powered by a 7.4v lipo battery.
The connector is on the back.
Plugging the battery in is sufficient.
To check if the system is powered, test if the servo is holding its position.

### 2. Connect to the Wifi
After powering on, the holder emits a wifi network.
By default it is called '**Marimbabot_Mallet**' with the password '**12345678**'.
Connect the computer running the ros application to the wifi.

### 3. Place the Mallet Holder in the Gripper
The mallet holder is held from the top.
The shape of the top part fits the gripper base plate and should allow for only one, repeatable holding position.
For grabbing the holder use the predefined gripper positions in RViz.
First run the main bringup to initialize the necessary drivers.

    roslaunch marimbabot_bringup marimbabot_ur5_bringup.launch

RViz can then be started

    rviz

First use the gripper position *basic_open*.
Then hold the holder in to gripper and change it to *basic_closed*.

Note: When restarting the robot / ros the gripper sometimes opens and closes, be ready to catch it if necessary.