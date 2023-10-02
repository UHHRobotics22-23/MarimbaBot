# TAMS Master Project 2022/2023

[![CI](https://github.com/UHHRobotics22-23/marimbabot/actions/workflows/main.yml/badge.svg)](https://github.com/UHHRobotics22-23/marimbabot/actions/workflows/main.yml)
[![ROS Version Noetic](https://img.shields.io/badge/ROS%20Version-Noetic-%2388b55a)](http://wiki.ros.org/noetic)

This repository contains the codebase for a [Marimba](https://en.wikipedia.org/wiki/Marimba) playing robot developed during the TAMS Master Project 2022/2023.
It is currently still "work in progress".

<p align="center">
  <img width="30%" src="https://github.com/UHHRobotics22-23/marimbabot/assets/15075613/277ce391-edd8-4c7a-8142-e50c12e855a2" alt="marimba playing robot" />
</p>

## Setup

The robot uses Ubuntu 20.04 and ROS noetic.
So make sure that it is installed on your system.
Visit the [ROS Wiki](http://wiki.ros.org/noetic/Installation) for further details. For Python version 3.8 is used.

Remember to source the ROS noetic installation in EACH terminal you use or add it once to your `.bashrc` file like this:

```bash
echo "source /opt/ros/noetic/setup.bash" >> ~/.bashrc
source ~/.bashrc
```

Make sure to [setup your ssh key](https://docs.github.com/en/authentication/connecting-to-github-with-ssh/adding-a-new-ssh-key-to-your-github-account) for easy synchronization without authorization.

We are using VCS for workspace management, so install it first:

```
sudo apt install python3-vcstool
```

Run the following commands:

```bash
# Create catkin workspace folder
mkdir -p catkin_ws/src

# Clone repository into source folder
cd catkin_ws/src
curl https://raw.githubusercontent.com/UHHRobotics22-23/marimbabot/main/workspace.repos | vcs import --recursive
cd ..

# Initialize workspace
catkin init

# Setup rosdep to download dependencies (only needed once after installing ROS)
sudo rosdep init

# Update rosdep packages list
rosdep update

# Install all dependencies needed by our repository
rosdep install --from-paths src --ignore-src -r -y
# Unfortunately this is needed as for unknown reasons the dependency is not recognized
rosdep install serial

# Install python dependencies
pip3 install wheel  # firstly install the wheel for further package building if you start from a empty virtual environment.
pip3 install -r src/marimbabot/requirements.txt

#install the precise engine
wget https://github.com/MycroftAI/mycroft-precise/releases/download/v0.3.0/precise-all_0.3.0_x86_64.tar.gz
tar -zxvf precise-all_0.3.0_x86_64.tar.gz -C src/marimbabot/marimbabot_speech/utils/kws/
rm precise-all_0.3.0_x86_64.tar.gz
```

Now you are ready to go.

## Update workspace

You might want to update the state of the software and get recent changes.
Normally a simple pull should be enough.

```bash
# Go to the repository
cd catkin_ws/src/marimbabot

# Pull recent changes
git pull
```

You sometimes also want to update the dependencies if new ones have been introduced. You can do this like this:

```bash
# Go to workspace
cd catkin_ws

# Update rosdep packages list
rosdep update

# Install new dependencies
rosdep install --from-paths src --ignore-src -r -y
# Unfortunately this is needed as for unknown reasons the dependency is not recognized
sudo apt install ros-noetic-serial

# Install new python dependencies
pip3 install --upgrade -r requirements.txt
```

## Build

To build the full software stack one needs to execute the following command in the `catkin_ws` folder or any of its subdirectories:

```bash
# Go to workspace
cd catkin_ws

catkin build
```

## Launch

Before you launch any of the provided launch files remember to source the current workspace by typing:

```bash
# Go to workspace
cd catkin_ws

source devel/setup.bash
```

In order to run the whole project on the real robot, launch the bringup package that brings up the launch file for each package:

```bash
roslaunch marimbabot_bringup marimbabot.launch
```


#### Note for development: Add the main launch files to the bringup if they are created.

To run the UR5 setup with the MoveIt Demo Mode run

```bash
roslaunch marimbabot_ur5_moveit_config demo.launch
```

you should be able to test simple planning things with it.

To view the marimba model in RViz run

```bash
roslaunch marimbabot_description marimba_rviz.launch
```

## Launch single packages

To launch a single package run this command each package:

```bash
roslaunch marimbabot_audio marimbabot.launch
# or
roslaunch marimbabot_planning marimbabot.launch
# or
roslaunch marimbabot_vision marimbabot.launch
```

This comes handy for purposes like debugging or testing.

## How do I contribute

To contribute changes to this repository go to the repository in your file system and create a new branch. Replace the placeholder `fix/my_dummy_branch` with a short but descriptive branch name:

```bash
git switch -c fix/my_dummy_branch
```

Now commit your changes and push them:

```bash
# Run this once if the repo is new. It will save you an additional command when pushing new branches.
git config push.autoSetupRemote true

# Now we can push our changes
git push
```

Now you can [create a pull request](https://docs.github.com/en/pull-requests/collaborating-with-pull-requests/proposing-changes-to-your-work-with-pull-requests/creating-a-pull-request#creating-the-pull-request) for your branch and merge it (after it is approved) into the `main` branch using the GitHub website.

## Project Overview

### Block Diagram
```mermaid
%%{init: {"flowchart": {"defaultRenderer": "elk"}} }%%
flowchart TD;
    subgraph vision
        cv_camera_node-->|/cv_camera_node/image_raw - sensor_msgs/Image| vision_node;
        vision_node-->|/vision_node/recognized_notes - std_msgs/String| visualization_node;
    end

    vision_node-->|/vision_node/recognized_notes - std_msgs/String| behavior_node;

    subgraph planning
        marimbabot_planning;
    end

    marimbabot_planning-->|MoveIt interface| mallet_hardware_control_node
    
    subgraph speech
        speech_node/speech_audio_capture-->|/speech_node/audio_stamped - audio_common_msgs/AudioDataStamped| speech_tmp_node;
        speech_tmp_node-->|/speech_node/audio_tmp - marimbabot_msgs/TmpFile| speech_stt_node;
        speech_stt_node-->|/speech_node/speech - marimbabot_msgs/Speech| speech_t2c_node;

        speech_tts_node;
    end

    speech_tts_node-->|/robotsound - sound_play/SoundRequest| soundplay_node;

    speech_t2c_node-->|/speech_node/command - marimbabot_msgs/Command| behavior_node;

    subgraph audio
        audio_from_lilypond;
        audio_node/note_audio_capture-->|/audio_node/audio_stamped - audio_common_msgs/AudioDataStamped| onset_detector
    
        onset_detector-->|/audio/onset_notes - marimbabot_msgs/NoteOnset| onset_midi
        onset_detector-->|/audio/onset_notes - marimbabot_msgs/NoteOnset| score_calculator
    end

    audio_from_lilypond-->|/robotsound - sound_play/SoundRequest| soundplay_node;

    subgraph behavior
        behavior_node;
    end

    behavior_node-->|/audio_from_lilypond - marimbabot_msgs/LilypondAudio - Action| audio_from_lilypond;

    behavior_node-->|/behavior_node/response - std_msgs/String| speech_tts_node;
    
    behavior_node-->|/hit_sequence - marimbabot_msgs/HitSequence - Action| marimbabot_planning;
    
    behavior_node-->|/audio/hit_sequence - marimbabot_msgs/HitSequence| score_calculator

    subgraph hardware
        mallet_hardware_control_node;
    end

```

## Declaration
The file [onset_detection.py](https://github.com/UHHRobotics22-23/marimbabot/blob/main/marimbabot_audio/scripts/onset_detection.py) and sloe the [messages](https://github.com/UHHRobotics22-23/marimbabot/blob/main/marimbabot_audio/msg) in the package marimbabot_audio is a modification of [@v4hn](https://github.com/v4hn)'s base [repository](https://github.com/TAMS-Group/music_perception.git).