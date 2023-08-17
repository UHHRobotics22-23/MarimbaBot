# TAMS Master Project 2022/2023 - motion documentation

In order to run the whole project on the real robot, launch the bringup package that brings up the launch file for each package:

```bash
roslaunch marimbabot_bringup marimbabot.launch
```

To activate planning code run this command

```bash
rosrun marimbabot_planning planning
```
To give motion commands to the robot with one of these commands:

```bash

rosrun marimbabot_planning timing.py
rosrun marimbabot_planning sound_pad.py
rosrun marimbabot_planning dummy_client.py
rosrun marimbabot_planning keyboard_player.py

```
