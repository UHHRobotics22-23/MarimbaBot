# TAMS Master Project 2022/2023 - Planning

### Test the motion in RViz / simulation environment
Launch the Demo Mode as described in the main README.md, for single mallet please run: 

```bash
roslaunch marimbabot_ur5_moveit_config demo.launch
```
for double mallet please run:
```bash
roslaunch marimbabot_ur5_flex_double_moveit_config demo.launch
```

And desired planning scripts can be called from marimbabot/scripts directory and run with several rosrun commands:

To perform custom motion planning passing arguments of octave and note sequence with maximum loudness:
```bash
rosrun marimbabot_planning dummy_client.py
```
example -> ( Octave : 4 , Sequence : C4, D2, A4, D4)


To perform timing and loudness of hitting motion. You can pass timing, note sequence, duration and loudness parameters as arguments into the terminal:
```bash
rosrun marimbabot_planning timing.py
```
sequence format -> Sequence : Note-duration[0,1]-loudness[0,1]
example -> (Octave : 5 , Sequence : C-0.5-0.8, D-0.8-0.2)


To perform pre-defined melodies or notes such as: 
Chromatic_scale
Brother_john
All_my_little_ducklings
Repeat_low_end
Repeat_high_end
Feather_pattern
All_white_keys
All_black_keys
Oh_my_chord
Black_and_white_chords
```bash
rosrun marimbabot_planning sound_pad.py
```
user can specify the melody index with numbers and desired Octave offest as well.
example of 2 Octave offest with Chromatic_scale -> 0 + 2


To perform custom motions and play some notes using keyboard buttons 


 Then start the planning package with:

```bash
 rosrun marimbabot_planning keyboard_player.py
```


To give motion commands to the robot with one of these commands:

```bash
roslaunch marimbabot_planning marimbabot.launch
```
enabled keyboard buttons are (q2w3er5t6z7u -  ysxdcvgbhnjm). Each keyboard button corresponds to a specific note on the marimba.

### Perform motion on real robot

In order to run the motion planning on the real robot, launch the bringup package that brings up the specifically motion_planning launch file:

```bash
roslaunch marimbabot_planning marimbabot.launch
```
and then same rosrun commands that were shown above can be performed as follows:

```bash

rosrun marimbabot_planning timing.py
rosrun marimbabot_planning sound_pad.py
rosrun marimbabot_planning dummy_client.py
rosrun marimbabot_planning keyboard_player.py

```

For further information, please look at the README.md in the scripts folder.