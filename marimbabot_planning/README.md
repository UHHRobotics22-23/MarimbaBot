# TAMS Master Project 2022/2023 - Planning

This part of the project contains all the code used for the trajectory generation.

## Testing

### Launch the robot
Launch the RViz only demo mode as described in the main README.md run

```bash
roslaunch marimbabot_ur5_flex_double_moveit_config demo.launch
```

otherwise, launch the full robot driver as described in the main README.md.


### Launch the planning
Launch the motion planning via

```bash
roslaunch marimbabot_planning marimbabot.launch
```

You can now send commands to the motion.

### Motion test scripts

For testing purposes, several scripts from the `marimbabot/scripts` directory can be called with the `rosrun` command.

#### Dummy Client

Use the following script to manually send a sequence of notes

```bash
rosrun marimbabot_planning dummy_client.py
```

example -> ( Octave : 4 , Sequence : C, <C#, D>, C#, A)

#### Timing

Use the following script to test the timing and loudness. You can pass timing, note sequence, duration, and loudness parameters

```bash
rosrun marimbabot_planning timing.py
```

```
sequence format -> Sequence : Note-duration[0,1]-loudness[0,1]
example -> (Octave : 5 , Sequence : C-0.5-0.8, D-0.8-0.2)
```

#### Sound pad

Run the following script to perform pre-defined note sequences such as
```
Chromatic_scale
Brother_john
All_my_little_ducklings
Repeat_low_end
Repeat_high_end
Feather_pattern
All_white_keys
All_black_keys
Black_and_white_chords
```

You can specify the melody index with numbers and the desired octave offset as well.

Note that these sequences do **not** include any timing information, and are therefore not the melodies you would expect.
This was never implemented. Use the predefined melodies in the behavior GUI instead.

```bash
rosrun marimbabot_planning sound_pad.py
```

#### Keyboard

Run this script to use your PC keyboard as a musical one. Note that the keyboard layout needs to be German for proper mapping.

```bash
 rosrun marimbabot_planning keyboard_player.py
```

### Experimental motion calibration

The `self_calibration_optimization_example.ipynb` notebook in the `scripts/` folder contains some preliminary code for an audio feedback-based self-calibration. 
The robot hits random points on the marimba and optimizes the pose of the instrument based on that information.

## Code structure

The `planning.cpp` file contains the main logic that handles the trajectory generation. It contains an action server that receives new hit sequences. They are converted into a joint space trajectory and executed. The robot is moved into an idle position if no action is performed. 
All utilities like the concatenation of two or more motion plans are part of the `utils.cpp` file.

Basic steps of the trajectory generation:

* The goals are received as a hit sequence by the action server. 
* The hitpoint for each note is estimated using the marimba model and tf.
* After we check for chords the hitpoints are assigned to a mallet (the left one is preferred).
* For each note inverse kinematics of the robot are solved so the mallet head(s) are at hitpoint, approach point, and retreat point while satisfying a number of additional contraints.
* The different key points are interpolated in the joint space resulting in the different subtrajectories.
* The sub-trajectories are timed to match the timing and loudness goals.
* All sub-trajectories are concatenated.
* The resulting trajectory is executed.
