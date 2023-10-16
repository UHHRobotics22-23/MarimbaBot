# Simulation package


## Overview

The simulation consists of a Gazebo world with a Marimbabot model, a node to detect contacts and generate MIDI, and optionally PlotJuggler to view contact velocities. The planning node is launched as well to allow the user to play the simulation using the action server. 


## Launch files

There are the following launch files:

* `marimbabot_gazebo.launch`: Launches the simulation in Gazebo.
* `marimbabot_full_sim.launch`: Launches the simulation in Gazebo, as well as nodes to detect contacts, publish the contact points, generate MIDI, and optionally launch PlotJuggler to view contact velocities.
  *This is the recommended launch file to use.*

The PlotJuggler can be toggled on/off by setting the appropriate bool flag in the if block in the 
[the launch file](launch/marimbabot_full_sim.launch).


## How to run

First, make sure you have completed the steps in the [main README](../README.md) and therefore have a working environment.

Launch the simulation with the following command:

```bash
roslaunch marimbabot_simulation marimbabot_full_sim.launch
```

The simulation can play any piece, and therefore needs to be fed a note string either manually using the action server, or using the `sound_pad.py` [script](../marimbabot_planning/scripts/sound_pad.py) in the `marimbabot_planning/scripts` package folder. E.g. to launch the Sound Pad script:

```bash
rosrun marimbabot_planning sound_pad.py
```


## MIDI generation

The simulation will record all bar hits (starting from the first hit to avoid a long silent interval in the beginning) and save them as a MIDI file in the `marimbabot_simulation/midi` package folder with current datetime as name. If the `marimbabot_simulation` directory cannot be found using `rospack find`, then the script falls back to the current directory. In any case, the MIDI directory will be logged to the output stream.

The MIDI file can be played as follows:

```bash
fluidsynth /usr/share/sounds/sf2/FluidR3_GM.sf2 <PATH/TO/MIDI_FILE>.midi -a alsa
```

In the above command, replace the path and `<FILENAME>` with the name of the MIDI file.

To convert a MIDI file to MP3, use the following command template:

```bash
fluidsynth -l -T raw -F - /usr/share/sounds/sf2/FluidR3_GM.sf2 <PATH/TO/MIDI_FILE>.midi | twolame -b 256 -r - <NAME>.mp3 
```


# TODO
- [ ] Try tilting the mallet holder to make sure the second mallet hits the bar
- [ ] detect_bar_contact: fix 2 notes hits at the same time