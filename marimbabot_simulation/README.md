# Simulation package

## Launch files

There are the following launch files:

* `marimbabot_gazebo.launch`: Launches the simulation in Gazebo.
* `marimbabot_full_sim.launch`: Launches the simulation in Gazebo, as well as nodes to detect contacts and publish the contact points, and generate MIDI.


## How to run

Launch the simulation with the following command:

```bash
roslaunch marimbabot_simulation marimbabot_full_sim.launch
```

The MIDI file will be saved in the `marimbabot_simulation/midi` package folder with current datetime as name.

The MIDI file can be played as follows:

```bash
fluidsynth /usr/share/sounds/sf2/FluidR3_GM.sf2 catkin_ws/src/marimbabot/marimbabot_simulation/midi/<FILENAME>.midi -a alsa
```