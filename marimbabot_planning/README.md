# TAMS Master Project 2022/2023 - Planning

### Test the motion in RViz / simulation environment
Launch the Demo Mode as described in the main README.md. Then start the planning package with:

```bash
roslaunch marimbabot_planning marimbabot.launch
```


To give motion commands to the robot with one of these commands:

```bash

rosrun marimbabot_planning timing.py
rosrun marimbabot_planning sound_pad.py
rosrun marimbabot_planning dummy_client.py
rosrun marimbabot_planning keyboard_player.py

```

For further information, please look at the README.md in the scripts folder.