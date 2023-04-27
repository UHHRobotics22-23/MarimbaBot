## Marimbabot Simulation package


### URDF/Xacro Model

The URDF model is generated using `xacro`, with the template `urdf/marimba.urdf.xacro`. See `marimba_display.launch` to see how it's run for RViz.


### RViz

To see the model in RViz, [prepare your environment for the project](../README.md#setup) and run:

```bash
roslaunch marimbabot_description marimba_display.launch
```

![Marimba](extra/marimba_sim.png)


### double_flex_mallet
 This model has a flexibility in two sides. It serves to make the sound on the marimba louder and at the same time ensure that the mallet does not break.