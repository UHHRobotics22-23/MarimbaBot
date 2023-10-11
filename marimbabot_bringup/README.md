# TAMS Master Project 2022/2023 - Bringup

This package contains the launch files for the bringup of the robot.
The launch process is described in the top level [README](../README.md).

## Launch Files

### marimbabot_ur5_bringup.launch
This file is already configured to work with our setup.
However, the following parameters and code parts may be of interest. 

When using a wifi connection to the mallet, one can set the ip address and port here:

```xml
<group if="$(eval mallet_holder_type=='flex_double')">
            <!--> ... <-->
            <group if="$(arg mallet_joint_wifi)">
                <node name="mallet_hardware_control_node" pkg="marimbabot_hardware" type="mallet_hardware_control_node_wifi" output="screen">
                    <!-- Setting device parameter -->
                    <param name="address" value="192.168.42.1"/>
                    <param name="port" value="8888"/>
                </node>
            </group>
            <!--> ... <-->
</group>

```

The robot ip address can be set here:
```xml
<include file="$(find ur_robot_driver)/launch/ur_control.launch">
    <!-- Other arguments -->
    <arg name="robot_ip" value="192.168.1.12"/>
    <!-- Other arguments -->
</include>

```

### marimbabot_bringup.launch
Launches the bringup of the robot. It launches the following packages:

- marimbabot_vision
- marimbabot_speech
- marimbabot_behavior
- marimbabot_audio
- marimbabot_planning