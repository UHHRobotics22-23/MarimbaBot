# Marimababot Hardware
This package defines the ROS driver for the two mallet hardware.
Three central versions of the driver exist
- Wifi - The current version using Wifi and UDP to communicate with the holder
- Serial - First version using a serial (USB) connection to communicate with the device
- Dummy - Used in simulations or for debugging

Each of these versions has associated source, header and launch files identified by the suffix.

## General Design
Generally every version is composed of two source files servo_interface_XXXX.cpp and mallet_hardware_control_node_XXXX.cpp in the [src](src/) folder.

We use the packages [controller_manager](http://wiki.ros.org/controller_manager) and [ros_controllers](http://wiki.ros.org/ros_controllers) to implement a [ros_control](http://wiki.ros.org/ros_control) (standard) control interface with a standard joint_trajectory_controller from ros_controllers. See the [github repository of ros_controllers](https://github.com/ros-controls/ros_controllers).

The [guide on rosroboticslearning](https://www.rosroboticslearning.com/ros-control) gives a good starting point for understanding what has to be done, but additionally reading the source code of the packages and looking for other examples is necessary.

### Implementation
Consequently we only have to implement the hardware_interface::RobotHW (See the figure in the [ros_control wiki page](http://wiki.ros.org/ros_control)).
It is defined in the servo_interface_XXXX.cpp (and related header file).
The servo_interface_XXXX.cpp contains the main part of the implementation and facilitates the communication with the device and the handling of connection interuptions.
The hardware interface is defined as a position based controller (the servo is controlled by the angle) and gives the currently commanded position as the feedback. The commanded position is retrieved from the device but no measurements of the actual position are employed.

The mallet_hardware_control_node_XXXX.cpp initializes the ROS node and loads the parameters given to it. The ServoInterface implemented in servo_interface_XXXX.cpp is started here.
The control node further runs the controller manager update loop and calls the read and write functions of the ServoInterface.

### Launch files
The components are initialized in a launch file. The files in [launch](launch/) give examples for debugging purpuses.
Centrally the loading of the configuration yaml files and the parameters of the controller_spawner are important to the implementation.

### Configuration yaml
The [controllers.yaml](config/controllers.yaml) file sets the parameters for the standard controllers of the [ros_controllers](http://wiki.ros.org/ros_controllers) package. Important is the definition of the mallet_finger joint for the trajectory_controller.

[joint_limits.yaml](config/joint_limits.yaml) defines the speed, acceleration and position limits of the mallet_finger joint. These are ensured by a check in the ServoInterface write function.

## Version Specific Usage

The differing versions of the ServoInterface have specific configuration parameters and precoditions for normal operation.

### Wifi
The main implementation of the ServoInterface.
For normal operation the controlling computer needs to be connected to the wifi of the two mallet holder controller (Marimbabot_Mallet - PW: 12345678).

Two configuration options have to be set to connect to the UDP server running on the microcontroller.
- address: The ip of the UDP server (default: 192.168.42.1)
- port: The port of the UDP server (default: 8888)

To change the correct values of these parameters the firmware on the microcontroller has to be adjusted. The firmware is found in the [hardware_design](https://github.com/UHHRobotics22-23/hardware_design/tree/main/arduino_code/mallet_play_servo_wifi) repository.

### Serial
The serial hardware_interface was first developed and later replaced by the wireless wifi version.
The serial version needs a different version of the firmware flashed onto the microcontroller found in the [hardware_design](https://github.com/UHHRobotics22-23/hardware_design/tree/main/arduino_code/mallet_play_servo) repository.
The microcontroller has to be connected over usb to the controlling computer.

Two parameters have to be set:
- device: The device path in the unix file system (default: /dev/ttyACM0 or /dev/ttyUSB0)
- baud: The baud rate of the serial connection (default: 115200)

The servo interface can tollerate the device not being present at the start without crashing. In some cases the permissions of a new device have to be adjusted to allow the node to access the device.

    sudo chmod 666 /dev/ttyACM0

### Dummy
The dummy hardware_interface is built to emulate an actual device responding to the commands given by ros.
It is mainly used to facilitate the simulation of the project.

No parameters have to be set to use the interface.

## Debug
The launchfiles in [launch](launch/) are used to create a simple testing setup in which the mallet holder is know to ROS. It uses the dummy robot model [two_mallet_holder.urdf](urdf/two_mallet_holder.urdf) (no practical significance).

Further the [scripts](scripts/) folder contains the two scripts [joint_state.py](scripts/joint_state.py) and [wifi_client_sim.py](scripts/wifi_client_sim.py).

[joint_state.py](scripts/joint_state.py) publishes a dummy position of the two mallet holder which is always in the zero position. It can be used when the mallet holder is included in a bigger model but the control nodes are not running to make the system able to plan.
The [dummy control node](src/mallet_hardware_control_node_dummy.cpp) is a more realistic solution to the problem.

[wifi_client_sim.py](scripts/wifi_client_sim.py) is a simple "emulator" of a running two mallet assembly to test the [wifi control node](src/mallet_hardware_control_node_wifi.cpp) against. (Note: It hasn't been used for some time and the protocol might not be implemented 100% correctly anymore)