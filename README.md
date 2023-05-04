# TAMS Master Project 2022/2023

This repository contains the codebase for a [Marimba](https://en.wikipedia.org/wiki/Marimba) playing robot developed during the TAMS Master Project 2022/2023.
It is currently still "work in progress".

## Declaration
The code of marimbabot_audio part are after the modification of this base [repository](https://github.com/TAMS-Group/music_perception.git), which is from [@v4hn](https://github.com/v4hn).

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

# Install python dependencies
pip3 install -r src/marimbabot/requirements.txt
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

In order to run the whole project, launch the bringup package that brings up the launch file for each package:

```bash
roslaunch marimbabot_bringup marimbabot.launch
```


#### Note for development: Add the main launch files to the bringup if they are created.

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
