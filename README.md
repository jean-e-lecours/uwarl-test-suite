# UWARL Test Suite

A collection of gazebo maps and scenarios with scripts that help run the simulations properly and extract data when needed.

Currently only ROS1 noetic is supported.

### Dependencies

The turtlebot3 package. On debian-based distros, this can be installed using the following commands:

```
sudo apt update
sudo apt install -y ros-noetic-turtlebot3
```

### Installation

These instructions assume that some linux distribution compatible with ROS is used. Make sure to replace `~/catkin_ws` by whatever the path is to your catkin workspace on your device.

```
cd ~/catkin_ws/src
git clone https://github.com/jean-e-lecours/uwarl-test-suite
cd ..
catkin_make
source /opt/ros/noetic/setup.bash
```

The launchfiles will then become available from the command line through the package name `uwarl_test_suite`.

To run the turtlebot simulations, the following should be added to your `~/.bashrc` or `~/.zshrc` depending on which one you use:

```
export TURTLEBOT_MODEL=burger
```

### Example

Currently the only test course available is test course 1. To launch the gazebo simulation with the turtlebot 3 burger, navigate to your catking workspace directory and use the following command:

```
roslaunch uwarl_test_suite turtle_test_1.launch
```

From there on, scripts and nodes can be ran to give instructions to the turtlebot 3 as usual. Note that on this test course, the robot *can* fall off into the void!
