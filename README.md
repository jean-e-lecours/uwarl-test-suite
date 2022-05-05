# UWARL Test Suite

A collection of gazebo maps and scenarios with scripts that help run the simulations properly and extract data when needed.

Currently only ROS1 noetic is supported.

### Installation

These instructions assume that some linux distribution compatible with ROS is used. Make sure to replace `~/catkin_ws` by whatever the path is to your catkin workspace on your device.

```
cd ~/catkin_ws/src
git clone https://github.com/jean-e-lecours/uwarl-test-suite
cd ..
catkin_make
source /opt/ros/noetic/setup.bash
```

This will make the launchfiles and
