# ROS Package to Teleoperate a P3-AT

This package comprises one way of implementing the exercise presented in the ROS [tutorial](https://github.com/mazrk7/tutorial_pub_sub) for the Human-Centred Robotics course taught at Imperial College London.

## Setup Instructions

Before launching this package, don't forget to run the following command to install the necessary dependencies:
```shell
rosdep install p3at_teleop
```

You will also need sudo permissions for read and write access to use the RS-232-USB adapter:
```shell
sudo chmod a+rw /dev/ttyUSB*
```

You may also need to change the joystick axes parameters ("axis\_linear" and "axis\_angular") depending on your controller. This can be done from the `.launch` file or directly from the command line. Ignore the feedback error message in red.

## General Use

The directory layout for this package is as follows:
- `config` contains the RViz configuration file
- `launch` contains a basic launch file for this package
- `src` contains the P3-AT teleoperation node

You can launch this package with RViz:
```shell
roslaunch p3at_teleop p3at_teleop.launch rviz:=true
```

**Note:** This visualisation setup is very basic at the moment as all the code makes use of the [p2os](http://wiki.ros.org/p2os) stack. However, for more extensive projects (e.g. making use of **sonar**) with the P3-AT or PeopleBot, then please migrate over to [ROSARIA](http://wiki.ros.org/ROSARIA) for a more complete ROS interface.
