# ROS Package to Teleoperate a P3-AT

This package comprises one way of implementing the exercise presented in the ROS [tutorial](https://github.com/mazrk7/tutorial_pub_sub) for the Human-Centred Robotics course taught at Imperial College London.

Before launching this package, don't forget to run the following command to install the necessary dependencies:
```shell
rosdep install p3at_teleop
```

The directory layout for this package is as follows:
- `config` contains the RViz configuration file
- `launch` contains a basic launch file for this package
- `src` contains the P3-AT teleoperation node

You can launch this package with RViz:
```shell
roslaunch p3at_teleop p3at_teleop.launch rviz:=true
```

**Note:** This visualisation setup is very basic at the moment as all the code makes use of the [p2os](http://wiki.ros.org/p2os) stack, however for more extensive projects involving mobile bases like the P3-AT or PeopleBot, then please migrate over to [ROSARIA](http://wiki.ros.org/ROSARIA) for a more up to date ROS interface.
