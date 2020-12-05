# fanuc-moveit-config package

![ROSVersion](https://img.shields.io/badge/ROS-melodic-blue)
![build](https://img.shields.io/badge/build-passed-success)

This package was created thanks to the use of the [MoveIt!](http://docs.ros.org/en/kinetic/api/moveit_tutorials/html/doc/setup_assistant/setup_assistant_tutorial.html) assistant.
____

What allowed this assistant to do was generate the [Semantic Robot Description Format](http://wiki.ros.org/srdf) related to the fanuc robot.

# Robot Preview

After compiling the generated package it is possible to have a preview of the robot by executing the command shown below:

```bash
roslaunch fanuc_moveit_config demo.launch
```

A rviz screen will then open that will allow us to have a preview of the robot and with the freedom of being able to add models in the instrument panel for any further information.

Make sure that, referring to the ```demo.launch``` file we set the variable to use gui to ```True``` so we are able to move robot joints. In particular the line 22 of this file should be:
```xml
22  <arg name="use_gui" default="true" />
```
___
##### Vittorio Fina