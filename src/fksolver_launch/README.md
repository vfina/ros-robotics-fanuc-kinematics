# fksolver_launch package

![ROSVersion](https://img.shields.io/badge/ROS-melodic-blue)
![build](https://img.shields.io/badge/build-passed-success)

This package implements a launch file to use the direct kinematics computation service and to correctly set the ```planning_group_name``` parameter on the parameter server.

*N.B This launch file **only** calls ```fksolver_server``` and **not** ```fksolver_client```. For a correct display of the client side output it has been assumed that the client can be launched on another terminal.*
___
##### Vittorio Fina