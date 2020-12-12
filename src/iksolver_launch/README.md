# iksolver_launch package

![ROSVersion](https://img.shields.io/badge/ROS-melodic-blue)
![build](https://img.shields.io/badge/build-passed-success)

This package implements a launch file to use the inverse kinematics computation action server and to correctly set the ```planning_group_name``` parameter on the parameter server.
Furthermore, the launch file loads the ```robot_description``` (in the current case it refers to the fanuc robot) and starts the node for the publication of the robots state in order to view the poses on rviz.

*N.B This launch file **only** calls ```iksolver_server``` and **not** ```iksolver_client```. For a correct display of the client side output it has been assumed that the client can be launched on another terminal.*
___
##### Vittorio Fina