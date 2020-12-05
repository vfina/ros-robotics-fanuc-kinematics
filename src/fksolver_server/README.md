# fksolver_client package

![ROSVersion](https://img.shields.io/badge/ROS-melodic-blue)
![build](https://img.shields.io/badge/build-passed-success)

This package implements a ```server``` for computing the direct kinematics of a robot.
___

# Output Preview

This node does not produce a true output, but simply responds to the service when called by responding with the direct kinematics referred to the last link of the robot, the ```end-effector```.


*N.B Before launching the node make sure that the ```/planning_group_name``` parameter has been defined on the parameter server*

___
##### Vittorio Fina