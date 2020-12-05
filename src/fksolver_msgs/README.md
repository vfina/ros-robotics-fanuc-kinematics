# fksolver_msgs package

![ROSVersion](https://img.shields.io/badge/ROS-melodic-blue)
![build](https://img.shields.io/badge/build-passed-success)

This package implements messages, in particular services, which are used to compute the ```direct kinematics``` of a robot.
____

# The Service

The service made available is ```srv/FkCustomSolver.srv``` and contains the definition of the request and the response to be able to use it. The message defines a ```RobotState`` type request to be able to perform the necessary calculations from which to generate the requested service.
____
##### Vittorio Fina