# iksolver_msgs package

![ROSVersion](https://img.shields.io/badge/ROS-melodic-blue)
![build](https://img.shields.io/badge/build-passed-success)

This package implements messages, in particular actions, which are used to compute the ```inverse kinematics``` of a robot.
____

# The Action

The action made available is ```action/IkSolutions.action``` and contains the definition of the goal and the feedback to be able to use it, moreover the result should be defined. The action defines a ```RobotState`` type result and feedback to be able to retrieve the Inverse Kinematics solutions one by one and, at the end, all the ones available.
____
##### Vittorio Fina