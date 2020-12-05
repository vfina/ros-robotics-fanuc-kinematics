# ros-robotics-fanuc-kinematics

![ROSVersion](https://img.shields.io/badge/ROS-melodic-blue)
![build](https://img.shields.io/badge/build-passed-success)

This package deals with showing in rviz the model of the Fanuc-RM-20iA and compute the forward and the inverse kinematics of the robot.

Implemented with [ROS (Robot Operating System)](http://wiki.ros.org/) and C++.  
___
# The task
The following is the task for which this repository was developed.

> Implement a service server that computes the direct kinematics of a robot and a service client that uses this service and prints the solution to stdout.
> 
> Verify the result by comparing it with the service /compute_fk of the move_group node.
>
> Implement an action server that computes all the inverse kinematic solutions of a robot (one by one and an action client that uses this action and prints the solutions to stdout (one by one, as their are received).The action server does not send the same solution twice and stops when all solutions have been found. At that time, they are returned all together.
>
> Repeat the experiment at point 3 by neglecting joint limits.
>
> Visualize the IK solutions in RViz.
___

##### Vittorio Fina