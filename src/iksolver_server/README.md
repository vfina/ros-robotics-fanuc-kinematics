# iksolver_server package

![ROSVersion](https://img.shields.io/badge/ROS-melodic-blue)
![build](https://img.shields.io/badge/build-passed-success)

This package implements a ```server``` for computing the inverse kinematics of a robot. The implementation is done relying on a custom class called ```InverseKinematicsAction``` defined in the same package inside the ```include``` folder. The ```main.cpp``` simply initialize the server node and calls the constructor of the ```InverseKinematicsAction``` class that takes care of instantiate the ```NodeHandle``` and all the functions necessary for calculating the inverse kinematics and sending the results to the client. 

This particular implementation allows to treat the module as an ```'off-the-shelf'``` and make it available not only for the fanuc robot for which this package was implemented.
___

# Output Preview

This node does not produce a true output, but simply responds to the action when called by responding with the inverse kinematics referred to the last link of the robot, the ```end-effector```. It display some messages in output about info of the action called.

The command to call the server in ROS is:
```bash
rosrun iksolver_server iksolver_server
```

*N.B Before launching the node make sure that the ```/planning_group_name``` parameter has been defined on the parameter server*

___
##### Vittorio Fina