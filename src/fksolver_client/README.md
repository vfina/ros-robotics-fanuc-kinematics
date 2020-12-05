# fksolver_client package

![ROSVersion](https://img.shields.io/badge/ROS-melodic-blue)
![build](https://img.shields.io/badge/build-passed-success)

This package implements a ```client``` for requesting a direct kinematics computation service of a robot. In the first case, a custom solver is used and then the results are compared with the MoveIt! Service ```/compute_fk```.
___

# Output Preview

The design of the node allows to modify from code the vector of the joint parameters declared as ```joint_positions``` to compute its direct kinematics. In any case, the result can be obtained on the screen as follows:

```bash
[ INFO] [1607188205.373539040]: 
----------- Server Response for Custom Solver-----------
RPY = [-3.13886 1.56848 -3.13887]
--------------------------------------------------------

[ INFO] [1607188205.375134764]: 
----------- Server Response for move_group solver -----------
RPY = [-3.13886 1.56848 -3.13887]
-------------------------------------------------------------
```

*N.B Before launching the node make sure that the ```/planning_group_name``` parameter has been defined on the parameter server*

___
##### Vittorio Fina