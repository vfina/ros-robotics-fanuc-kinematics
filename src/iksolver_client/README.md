# iksolver_client package

![ROSVersion](https://img.shields.io/badge/ROS-melodic-blue)
![build](https://img.shields.io/badge/build-passed-success)

This package implements a ```client``` for requesting the inverse kinematics computation action for a robot. Through the use of a callback function the solutions are displayed one by one that they are retrieved and, once the server computes all of them, send in one shot at the end. 
___

# Output Preview

The design of the node allows to modify from code the coordinates of the ```pose``` of the ```end_effector``` that must be sent to server in order to compute the kinematics. In particular this block of code in the ```src/main.cpp``` file  must me modified:
```c++
goal.end_effector_pose.position.x = 1.0;
goal.end_effector_pose.position.y = 0.0;
goal.end_effector_pose.position.z = 1.0;
```
The launch ```command``` to see the client in action is:
```bash
rosrun iksolver_client iksolver_client 
```
Once the client is ready waits for the server to compute the calculation and shows the results in the following way:

```bash
[ INFO] [1607783983.028609395]: Inverse kinematics request sent to the IK resolution action server
[ INFO] [1607783983.030646725]: Received Inverse Kinematic solution: [1.76958e-06, 0.101181, 0.561648, -3.14158, 2.23362, 1.24933e-05]
[ INFO] [1607783983.042274931]: Received Inverse Kinematic solution: [1.78187e-06, 0.101182, 0.561647, 4.02077e-10, -2.23362, 3.14159]
[ INFO] [1607783984.647545427]: Goal achieved. Found 2 IK solutions
[1.76958e-06, 0.101181, 0.561648, -3.14158, 2.23362, 1.24933e-05]
[1.78187e-06, 0.101182, 0.561647, 4.02077e-10, -2.23362, 3.14159]

[ INFO] [1607783984.650040960]: Loading robot model 'fanuc'...
[ INFO] [1607783984.650086986]: No root/virtual joint specified in SRDF. Assuming fixed joint
[ INFO] [1607783984.834102590]: Publishing solutions . . .
[ INFO] [1607783988.834489241]: All solutions published
```
___
##### Vittorio Fina