# fanuc-description package
![ROSVersion](https://img.shields.io/badge/ROS-melodic-blue)
![build](https://img.shields.io/badge/build-passed-success)

This package deals with the description of the Fanuc-m20iA robot according to the [```Fanuc_Robot_M-20iA``` datasheet](https://www.fanuc.co.jp/en/product/catalog/pdf/robot/RM-10iA(E)-07.pdf). To do this, a URDF file has been created which describes the fundamental characteristics of the robot joints, following the operations for calculating the Denavit-Hartenberg parameters, as reported in the ```doc/fanuc_20iA_DH.png``` file.
___
# Denavit-Hartenberg parameters for the Fanuc-m20iA

As we can see in the screen below, the DH parameters were first calculated and then the robot description file was created.

<img src="https://raw.githubusercontent.com/Bettorio/ros-robotics-fanuc-m20iA/main/src/fanuc_description/doc/fanuc-m20iA_DH.png" alt="drawing" width="600"/>

____
# Preview of the robot using rviz

![rviz](https://img.shields.io/badge/rviz-required-important)

Thanks to the rviz software we can see a brief preview of the fanuc robot followin the ```robot/fanuc.urdf``` file descriptor. To do so, it is necessary to launch the following command:
```bash
cd src/fanuc_description
roslaunch urdf_tutorial display.launch model:=robot/fanuc.urdf
```

Here what you should see:

<img src="https://raw.githubusercontent.com/Bettorio/ros-robotics-fanuc-m20iA/main/src/fanuc_description/doc/rviz_preview.png" alt="drawing" width="600"/>

___
##### Vittorio Fina