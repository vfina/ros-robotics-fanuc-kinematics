/* 
 * -------------------------------------------------------------------
 * This module has been developed as an exercise for Robotics course
 * @ UniSa.
 *
 * Title:   main.cpp
 * Author:  Vittorio Fina
 * Date:    Dec 6, 2020
 *
 * This node implements an action server for computing the inverse kinematics
 * of a robot simply calling an external constructor specially implemented 
 * for this task. 
 * 
 * -------------------------------------------------------------------
 */

#include <iksolver_server/iksolver_server_action.h>

int main(int argc, char **argv) {

    /*
     * The ros::init() function need to see argc and argv in order 
     * to take eventually argoments from command line.
     * The third argument of the init() is the name of the node.
     */
    ROS_INFO("Bring up the ikaction_server node");
    ros::init(argc, argv, "ikaction_server");
    
    /*
     * Calling the constructor of the InverseKinematicsAction class
     */
    iksolver_server::InverseKinematicsAction ik_action;

    ROS_INFO("Started inverse kinematics action server");

    /* Keep the node alive and ready to receive other goals */
    ros::spin();

    ros::shutdown();
    return 0;
}