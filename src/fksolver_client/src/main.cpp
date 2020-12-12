/* 
 * -------------------------------------------------------------------
 * This module has been developed as an exercise for Robotics course
 * @ UniSa.
 *
 * Title:   main.cpp
 * Author:  Vittorio Fina
 * Date:    Dec 2, 2020
 *
 * This node implements a client that requests a service for the 
 * computation of the direct kinematics of a robot using the 
 * FkCustomSolver.srv service.
 *
 * -------------------------------------------------------------------
 */

#include "ros/ros.h"
#include "fksolver_msgs/FkCustomSolver.h"
#include <moveit/robot_model_loader/robot_model_loader.h>
#include <moveit/robot_state/robot_state.h>
#include <moveit/robot_model/robot_model.h>
#include <moveit/robot_state/conversions.h>
#include <moveit_msgs/GetPositionFK.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

using namespace std;

int main (int argc, char **argv) {

    /*
     * The ros::init() function need to see argc and argv in order 
     * to take eventually argoments from command line.
     * The third argument of the init() is the name of the node.
     * 
     * NodeHandle is the main access point to communication with ROS system.
     */
    ROS_INFO("Bring up fksolver_client node");
    ros::init(argc, argv, "fksolver_client");
    ros::NodeHandle n;

    /*
     * The serviceClient() function is how you tell ROS that you want to make a request
     * for a service available on the service server, which is in this case /fk_solver. 
     */
    ros::ServiceClient fk_client = n.serviceClient<fksolver_msgs::FkCustomSolver>("fk_solver");

    fksolver_msgs::FkCustomSolver service;

    /*
     * Preparing the request to send in order to receive the results.
     * The Robot Model is initialized via smart pointers and the model 
     * is retrieved via the getModel () function. 
     */
    robot_model_loader::RobotModelLoaderConstPtr robot_model_loader = std::shared_ptr<const robot_model_loader::RobotModelLoader>(new robot_model_loader::RobotModelLoader("robot_description"));
    robot_model::RobotModelConstPtr kin_model = robot_model_loader->getModel();
    moveit::core::RobotState robot_state(kin_model);

    /* Arbitrarily random values */
    const std::vector<double> joint_positions = {0, 1.5708, 0.002324, 0, -1.5708, 3.14159};

    /*
     * The client must check if the planning_group has been defined on the parameter server
     */
    std::string planning_group_name;

    if(!n.getParam("planning_group_name", planning_group_name)) {
        ROS_ERROR("planning_group_name is undefined on the parameter server!");
        return false;
    }

    const robot_state::JointModelGroup *joint_model_group = kin_model->getJointModelGroup(planning_group_name);

    robot_state.setJointGroupPositions(joint_model_group, joint_positions);

    /* 
     * This function allows you to convert the Robot State into a message 
     * to be sent as a request to the server  
     */
    moveit::core::robotStateToRobotStateMsg(robot_state, service.request.robot_state);

    /* Calling the service sending the request */
    if(!fk_client.call(service)) {
        ROS_ERROR("Error calling the service!");
    }

    /*
     * Prepare the response of the server in order to send it in stdout. 
     */
    tf2::Quaternion quaternion;
    tf2::fromMsg(service.response.end_effector_pose.orientation, quaternion);

    tf2::Matrix3x3 matrix(quaternion);
    tf2Scalar roll, pitch, yaw;
    matrix.getRPY(roll, pitch, yaw);

    std::ostringstream ss;

    ss << std::endl << "----------- Server Response for Custom Solver-----------" << std::endl;
    ss << "RPY = [" << roll << " " << pitch << " " << yaw << "]" << std::endl;
    ss << "--------------------------------------------------------" << std::endl;

    ROS_INFO_STREAM(ss.str());

    /*
     *  To compare the values ​​obtained with the service made available by MoveIt!
     *  we can subscribe the client to the default service /compute_fk.
     */
    fk_client = n.serviceClient<moveit_msgs::GetPositionFK>("compute_fk");

    std::vector<std::string> link_names = joint_model_group->getLinkModelNames();

    /*
     * To have a well-formed request to make to the MoveIt! we need the link 
     * vector of our robot and the RobotState. 
     * The frame_id in the header is the frame in which the forward kinematics
     * poses will be returned. That's why we pass the base_link.
     */
    moveit_msgs::GetPositionFK move_group_fk_service;
    move_group_fk_service.request.header.frame_id = link_names[0];
    move_group_fk_service.request.fk_link_names.push_back(link_names.back());
    moveit::core::robotStateToRobotStateMsg(robot_state, move_group_fk_service.request.robot_state);

    /* Calling the service sending the request */
    if(!fk_client.call(move_group_fk_service)) {
        ROS_ERROR("Error calling the service!");
    }

    /*
     * Prepare the response of the server in order to send it in stdout. 
     */
    tf2::fromMsg(move_group_fk_service.response.pose_stamped[0].pose.orientation, quaternion);

    tf2::Matrix3x3 rotation_matrix(quaternion);
    rotation_matrix.getRPY(roll, pitch, yaw);

    std::ostringstream msg;

    msg << std::endl << "----------- Server Response for move_group solver -----------" << std::endl;
    msg << "RPY = [" << roll << " " << pitch << " " << yaw << "]" << std::endl;
    msg << "-------------------------------------------------------------" << std::endl;
    
    ROS_INFO_STREAM(msg.str());
    
    ros::shutdown();

    return 0;
}