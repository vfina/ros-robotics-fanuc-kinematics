/* 
 * -------------------------------------------------------------------
 * This module has been developed as an exercise for Robotics course
 * @ UniSa.
 *
 * Title:   main.cpp
 * Author:  Vittorio Fina
 * Date:    Dec 2, 2020
 *
 * This node implements a service that computes a service for the 
 * direct kinematics of a robot through the FkCustomSolver.srv service.
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
#include <eigen_conversions/eigen_msg.h>

using namespace std;

/* Service callback prototype */
bool computeFkSolution(fksolver_msgs::FkCustomSolverRequest &req, fksolver_msgs::FkCustomSolverResponse &res);

int main (int argc, char **argv) {

    /*
     * The ros::init() function need to see argc and argv in order 
     * to take eventually argoments from command line.
     * The third argument of the init() is the name of the node.
     * 
     * NodeHandle is the main access point to communication with ROS system.
     */
    ROS_INFO("Bring up fksolver_server node");
    ros::init(argc, argv, "fksolver_server");
    ros::NodeHandle n;
    
    /*
     * The adveritseService() function is a how you tell ROS that you want to make a server
     * which is able of answering requests for the FkCustomSolver. 
     */
    ros::ServiceServer service = n.advertiseService("fk_solver", computeFkSolution);

    ROS_INFO("Started service ...");

    ros::spin();

    return 0;
}

/* Service callback implementation */
bool computeFkSolution(fksolver_msgs::FkCustomSolverRequest &req, fksolver_msgs::FkCustomSolverResponse &res) {
    
    /*
     * In order to compute the direct kinematics we need to extract the RobotState
     * coming from the request and we can do it through the function robot_state()
     * and convert the request into the RobotState desired with the function
     * robotStateMsgToRobotState(). 
     */
    robot_model_loader::RobotModelLoaderConstPtr robot_model_loader = std::shared_ptr<const robot_model_loader::RobotModelLoader>(new robot_model_loader::RobotModelLoader("robot_description"));

    robot_model::RobotModelConstPtr kin_model = robot_model_loader->getModel();

    moveit::core::RobotState robot_state(kin_model);

    moveit::core::robotStateMsgToRobotState(req.robot_state, robot_state);

    robot_state.updateLinkTransforms();

    ros::NodeHandle n;

    /*
     * The client must check if the planning_group has been defined on the parameter server
     */
    std::string planning_group_name;

    if(!n.getParam("planning_group_name", planning_group_name)){
        ROS_ERROR("planning_group_name is undefined on the parameter server!");
        return false;
    }

    /*
     * Information on the robot links is obtained from the planning_group_name. 
     */
    const robot_state::JointModelGroup *joint_model_group = kin_model->getJointModelGroup(planning_group_name);

    std::vector<std::string> link_names = joint_model_group->getLinkModelNames();

    /*
     * Through function getGlobalLinkTransform() it is possible to obtain direct 
     * kinematics referring to the end effector of the robot and then respond to 
     * the client by calling the poseEigentoMsg function to modify the response 
     * and send it.
     */
    Eigen::Isometry3d forward_kinematics = robot_state.getGlobalLinkTransform(link_names.back());

    tf::poseEigenToMsg(forward_kinematics, res.end_effector_pose);

    return true;

}
