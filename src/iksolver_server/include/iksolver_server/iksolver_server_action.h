#ifndef INCLUDE_IKSOLVER_SERVER_IKSOLVER_SERVER_ACTION_H_
#define INCLUDE_IKSOLVER_SERVER_IKSOLVER_SERVER_ACTION_H_

#include <ros/ros.h>
#include <iksolver_msgs/IkSolutionsAction.h>
#include <actionlib/server/simple_action_server.h>
#include <moveit/robot_model_loader/robot_model_loader.h>
#include <moveit/robot_state/robot_state.h>

namespace iksolver_server
{

class InverseKinematicsAction
{

public:
    InverseKinematicsAction();
    ~InverseKinematicsAction();    

private:
    void ik_callback_(const iksolver_msgs::IkSolutionsGoalConstPtr & goal);
    bool isSolutionNew_(const std::vector<double> & solution) const;
    std::vector<double> generateSeedState_() const;
    void normalizeJointPositions_(std::vector<double> & solution) const;

    ros::NodeHandle n_;
    
    actionlib::SimpleActionServer<iksolver_msgs::IkSolutionsAction> action_server_;

    std::vector<std::vector<double>> ik_solutions_;
    robot_model_loader::RobotModelLoaderConstPtr robot_model_loader_;
    robot_model::RobotModelConstPtr kinematic_model_;
    const robot_state::JointModelGroup * joint_model_group_;
};

} // namespace iksolver_server

#endif //  INCLUDE_IKSOLVER_SERVER_IKSOLVER_SERVER_ACTION_H_