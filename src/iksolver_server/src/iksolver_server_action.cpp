#include <iksolver_server/iksolver_server_action.h>
#include <moveit/robot_state/conversions.h>
#include <angles/angles.h>

iksolver_server::InverseKinematicsAction::InverseKinematicsAction():
    action_server_(n_, "ik_solver", boost::bind(&iksolver_server::InverseKinematicsAction::ik_callback_, this, _1), false),
    robot_model_loader_(new robot_model_loader::RobotModelLoader("robot_description")),
    kinematic_model_(robot_model_loader_->getModel()),
    joint_model_group_(nullptr)
{
    // Get the planning group name from the parameter server
    std::string planning_group_name;

    if(!n_.getParam("planning_group_name", planning_group_name)) {
        ROS_ERROR("'planning_group_name' is undefined on the parameter server");
        return;
    }

    // Get the planning group
    joint_model_group_ = kinematic_model_->getJointModelGroup(planning_group_name);

    action_server_.start();
}

iksolver_server::InverseKinematicsAction::~InverseKinematicsAction() {  
    // pass
}

void iksolver_server::InverseKinematicsAction::ik_callback_(const iksolver_msgs::IkSolutionsGoalConstPtr & goal) {
    ROS_INFO("IK solver: Just Received the Goal");

    // Joint model group solver instance
    const kinematics::KinematicsBaseConstPtr solver = joint_model_group_->getSolverInstance();

    iksolver_msgs::IkSolutionsResult result;

    int ik_calls_counter = 0;

    while(ik_calls_counter < 2000 && ros::ok()) {
        std::vector<double> seed_state = generateSeedState_();
        std::vector<double> solution;
        moveit_msgs::MoveItErrorCodes error_code;

        solver->getPositionIK(goal->end_effector_pose, seed_state, solution, error_code);

        if(error_code.val == moveit_msgs::MoveItErrorCodes::SUCCESS) {
            normalizeJointPositions_(solution);

            if(isSolutionNew_(solution)){
                ik_solutions_.push_back(solution);

                moveit::core::RobotState robot_state(kinematic_model_);

                robot_state.setVariablePositions(solution);

                iksolver_msgs::IkSolutionsFeedback feedback;

                moveit::core::robotStateToRobotStateMsg(robot_state, feedback.ik_solution);

                action_server_.publishFeedback(feedback);

                result.ik_solutions.push_back(feedback.ik_solution);
            }
        }

        ik_calls_counter++;
    }

    if(ik_solutions_.size() == 0) {
        action_server_.setAborted(result, "Could not find any IK solution");
    } else {
        std::ostringstream ss;
        ss << "Found " << ik_solutions_.size() << " IK solutions";
        
        action_server_.setSucceeded(result, ss.str());
    }

    ik_solutions_.resize(0);
}

bool iksolver_server::InverseKinematicsAction::isSolutionNew_(const std::vector<double> & solution) const {
    
    for(int i = 0; i < ik_solutions_.size(); i++) {
        bool are_solutions_equal = true;

        for(int j = 0; j < ik_solutions_[i].size() && are_solutions_equal; j++) {
            double diff;

            if(joint_model_group_->getActiveJointModels()[j]->getType() == robot_model::JointModel::REVOLUTE) {
                diff = angles::shortest_angular_distance(ik_solutions_[i][j], solution[j]);
            } else {
                diff = ik_solutions_[i][j] - solution[j];
            }

            if(std::fabs(diff) > 1e-3) {
                are_solutions_equal = false;
            }
        }

        if(are_solutions_equal) {
            return false;
        }
    }

    return true;
}

std::vector<double> iksolver_server::InverseKinematicsAction::generateSeedState_() const {
    std::vector<double> seed_state;
    
    std::vector<std::string> joint_names = kinematic_model_->getVariableNames();

    for(int i=0; i < joint_names.size(); i++) {
        double ub = kinematic_model_->getURDF()->getJoint(joint_names[i])->limits->upper;
        double lb = kinematic_model_->getURDF()->getJoint(joint_names[i])->limits->lower;
        double span = ub-lb;
        
        seed_state.push_back((double)std::rand()/RAND_MAX * span + lb);
    }

    return seed_state;
}

void iksolver_server::InverseKinematicsAction::normalizeJointPositions_(std::vector<double> & solution) const {
    for(int i=0; i < solution.size(); i++) {
        if (joint_model_group_->getActiveJointModels()[i]->getType() == robot_model::JointModel::REVOLUTE) {
            solution[i] = angles::normalize_angle(solution[i]);
        }
    }
}

