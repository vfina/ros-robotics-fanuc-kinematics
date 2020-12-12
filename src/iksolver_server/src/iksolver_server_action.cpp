/* 
 * -------------------------------------------------------------------
 * This module has been developed as an exercise for Robotics course
 * @ UniSa.
 *
 * Title:   iksolver_server_action.cpp
 * Author:  Vittorio Fina
 * Date:    Dec 6, 2020
 *
 * This node implements the class defined in the header file in 
 * include/iksolver_server_action.h that declares the InverseKinematicsAction 
 * class for the inverse kinematics computation.
 *
 * -------------------------------------------------------------------
 */

#include <iksolver_server/iksolver_server_action.h>
#include <moveit/robot_state/conversions.h>
#include <angles/angles.h>

/**
 * Implementation of the constructor of the class 
 */
iksolver_server::InverseKinematicsAction::InverseKinematicsAction():
    action_server_(n_, "ik_solver", boost::bind(&iksolver_server::InverseKinematicsAction::ik_callback_, this, _1), false),
    robot_model_loader_(new robot_model_loader::RobotModelLoader("robot_description")),
    kinematic_model_(robot_model_loader_->getModel()),
    joint_model_group_(nullptr) {

    /* Check if the planning group name has been defined and get it from the parameter server */
    std::string planning_group_name;

    if(!n_.getParam("planning_group_name", planning_group_name)) {
        ROS_ERROR("'planning_group_name' is undefined on the parameter server");
        return;
    }

    /* Get the planning group */
    joint_model_group_ = kinematic_model_->getJointModelGroup(planning_group_name);

    /* Start the action server */
    action_server_.start();
}

iksolver_server::InverseKinematicsAction::~InverseKinematicsAction() {  
    /* Possible implementation of the destructor */
}

void iksolver_server::InverseKinematicsAction::ik_callback_(const iksolver_msgs::IkSolutionsGoalConstPtr & goal) {
    ROS_INFO("IK solver: Just Received the Goal");

    /* Retrieve the solver instance from the Joint Model Group */
    const kinematics::KinematicsBaseConstPtr solver = joint_model_group_->getSolverInstance();

    /*
     * Prepare the result to be sent back to the client. 
     */
    iksolver_msgs::IkSolutionsResult result;

    int ik_calls_counter = 0;

    /*
     * The condition defined here means that the algorithm must try at most
     * 2000 times in order to find the ik solutions and display them. 
     */
    while(ik_calls_counter < 2000 && ros::ok()) {
        std::vector<double> seed_state = generateSeedState_();
        std::vector<double> solution;
        moveit_msgs::MoveItErrorCodes error_code;

        /* 
         * The kinematics solver available from the kinematics base module takes in 
         * input the end effector pose and computes the set joint angles solution
         * that is able to reach that pose, in this case stored in 'solution'.
         */
        solver->getPositionIK(goal->end_effector_pose, seed_state, solution, error_code);

        /*
         * Check if the kinematics solver has found a solution and it is a new one.
         * If it is not then publish a feedback message for the client containing
         * the ik solution. 
         */
        if(error_code.val == moveit_msgs::MoveItErrorCodes::SUCCESS) {
            normalizeJointPositions_(solution);

            if(isSolutionNew_(solution)){
                ik_solutions_.push_back(solution);

                moveit::core::RobotState robot_state(kinematic_model_);

                robot_state.setVariablePositions(solution);

                iksolver_msgs::IkSolutionsFeedback feedback;

                moveit::core::robotStateToRobotStateMsg(robot_state, feedback.ik_solution);

                action_server_.publishFeedback(feedback);

                /* Store the solution in the result array of solutions */
                result.ik_solutions.push_back(feedback.ik_solution);
            }
        }

        ik_calls_counter++;
    }

    /* The last step sends the solutions in one shot (if any is found). */
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
    
    /*
     * Search algorithm that consumes the matrix of solutions in search of a match 
     * between them and returns the corresponding boolean. 
     */
    for(int i = 0; i < ik_solutions_.size(); i++) {
        bool are_equal = true;

        for(int j = 0; j < ik_solutions_[i].size() && are_equal; j++) {
            double diff;

            if(joint_model_group_->getActiveJointModels()[j]->getType() == robot_model::JointModel::REVOLUTE) {
                /* If the joint is a revolute one keep as a metrics the shortest angular distance */
                diff = angles::shortest_angular_distance(ik_solutions_[i][j], solution[j]);
            } else {
                /* Else simply make the difference */
                diff = ik_solutions_[i][j] - solution[j];
            }

            /* Float Absolute Value intended to be a tolerance value */
            if(std::fabs(diff) > 1e-3) {
                are_equal = false;
            }
        }

        if(are_equal) {
            return false;
        }
    }

    return true;
}

std::vector<double> iksolver_server::InverseKinematicsAction::generateSeedState_() const {
    std::vector<double> seed_state;
    
    std::vector<std::string> joint_names = kinematic_model_->getVariableNames();

    for(int i = 0; i < joint_names.size(); i++) {
        double ub = kinematic_model_->getURDF()->getJoint(joint_names[i])->limits->upper;
        double lb = kinematic_model_->getURDF()->getJoint(joint_names[i])->limits->lower;
        double span = ub - lb;
        
        seed_state.push_back((double)std::rand()/RAND_MAX * span + lb);
    }

    return seed_state;
}

void iksolver_server::InverseKinematicsAction::normalizeJointPositions_(std::vector<double> & solution) const {
    /*
     * Angle normalization  
     */
    for(int i = 0; i < solution.size(); i++) {
        if (joint_model_group_->getActiveJointModels()[i]->getType() == robot_model::JointModel::REVOLUTE) {
            solution[i] = angles::normalize_angle(solution[i]);
        }
    }
}

