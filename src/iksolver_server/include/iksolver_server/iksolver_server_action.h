/* 
 * -------------------------------------------------------------------
 * This module has been developed as an exercise for Robotics course
 * @ UniSa.
 *
 * Title:   iksolver_server_action.h
 * Author:  Vittorio Fina
 * Date:    Dec 6, 2020
 *
 * This header declares the InverseKinematicsAction class and the prototypes
 * of the functions within it and all of its private attributes.
 *
 * -------------------------------------------------------------------
 */

#ifndef INCLUDE_IKSOLVER_SERVER_IKSOLVER_SERVER_ACTION_H_
#define INCLUDE_IKSOLVER_SERVER_IKSOLVER_SERVER_ACTION_H_

#include <ros/ros.h>
#include <iksolver_msgs/IkSolutionsAction.h>
#include <actionlib/server/simple_action_server.h>
#include <moveit/robot_model_loader/robot_model_loader.h>
#include <moveit/robot_state/robot_state.h>

namespace iksolver_server {

    class InverseKinematicsAction {

        public:
            /* Constructors definition */
            InverseKinematicsAction();
            ~InverseKinematicsAction();    

        private:
            /*
             * This funcions and attributes are understood as private because the class 
             * is not meant to have its methods and attributes called externally,
             * but everything is done within it.
             */
            void ik_callback_(const iksolver_msgs::IkSolutionsGoalConstPtr & goal);
            bool isSolutionNew_(const std::vector<double> & solution) const;
            std::vector<double> generateSeedState_() const;
            void normalizeJointPositions_(std::vector<double> & solution) const;

            ros::NodeHandle n_;
            
            /* The server must initialize the action_server */
            actionlib::SimpleActionServer<iksolver_msgs::IkSolutionsAction> action_server_;

            std::vector<std::vector<double>> ik_solutions_;
            robot_model_loader::RobotModelLoaderConstPtr robot_model_loader_;
            robot_model::RobotModelConstPtr kinematic_model_;
            const robot_state::JointModelGroup * joint_model_group_;

    };

} /* namespace iksolver_server */

#endif /* INCLUDE_IKSOLVER_SERVER_IKSOLVER_SERVER_ACTION_H_ */