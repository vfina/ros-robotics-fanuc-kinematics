/* 
 * -------------------------------------------------------------------
 * This module has been developed as an exercise for Robotics course
 * @ UniSa.
 *
 * Title:   main.cpp
 * Author:  Vittorio Fina
 * Date:    Dec 6, 2020
 *
 * This node implements a client that requests an action for the 
 * computation of the inverse kinematics of a robot using the 
 * IkSolutions.action action.
 *
 * -------------------------------------------------------------------
 */

#include <condition_variable>
#include <ros/ros.h>
#include <actionlib/client/simple_action_client.h>
#include <iksolver_msgs/IkSolutionsAction.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <sensor_msgs/JointState.h>
#include <moveit/robot_model_loader/robot_model_loader.h>

/* Function prototypes */
void handleGoalCompletionEvent(const actionlib::SimpleClientGoalState & state, const iksolver_msgs::IkSolutionsResultConstPtr & result);
void handleGoalActiveEvent();
void handleFeedbackEvent(const iksolver_msgs::IkSolutionsFeedbackConstPtr & feedback);
void serializeIKSolution(std::ostringstream & ss, const moveit_msgs::RobotState & robot_state);

std::mutex mutex;
std::condition_variable result_handled;

int main(int argc, char **argv) {
    
    /*
     * The ros::init() function need to see argc and argv in order 
     * to take eventually argoments from command line.
     * The third argument of the init() is the name of the node.
     */
    ROS_INFO("Bring up ikaction_client node");
    ros::init(argc, argv, "ikaction_client");
    
    /*
     * The client() function is how you tell ROS that you want to initialize a client
     * for an action available on the action server, which is in this case /ik_solver. 
     */
    actionlib::SimpleActionClient<iksolver_msgs::IkSolutionsAction> client("ik_solver", true);

    /*
     * The client must wait the server to be up. 
     */
    client.waitForServer();
    
    iksolver_msgs::IkSolutionsGoal goal;
    /*
     * Prepare the goal to be sent.
     * Coordinates can be changed arbitrarily. 
     * 
     * Setting the RPY we can say to the server about rotations we desire.
     */
    goal.end_effector_pose.position.x = 0.5;
    goal.end_effector_pose.position.y = 0.7;
    goal.end_effector_pose.position.z = 2.0;
    
    tf2::Quaternion quaternion;
    quaternion.setRPY(0.0, 0.0, 0.0);
    
    goal.end_effector_pose.orientation = tf2::toMsg(quaternion);
    
    /* Send the goal and intercept the events with the related callbacks */
    client.sendGoal(goal, &handleGoalCompletionEvent, &handleGoalActiveEvent, &handleFeedbackEvent);

    /* Considered a standard resolution time of inverse kinematics. */
    if(!client.waitForResult(ros::Duration(30.0))) {
        ROS_ERROR("The Ik Solver did not complete in the desired time");    
    }

    std::unique_lock<std::mutex> lock(mutex);
    result_handled.wait(lock);

    ros::shutdown();
    return 0;
}

void handleGoalCompletionEvent(const actionlib::SimpleClientGoalState & state, const iksolver_msgs::IkSolutionsResultConstPtr & result) {
    std::ostringstream ss;

    /*
     * Check in case the Goal has been reached or not by the action server. 
     */
    if(state == actionlib::SimpleClientGoalState::StateEnum::SUCCEEDED) {
        int num_of_solutions = result->ik_solutions.size();

        ss << "Goal achieved. " << state.getText() << ", showing them:" << std::endl;

        for(int i = 0; i < num_of_solutions; i++) {
            serializeIKSolution(ss, result->ik_solutions[i]);
            ss << std::endl;
        }

        ROS_INFO_STREAM(ss.str());

        ros::NodeHandle n;

        /* Instantiate the joint state publisher publishing on the joint_states topic */
        ros::Publisher joint_state_publisher = n.advertise<sensor_msgs::JointState>("joint_states", 1);

        /* Load the robot model */
        robot_model_loader::RobotModelLoaderConstPtr robot_model_loader = std::shared_ptr<const robot_model_loader::RobotModelLoader>(new robot_model_loader::RobotModelLoader("robot_description"));

        /* Get the robot kinematic model */
        robot_model::RobotModelConstPtr kinematic_model = robot_model_loader->getModel();

        /* Check if the planning group name has been defined and get it from the parameter server */
        std::string planning_group_name;
    
        if(!n.getParam("planning_group_name", planning_group_name)) {
            ROS_ERROR("'planning_group_name' is undefined on the parameter server");
        }

        /* Get the planning group */
        const robot_state::JointModelGroup * joint_model_group = kinematic_model->getJointModelGroup(planning_group_name);

        sensor_msgs::JointState joint_state_msg;
        joint_state_msg.name = joint_model_group->getVariableNames();

        ROS_INFO("Publishing solutions . . .");

        ros::Duration sleep_time(2.0);

        /*
         * The goal here is to publish the solutions in the joint state publisher in order
         * to show the robot pose in rviz. So the message is prepared managing the position
         * field of the message, filled with positions of the joints of the robot.
         */
        for(int i = 0; i < num_of_solutions; i++){
            sleep_time.sleep();

            joint_state_msg.position = result->ik_solutions[i].joint_state.position;
            joint_state_msg.header.stamp = ros::Time::now();

            joint_state_publisher.publish(joint_state_msg);
        }       

        ROS_INFO("All solutions published");        
    } else {
        /* Goal not reached case */
        ss << "Goal aborted. " << state.getText();
        ROS_INFO_STREAM(ss.str());
    }    

    result_handled.notify_all();
}

void handleGoalActiveEvent() {
    ROS_INFO("Inverse kinematics request sent to the IK resolution action server");
}

void handleFeedbackEvent(const iksolver_msgs::IkSolutionsFeedbackConstPtr & feedback) {
    std::ostringstream ss;

    ss << "Received Inverse Kinematic solution: ";

    serializeIKSolution(ss, feedback->ik_solution);

    ROS_INFO_STREAM(ss.str());
}

void serializeIKSolution(std::ostringstream & ss, const moveit_msgs::RobotState & robot_state) {
    int n_joints = robot_state.joint_state.position.size();

    ss << "[";

    for(int i=0; i < n_joints; i++) {
        ss << robot_state.joint_state.position[i];

        if(i != n_joints - 1)
            ss << ", ";
    }

    ss << "]";
}