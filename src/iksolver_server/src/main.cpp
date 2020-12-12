#include <iksolver_server/iksolver_server_action.h>

int main(int argc, char **argv)
{
    ros::init(argc, argv, "ikaction_server");
    
    iksolver_server::InverseKinematicsAction ik_action;

    ROS_INFO("Started inverse kinematics action server");

    ros::spin();

    ros::shutdown();
    return 0;
}