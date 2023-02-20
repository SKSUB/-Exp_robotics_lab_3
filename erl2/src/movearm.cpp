/*************************************************************************************************************************//**
 * \file   movearm.cpp
 *
 * description:
 *    This node implements the MovearmActionInterface: the corresponding behaviour for the movearm action defined with the moveit setup assistant.
*****************************************************************************************************************************/

#include <ros/ros.h>
// MoveIt
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/robot_model_loader/robot_model_loader.h>
#include <moveit/robot_model/robot_model.h>
#include <moveit/robot_state/robot_state.h>
#include <geometry_msgs/Pose.h>
#include <moveit_msgs/GetStateValidity.h>
#include <moveit_msgs/GetPlanningScene.h>
#include<unistd.h>


/** 
* int main(argc, argv)
* 
*This is the main function declare the MovearmActionInterface. Load the model. ANd try to move the robot in five different poses in a continuous loop defined in the move it setup assistant.
**/
int main(int argc, char **argv)
{
    
    ros::init(argc, argv, "random_position_server");
    ros::NodeHandle n;
    robot_model_loader::RobotModelLoader robot_model_loader("robot_description");

    const moveit::core::RobotModelPtr& kinematic_model = robot_model_loader.getModel();
    ROS_INFO("Model frame: %s", kinematic_model->getModelFrame().c_str());

    moveit::core::RobotStatePtr kinematic_state(new moveit::core::RobotState(kinematic_model));
    kinematic_state->setToDefaultValues();
    const moveit::core::JointModelGroup* joint_model_group = kinematic_model->getJointModelGroup("arm");
    moveit::planning_interface::MoveGroupInterface group("arm");
    const std::vector<std::string>& joint_names = joint_model_group->getVariableNames();

    ros::AsyncSpinner spinner(1);
    spinner.start();

    while(1)
    {
        group.setNamedTarget("search_aruco_01");
        group.move();
        sleep(0.1);
        group.setNamedTarget("search_aruco_02");
        group.move();
        sleep(0.1);
        group.setNamedTarget("search_aruco_03");
        group.move();
        sleep(0.1);
        group.setNamedTarget("search_aruco_04");
        group.move();
        sleep(0.1);
        group.setNamedTarget("search_aruco_03");
        group.move();
        sleep(0.1);

        //Return to initial position
        group.setNamedTarget("initial_pose");
        group.move();
        sleep(0.1);
    }
}
