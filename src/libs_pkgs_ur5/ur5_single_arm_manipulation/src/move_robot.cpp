
#include <ros/ros.h>

#include <ur5_single_arm_manipulation/SetPosition.h>
#include <ur5_single_arm_manipulation/SetDefaultPose.h>
#include <ur5_single_arm_manipulation/SetGripperState.h>
#include <ur5_single_arm_manipulation/OpenDoor.h>
#include <std_msgs/String.h>
#include <trajectory_msgs/JointTrajectoryPoint.h>

#include <pluginlib/class_loader.h>
#include <tf/transform_listener.h>

#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit/robot_model_loader/robot_model_loader.h>
#include <moveit/planning_interface/planning_interface.h>
#include <moveit/planning_scene/planning_scene.h>
#include <moveit/kinematic_constraints/utils.h>
#include <boost/scoped_ptr.hpp>

#include <settings_custom_lib/settings_custom_lib.hpp>

#include <tf2/LinearMath/Quaternion.h>

#include <iostream>
#include <math.h> 
#include <stdio.h>
#include <stdlib.h>
#include <string>
#include <fstream>
#include <cmath>

#include "MoveOperationClass.hpp"

SettingsCustomLibClass settingsConfig;

// При работе с этим пакетом записать актуальные значения для переменныъ ниже
std::string PLANNING_GROUP = "arm";
std::string GRIPPER_GROUP = "gripper";
std::string robotDefaultPose = "start";
std::string robotDefaultJoint = "ee_joint";
std::string pose,robotDefaultLink = "";


bool setGripperAngular(MoveOperationClass *move_group_gripper,
                       const robot_state::JointModelGroup *gripper_joint_group,
                       moveit::core::RobotStatePtr kinematic_state,
                       float value) {

    ROS_INFO("Get value of angular for gripper = %f", value);

    const std::vector<std::string>& joint_names = gripper_joint_group->getVariableNames();
    std::vector<double> joint_values;
    kinematic_state->copyJointGroupPositions(gripper_joint_group, joint_values);


    for (std::size_t i = 0; i < joint_names.size(); ++i) {
        if (joint_names[i] == robotDefaultJoint) {
            joint_values[i] = value;
        }
        ROS_INFO("Joint %s: %f", joint_names[i].c_str(), joint_values[i]);
    }

    move_group_gripper->move->setJointValueTarget(joint_values);
    move_group_gripper->move->move();

    return true;
}


bool robotMove(float x, float y, float z, MoveOperationClass *move_group) {
    ROS_INFO("Got x = %f, y = %f, z = %f", x, y, z);

    geometry_msgs::Pose pose;
    bool success = true;

    pose.position.x = x;
    pose.position.y = y;
    pose.position.z = z;

    tf2::Quaternion q;
    q.setRPY(M_PI/2, 0, M_PI);
    pose.orientation.x = q.getX();
    pose.orientation.y = q.getY();
    pose.orientation.z = q.getZ();
    pose.orientation.w = q.getW();

    move_group->move->setApproximateJointValueTarget(pose,robotDefaultLink);
    moveit::planning_interface::MoveGroupInterface::Plan plan;

    robot_state::RobotState current_state(*(move_group->move)->getCurrentState());
    move_group->move->setStartState(current_state);
    success = (move_group->move->plan(plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);

    ROS_INFO("Visualizing move 1 (pose goal) %s", success ? "" : "FAILED");

    if (success) {
        ROS_INFO("Start move");

        robot_state::RobotState current_state(*(move_group->move)->getCurrentState());
        move_group->move->setStartState(current_state);
        move_group->move->move();

    } else {
        ROS_ERROR("Trajectory calculation error. A series of commands will not be sent to the robot.");
    }

    return true;
}


bool setPosition(ur5_single_arm_manipulation::SetPosition::Request &req, 
                ur5_single_arm_manipulation::SetPosition::Response &res,
                MoveOperationClass *move_group,
                MoveOperationClass *move_group_gripper,
                const robot_state::JointModelGroup *arm_joint_group,
                const robot_state::JointModelGroup *gripper_joint_group,
                moveit::core::RobotStatePtr kinematic_state) {

    bool success = true;

    success = robotMove(req.x, req.y, req.z, move_group);
    res.result = success ? "SUCCESS" : "ERROR";

    ros::spinOnce();

    return true;
}


bool _setDefaultPose(std::string name) {
    moveit::planning_interface::MoveGroupInterface arm(PLANNING_GROUP);

    arm.setGoalJointTolerance(0.001);
    arm.setMaxAccelerationScalingFactor(0.2);
    arm.setMaxVelocityScalingFactor(0.2);
    arm.setNamedTarget(name);
    arm.move();
    sleep(1);

    return true;
}


bool setDefaultPose(ur5_single_arm_manipulation::SetDefaultPose::Request &req, ur5_single_arm_manipulation::SetDefaultPose::Response &res) {

    _setDefaultPose(req.name);

    res.result = "Set default pose END";
    return true;
}


bool setGripperState(ur5_single_arm_manipulation::SetGripperState::Request &req, ur5_single_arm_manipulation::SetGripperState::Response &res,
                        MoveOperationClass *move_group_gripper,
                       const robot_state::JointModelGroup *gripper_joint_group,
                       moveit::core::RobotStatePtr kinematic_state) {

    return setGripperAngular(move_group_gripper, gripper_joint_group, kinematic_state, req.angular);
}


bool openDoor(ur5_single_arm_manipulation::OpenDoor::Request &req, 
             ur5_single_arm_manipulation::OpenDoor::Response &res,
             MoveOperationClass *move_group,
             MoveOperationClass *move_group_gripper,
             const robot_state::JointModelGroup *arm_joint_group,
             const robot_state::JointModelGroup *gripper_joint_group,
             moveit::core::RobotStatePtr kinematic_state) {


    robotMove(settingsConfig.handlePosition_x, settingsConfig.handlePosition_y, settingsConfig.handlePosition_z, move_group);
    setGripperAngular(move_group_gripper, gripper_joint_group, kinematic_state, settingsConfig.gripperPickHandle);

    // Информация о joint`ах
    ROS_INFO("Current joints");

    const std::vector<std::string>& joint_names = arm_joint_group->getVariableNames();
    std::vector<double> joint_values = move_group->move->getCurrentJointValues();

    for (std::size_t i = 0; i < joint_names.size(); i++) {
        ROS_INFO("Joint %s: %f", joint_names[i].c_str(), joint_values[i]);
    }

    // Поворот для открытия двери
    // Траектория неправильная
    // Есть вариант перестроить положение робота, чтобы иметь возможность поворачивать еще один сустав в сторону, обратную направлению вращения
    // Планировщик считает криво (если задавать координаты в цикле)
    double step = 0.1;
    int minAngular = 1.5;
    while (joint_values[0] > minAngular) {

        for (std::size_t i = 0; i < joint_names.size(); i++) {
            if (joint_names[i] == "shoulder_pan_joint") {
                joint_values[i] -= step;
            }

            ROS_INFO("Joint %s: %f", joint_names[i].c_str(), joint_values[i]);
        }

        move_group->move->setJointValueTarget(joint_values);
        move_group->move->move();
        joint_values = move_group->move->getCurrentJointValues();
    }

    // Вернуться в исходное положение
    _setDefaultPose(robotDefaultPose);
    setGripperAngular(move_group_gripper, gripper_joint_group, kinematic_state, 0);

    res.result = "SUCCESS. Robot tried to close the door.";

    return true;
}


int main(int argc, char *argv[]) {
    ROS_INFO("start:");
    ros::init(argc, argv, "move");

    ros::NodeHandle node_handle("~");
    ros::AsyncSpinner spinner(10);
    spinner.start();

    std::string planner_plugin_name;
    if (!node_handle.getParam("planning_plugin", planner_plugin_name)) {
        ROS_FATAL_STREAM("Could not find planner plugin name");
    }

    MoveOperationClass *move_group = new MoveOperationClass(PLANNING_GROUP);
    MoveOperationClass *move_group_gripper = new MoveOperationClass(GRIPPER_GROUP);

    robot_model_loader::RobotModelLoader robot_model_loader("robot_description");

    robot_state::RobotState start_state(*(move_group->move)->getCurrentState());
    const robot_state::JointModelGroup* arm_joint_group = move_group->move->getCurrentState()->getJointModelGroup(PLANNING_GROUP);
    const robot_state::JointModelGroup* gripper_joint_group = move_group->move->getCurrentState()->getJointModelGroup(GRIPPER_GROUP);

    const moveit::core::RobotModelPtr& kinematic_model = robot_model_loader.getModel();
    moveit::core::RobotStatePtr kinematic_state(new moveit::core::RobotState(kinematic_model));
    kinematic_state->setToDefaultValues();

    boost::scoped_ptr<pluginlib::ClassLoader<planning_interface::PlannerManager>> planner_plugin_loader;
    planning_interface::PlannerManagerPtr planner_instance;

    try {
        planner_plugin_loader.reset(new pluginlib::ClassLoader<planning_interface::PlannerManager>("moveit_core", "planning_interface::PlannerManager"));
    } catch (pluginlib::PluginlibException& ex) {
        ROS_FATAL_STREAM("Exception while creating planning plugin loader " << ex.what());
    }

    move_group->move->setPlanningTime(60*5);
    move_group->move->setGoalTolerance(0.001);

    settingsConfig.update();

    ros::NodeHandle n;

    // Получает позицию из position
    ros::ServiceServer setPositionService = n.advertiseService<ur5_single_arm_manipulation::SetPosition::Request, ur5_single_arm_manipulation::SetPosition::Response>
                                ("set_position", boost::bind(setPosition, _1, _2, move_group, move_group_gripper, arm_joint_group, gripper_joint_group, kinematic_state));

    // Установить позу из поз по умолчанию
    ros::ServiceServer setDefaultPoseService = n.advertiseService<ur5_single_arm_manipulation::SetDefaultPose::Request, ur5_single_arm_manipulation::SetDefaultPose::Response>
                                ("set_default_pose", boost::bind(setDefaultPose, _1, _2));

    // Установить угол размыкания схвата
    ros::ServiceServer setGripperStateService = n.advertiseService<ur5_single_arm_manipulation::SetGripperState::Request, ur5_single_arm_manipulation::SetGripperState::Response>
                                ("set_gripper_state", boost::bind(setGripperState, _1, _2, move_group_gripper, gripper_joint_group, kinematic_state));

    // Открыть дверь одной командой
    ros::ServiceServer openDoorService = n.advertiseService<ur5_single_arm_manipulation::OpenDoor::Request, ur5_single_arm_manipulation::OpenDoor::Response>
                                ("robot_open_door", boost::bind(openDoor, _1, _2, move_group, move_group_gripper, arm_joint_group, gripper_joint_group, kinematic_state));

    ros::Duration(1).sleep();
    ros::waitForShutdown();
    return 0;
}