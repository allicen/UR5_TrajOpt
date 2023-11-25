// ROS headers
#include <tesseract_common/macros.h>
TESSERACT_COMMON_IGNORE_WARNINGS_PUSH
#include <jsoncpp/json/json.h>
#include <console_bridge/console.h>
TESSERACT_COMMON_IGNORE_WARNINGS_POP

#include <ros/ros.h>
#include <actionlib/client/simple_action_client.h>
#include <control_msgs/FollowJointTrajectoryAction.h>
#include <ros/topic.h>
#include <termios.h>

#include <KinematicsUR5.hpp>
#include <TestIK.hpp>
#include <UR5Trajopt.hpp>
#include <UR5TrajoptResponce.hpp>
#include <robot_context/ur_rtde_interface.hpp>

#include <ur5_husky_main/Box.h>
#include <ur5_husky_main/Mesh.h>
#include <ur5_husky_main/ObstacleGroup.h>
#include <ur5_husky_main/ObstacleDeleteGroup.h>
#include <ur5_husky_main/BoxEntity.h>
#include <ur5_husky_main/MeshEntity.h>
#include <ur5_husky_main/SetJointState.h>
#include <ur5_husky_main/GetJointState.h>
#include <ur5_husky_main/RobotPlanTrajectory.h>
#include <ur5_husky_main/RobotExecuteTrajectory.h>
#include <ur5_husky_main/RobotRestart.h>
#include <ur5_husky_main/Freedrive.h>
#include <ur5_husky_main/IKSolver.h>
#include <ur5_husky_main/GetInfo.h>
#include <ur5_husky_main/GripperService.h>
#include <ur5_husky_main/InfoUI.h>
#include <ur5_husky_main/PoseList.h>
#include <ur5_husky_main/CalculateTrajectory.h>
#include <ur5_husky_main/GripperMoveRobot.h>
#include <ur5_husky_main/GetGripperState.h>
#include <ur5_husky_main/Settings.h>
#include <tesseract_monitoring/environment_monitor.h>
#include <tesseract_rosutils/plotting.h>
#include <trajectory_msgs/JointTrajectory.h>
#include <sensor_msgs/JointState.h>
#include <std_msgs/String.h>
#include <std_srvs/Empty.h>
#include <tesseract_environment/utils.h>
#include <tesseract_common/timer.h>
#include <tesseract_common/resource_locator.h>
#include <tesseract_command_language/composite_instruction.h>
#include <tesseract_command_language/state_waypoint.h>
#include <tesseract_command_language/cartesian_waypoint.h>
#include <tesseract_command_language/joint_waypoint.h>
#include <tesseract_command_language/move_instruction.h>
#include <tesseract_command_language/utils.h>
#include <tesseract_task_composer/planning/planning_task_composer_problem.h>
#include <tesseract_task_composer/taskflow/taskflow_task_composer_executor.h>
#include <tesseract_visualization/markers/toolpath_marker.h>

#include <tesseract_msgs/EnvironmentState.h>
#include <tesseract_msgs/ModifyEnvironment.h>
#include <tesseract_msgs/EnvironmentCommand.h>
#include <tesseract_msgs/GetEnvironmentChanges.h>
#include <ros/service.h>

#include <tesseract_geometry/impl/mesh_material.h>
#include <tesseract_geometry/geometries.h>
#include <tesseract_geometry/mesh_parser.h>

#include <tesseract_collision/bullet/convex_hull_utils.h>

#include <tesseract_motion_planners/core/utils.h>
#include <tesseract_motion_planners/trajopt/trajopt_motion_planner.h>

#include <tesseract_motion_planners/trajopt/profile/trajopt_default_composite_profile.h>
#include <tesseract_motion_planners/trajopt/profile/trajopt_default_plan_profile.h>
#include <tesseract_motion_planners/trajopt/profile/trajopt_default_solver_profile.h>

#include <tesseract_visualization/trajectory_player.h>

#include <ur_rtde/dashboard_client.h>
#include <ur_rtde/rtde_control_interface.h>
#include <ur_rtde/rtde_io_interface.h>
#include <ur_rtde/rtde_receive_interface.h>

#include <ur_rtde/robotiq_gripper.h>

#include <actionlib/client/simple_action_client.h>
#include <robotiq_2f_gripper_msgs/CommandRobotiqGripperAction.h>
#include <robotiq_2f_gripper_control/robotiq_gripper_client.h>

#include <iostream>
#include <thread>
#include <chrono>
#include <vector>
#include <string>
#include <cstdlib>

#include <settings_custom_lib/settings_custom_lib.hpp>

#include <urdf_parser/urdf_parser.h>
#include <srdfdom/model.h>

#include "ColorInfo.hpp"

using namespace ur_rtde;
using namespace std;

using namespace ur5_husky_main;
using namespace tesseract_rosutils;

using namespace trajopt;
using namespace tesseract_environment;
using namespace tesseract_kinematics;
using namespace tesseract_scene_graph;
using namespace tesseract_collision;
using namespace tesseract_visualization;
using namespace tesseract_planning;
using tesseract_common::ManipulatorInfo;

typedef robotiq_2f_gripper_control::RobotiqActionClient RobotiqActionClient;

SettingsCustomLibClass settingsConfig;

/** @brief Default ROS parameter for robot description */
const std::string ROBOT_DESCRIPTION_PARAM = "robot_description";

/** @brief Default ROS parameter for robot description */
const std::string ROBOT_SEMANTIC_PARAM = "robot_description_semantic";

/** @brief RViz Example Namespace */
const std::string EXAMPLE_MONITOR_NAMESPACE = "tesseract_ros_examples";

Eigen::VectorXd joint_start_pos(6);
Eigen::VectorXd joint_end_pos(6);
std::vector<Eigen::VectorXd> joint_middle_pos_list;
bool robotPlanTrajectory = false;
bool robotExecuteTrajectory = false;
bool setRobotNotConnectErrorMes = false;
bool robotRestart = true;
bool freeDriveOn = false;
bool robotBusy = false;

bool async_move = false;
double joints_delta_target = 0.0001; // максимальная погрешность при расчете фактического положения джоинтов к желаемому
std::vector<double> joints_state_current;
bool robotMoveNow = false;

std::vector<ur5_husky_main::Gripper> gripperPoseList;
ur5_husky_main::Gripper gripperPosePrev;

/// Иначе не работает асинхронное движение
URRTDEInterface* rtde;

float gripperAngle = 0.85;

std::string robot_ip = "127.0.0.1";

double delay_loop_rate = 0.0;
double ur_speed = 0.1;
double ur_acceleration = 10.0;
double ur_blend = 0.0;



ColorInfo getDefaultColor(std::string colorName) {
  if (colorName == "brown") {
    ColorInfo colorBrown{colorName, 0.83, 0.37, 0.2, 1.0};
    return colorBrown;
  } 
  ColorInfo colorDefault{colorName, 1.0, 1.0, 1.0, 1.0};
  return colorDefault;
}


tesseract_environment::Command::Ptr addBox(std::string link_name, std::string joint_name,
                                           float length, float width, float height,
                                           float pos_x, float pos_y, float pos_z,
                                           ColorInfo color) {

  auto colorBox = std::make_shared<tesseract_scene_graph::Material>(color.getName().c_str());
  colorBox->color = Eigen::Vector4d(color.getR(), color.getG(), color.getB(), color.getA());

  // Add sphere to environment
  Link link_sphere(link_name.c_str());

  Visual::Ptr visual = std::make_shared<Visual>();
  visual->origin = Eigen::Isometry3d::Identity();
  visual->origin.translation() = Eigen::Vector3d(pos_x, pos_y, pos_z);
  visual->geometry = std::make_shared<tesseract_geometry::Box>(width, length, height);
  visual->material = colorBox;

  link_sphere.visual.push_back(visual);

  Collision::Ptr collision = std::make_shared<Collision>();
  collision->origin = visual->origin;
  collision->geometry = visual->geometry;
  link_sphere.collision.push_back(collision);

  Joint joint_sphere(joint_name.c_str());
  joint_sphere.parent_link_name = "world";
  joint_sphere.child_link_name = link_sphere.getName();
  joint_sphere.type = JointType::FIXED;

  return std::make_shared<tesseract_environment::AddLinkCommand>(link_sphere, joint_sphere);
}



tesseract_environment::Command::Ptr addMesh(std::string link_name,
                                            std::string joint_name,
                                            std::string mesh_name,
                                            Eigen::Vector3d scale,
                                            Eigen::Vector3d translation,
                                            Eigen::Vector3d rotation) {

  std::string mesh_path = "package://ur5_husky_main/meshes/objects/" + mesh_name;

  tesseract_common::ResourceLocator::Ptr locator = std::make_shared<tesseract_common::GeneralResourceLocator>();
  std::vector<tesseract_geometry::Mesh::Ptr> meshes =
    tesseract_geometry::createMeshFromResource<tesseract_geometry::Mesh>(
        locator->locateResource(mesh_path), scale, true, false, true);

  Link link_sphere(link_name.c_str());

  for (auto& mesh : meshes) {
    Visual::Ptr visual = std::make_shared<Visual>();
    visual->origin = Eigen::Isometry3d::Identity();
    visual->geometry = mesh;
    link_sphere.visual.push_back(visual);

    Collision::Ptr collision = std::make_shared<Collision>();
    collision->origin = visual->origin;
    collision->geometry = makeConvexMesh(*mesh);
    link_sphere.collision.push_back(collision);
  }

  Joint joint_sphere(joint_name.c_str());
  joint_sphere.parent_link_name = "world";
  joint_sphere.child_link_name = link_sphere.getName();
  joint_sphere.type = JointType::FIXED;

  return std::make_shared<tesseract_environment::AddLinkCommand>(link_sphere, joint_sphere);
}


tesseract_environment::Command::Ptr renderMove(std::string link_name, std::string joint_name,
                                           float pos_x, float pos_y, float pos_z,
                                           float rotateX = 0, float rotateY = 0, float rotateZ = 0) {

  auto joint_limits =  std::make_shared<tesseract_scene_graph::JointLimits>();
  joint_limits->lower = 1.0;
  joint_limits->upper = 2.0;

  Eigen::AngleAxisd rotX(rotateX, Eigen::Vector3d::UnitX());
  Eigen::AngleAxisd rotY(rotateY, Eigen::Vector3d::UnitY());
  Eigen::AngleAxisd rotZ(rotateZ, Eigen::Vector3d::UnitZ());

  Joint joint_sphere(joint_name.c_str());
  joint_sphere.limits = joint_limits;
  joint_sphere.parent_link_name = "world";
  joint_sphere.child_link_name = link_name;
  joint_sphere.type = JointType::FIXED;

  joint_sphere.parent_to_joint_origin_transform.translation() = Eigen::Vector3d(pos_x, pos_y, pos_z);
  joint_sphere.parent_to_joint_origin_transform.rotate(rotX);
  joint_sphere.parent_to_joint_origin_transform.rotate(rotY);
  joint_sphere.parent_to_joint_origin_transform.rotate(rotZ);

  return std::make_shared<tesseract_environment::MoveLinkCommand>(joint_sphere);
}


bool moveBox(ur5_husky_main::Box::Request &req,
             ur5_husky_main::Box::Response &res,
             const std::shared_ptr<tesseract_environment::Environment> &env) {

  std::string joint_name = std::string(req.name) + "_joints";

  Command::Ptr box = renderMove(req.name, joint_name.c_str(), req.offsetX, req.offsetY, req.offsetZ, req.rotateX, req.rotateY, req.rotateZ);
  if (!env->applyCommand(box)) {
    res.result = "ERROR - create box";
    return false;
  }

  res.result = "Move Box end...";

  return true;
}


bool moveMesh(ur5_husky_main::Mesh::Request &req,
             ur5_husky_main::Mesh::Response &res,
             const std::shared_ptr<tesseract_environment::Environment> &env) {

  std::string joint_name = std::string(req.name) + "_joints";

  Command::Ptr mesh = renderMove(req.name, joint_name.c_str(), req.offsetX, req.offsetY, req.offsetZ, req.rotateX, req.rotateY, req.rotateZ);
  if (!env->applyCommand(mesh)) {
    res.result = "ERROR - move mesh";
    return false;
  }

  res.result = "Move Mesh end...";

  return true;
}

tesseract_environment::Command::ConstPtr renderRemoveLink(std::string link_name) {
  return std::make_shared<tesseract_environment::RemoveLinkCommand>(link_name);
}

tesseract_environment::Command::ConstPtr renderRemoveJoint(std::string joints_name) {
  return std::make_shared<tesseract_environment::RemoveJointCommand>(joints_name);
}

tesseract_environment::Command::ConstPtr renderHideLink(std::string link_name) {
  return std::make_shared<tesseract_environment::ChangeLinkVisibilityCommand>(link_name, false);
}

bool removeBox(ur5_husky_main::Box::Request &req,
             ur5_husky_main::Box::Response &res,
             const std::shared_ptr<tesseract_environment::Environment> &env) {
  Command::ConstPtr boxLink = renderRemoveLink(req.name);
  if (!env->applyCommand(boxLink)) {
    res.result = "ERROR - remove link box";
    return false;
  }

  res.result = "Remove Box end...";

  return true;
}


bool removeMesh(ur5_husky_main::Mesh::Request &req,
             ur5_husky_main::Mesh::Response &res,
             const std::shared_ptr<tesseract_environment::Environment> &env) {

  Command::ConstPtr meshLink = renderRemoveLink(req.name);
  if (!env->applyCommand(meshLink)) {
    res.result = "ERROR - remove link mesh";
    return false;
  }

  res.result = "Remove Mesh end...";

  return true;
}


bool createBox(ur5_husky_main::Box::Request &req,
              ur5_husky_main::Box::Response &res,
              const std::shared_ptr<tesseract_environment::Environment> &env) {

  std::string joint_name = std::string(req.name) + "_joints";

  ColorInfo color{req.color.name, req.color.r, req.color.g, req.color.b, req.color.a};

  Command::Ptr box = addBox(req.name, joint_name.c_str(), req.length, req.width, req.height, req.x, req.y, req.z, color);
  if (!env->applyCommand(box)) {
    res.result = "ERROR - create box";
    return false;
  }

  // Сдвинуть box
  Command::Ptr move = renderMove(req.name, joint_name.c_str(), req.offsetX, req.offsetY, req.offsetZ, req.rotateX, req.rotateY, req.rotateZ);
  if (!env->applyCommand(move)) {
    res.result = "ERROR - move new box";
    return false;
  }

  res.result = "Create box success";
  return true;
}


bool createMesh(ur5_husky_main::Mesh::Request &req,
              ur5_husky_main::Mesh::Response &res,
              const std::shared_ptr<tesseract_environment::Environment> &env) {

  std::string joint_name = std::string(req.name) + "_joints";

  Command::Ptr mesh = addMesh(req.name, joint_name.c_str(), req.fileName,
                      Eigen::Vector3d(req.scale, req.scale, req.scale), 
                      Eigen::Vector3d(req.x, req.y, req.z), 
                      Eigen::Vector3d(req.rotateX, req.rotateY, req.rotateZ));

  // Создать меш
  if (!env->applyCommand(mesh)) {
    res.result = "ERROR - create mesh";
    return false;
  }

  // Сдвинуть меш
  Command::Ptr move = renderMove(req.name, joint_name.c_str(), req.offsetX, req.offsetY, req.offsetZ, req.rotateX, req.rotateY, req.rotateZ);
  if (!env->applyCommand(move)) {
    res.result = "ERROR - move new mesh";
    return false;
  }

  res.result = "Create mesh success";
  return true;
}



bool deleteObstacleGroup(ur5_husky_main::ObstacleDeleteGroup::Request &req,
                        ur5_husky_main::ObstacleDeleteGroup::Response &res,
                        const std::shared_ptr<tesseract_environment::Environment> &env) {

  std::vector<std::string> mesh_names_success;
  std::vector<std::string> mesh_names_failed;
  std::vector<std::string> box_names_success;
  std::vector<std::string> box_names_failed;

  for (int i = 0; i < req.meshes_delete.size(); i++) {

    Command::ConstPtr meshLink = renderRemoveLink(req.meshes_delete[i]);
    if (env->applyCommand(meshLink)) {
      mesh_names_success.push_back(req.meshes_delete[i]);
    } else {
      mesh_names_failed.push_back(req.meshes_delete[i]);
    }
  }

  for (int i = 0; i < req.boxes_delete.size(); i++) {
    Command::ConstPtr boxLink = renderRemoveLink(req.boxes_delete[i]);
    if (env->applyCommand(boxLink)) {
      box_names_success.push_back(req.boxes_delete[i]);
    } else {
      box_names_failed.push_back(req.boxes_delete[i]);
    }
  }

  res.message = "Удалены препятствия группы.";
  res.mesh_names_success = mesh_names_success;
  res.mesh_names_failed = mesh_names_failed;
  res.box_names_success = box_names_success;
  res.box_names_failed = box_names_failed;

  return true;
}



bool createObstacleGroup(ur5_husky_main::ObstacleGroup::Request &req,
                        ur5_husky_main::ObstacleGroup::Response &res,
                        const std::shared_ptr<tesseract_environment::Environment> &env) {

  std::set<int> mesh_ids_success;
  std::set<int> mesh_ids_failed;
  std::set<int> box_ids_success;
  std::set<int> box_ids_failed;

  // Создать боксы
  for (int i = 0; i < req.boxList.size(); i++) {

    BoxEntity box = req.boxList[i];
    std::string joint_name = std::string(box.name) + "_joints";
    ColorInfo color{box.color.name, box.color.r, box.color.g, box.color.b, box.color.a};

    Command::Ptr boxNew = addBox(box.name, joint_name.c_str(), box.length, box.width, box.height, box.x, box.y, box.z, color);
    if (env->applyCommand(boxNew)) {
      box_ids_success.insert(box_ids_success.end(), box.id);
      std::cout << i << " === " << box.id << std::endl;
    } else {
      box_ids_failed.insert(box_ids_failed.end(), box.id);
    }

    // Сдвинуть box
    Command::Ptr moveBox = renderMove(box.name, joint_name.c_str(), box.offsetX, box.offsetY, box.offsetZ, box.rotateX, box.rotateY, box.rotateZ);
    if (env->applyCommand(moveBox)) {
      box_ids_success.insert(box_ids_success.end(), box.id);
    } else {
      box_ids_failed.insert(box_ids_failed.end(), box.id);
    }
  }

  // Создать меши
  for (int i = 0; i < req.meshList.size(); i++) {

    MeshEntity mesh = req.meshList[i];
    std::string joint_name = std::string(mesh.name) + "_joints";

    Command::Ptr meshNew = addMesh(mesh.name, joint_name.c_str(), mesh.fileName,
                    Eigen::Vector3d(mesh.scale, mesh.scale, mesh.scale), 
                    Eigen::Vector3d(mesh.x, mesh.y, mesh.z), 
                    Eigen::Vector3d(mesh.rotateX, mesh.rotateY, mesh.rotateZ));

    if (env->applyCommand(meshNew)) {
      mesh_ids_success.insert(mesh_ids_success.end(), mesh.id);
    } else {
      mesh_ids_failed.insert(mesh_ids_failed.end(), mesh.id);
    }

    // Сдвинуть меш
    Command::Ptr moveMesh = renderMove(mesh.name, joint_name.c_str(), mesh.offsetX, mesh.offsetY, mesh.offsetZ, mesh.rotateX, mesh.rotateY, mesh.rotateZ);
    if (env->applyCommand(moveMesh)) {
      mesh_ids_success.insert(mesh_ids_success.end(), mesh.id);
    } else {
      mesh_ids_failed.insert(mesh_ids_failed.end(), mesh.id);
    }
  }

  std::vector<int> b_success;
  std::copy(box_ids_success.begin(), box_ids_success.end(), std::back_inserter(b_success));

  std::vector<int> b_filed;
  std::copy(box_ids_failed.begin(), box_ids_failed.end(), std::back_inserter(b_filed));

  std::vector<int> m_success;
  std::copy(mesh_ids_success.begin(), mesh_ids_success.end(), std::back_inserter(m_success));

  std::vector<int> m_filed;
  std::copy(mesh_ids_failed.begin(), mesh_ids_failed.end(), std::back_inserter(m_filed));


  res.box_ids_success = b_success;
  res.box_ids_failed = b_filed;
  res.mesh_ids_success = m_success;
  res.mesh_ids_failed = m_filed;

  res.message = "Группа создана: " + req.name;

  return true;
}


void sendRobotState(const ros::Publisher &messageRobotBusyPub) {
  std_msgs::String msg;
  if (robotBusy) {
      msg.data = "robot_busy";
    } else {
      msg.data = "robot_ready";
    }

    messageRobotBusyPub.publish(msg);
}


void robotBusyPublish() {

  ros::NodeHandlePtr node = boost::make_shared<ros::NodeHandle>();
  ros::Publisher messageRobotBusyPub = node->advertise<std_msgs::String>("robot_busy", 10);

  ros::Rate loop_rate(15);

  while(ros::ok()) {
    sendRobotState(messageRobotBusyPub);
    ros::spinOnce();
    loop_rate.sleep();
  }
}


void robotAsyncMoveCalc(std::shared_ptr<ur_rtde::RTDEReceiveInterface> &rtde_receive) {

  while(ros::ok()) {

    if (!robotMoveNow) {
      break;
    }

    joints_state_current = rtde_receive->getTargetQ();

    ros::spinOnce();
    ros::Rate loop_rate(15);
    loop_rate.sleep();
  }
}


double joints_pos_delta(std::vector<double> &path_pose) {
  double joints_current_sum = 0;
  double joints_target_sum = 0;

  if (joints_state_current.size() == 0) {
    ROS_ERROR("Var `joints_state_current` can`t == 0!");
    return 0.0;
  }

  for (int i = 0; i < 6; i++) {
    joints_current_sum += abs(path_pose[i]);
    joints_target_sum += abs(joints_state_current[i]);
  }

  return abs(joints_current_sum - joints_target_sum);
}


void robotMove(std::vector<double> &path_pose, const ros::Publisher &messageRobotBusyPub, ros::Rate &loop_rate) {

  if (!rtde->robotConnect()) {
    setRobotNotConnectErrorMes = true;
    return;
  }

  auto rtde_control = rtde->getRtdeControl();
  auto rtde_receive = rtde->getRtdeReceive();

  path_pose.push_back(ur_speed);
  path_pose.push_back(ur_acceleration);
  path_pose.push_back(ur_blend);

  std::vector<std::vector<double>> jointsPath;
  jointsPath.push_back(path_pose);
  robotBusy = true;
  sendRobotState(messageRobotBusyPub);
  rtde_control->moveJ(jointsPath, async_move);

  if (async_move) {
    robotMoveNow = true;
    std::thread robotAsyncMoveThread(robotAsyncMoveCalc, std::ref(rtde_receive));
    robotAsyncMoveThread.detach();
    while (joints_pos_delta(path_pose) > joints_delta_target) {
      loop_rate.sleep();
      continue;
    }

    robotMoveNow = false;
    robotBusy = false;
    sendRobotState(messageRobotBusyPub);
  }

  robotBusy = false;
  sendRobotState(messageRobotBusyPub);
}


bool updateStartJointValue(ur5_husky_main::SetJointState::Request &req,
                      ur5_husky_main::SetJointState::Response &res,
                      const std::shared_ptr<tesseract_environment::Environment> &env,
                      const ros::Publisher &joint_pub_state,
                      const std::vector<std::string> &joint_names,
                      const bool connect_robot,
                      ros::Rate &loop_rate,
                      ros::Publisher &gripperPub,
                      const ros::Publisher &messageRobotBusyPub) {

    std::vector<double> position_vector;
    std::vector<double> velocity;
    std::vector<double> effort;

    int index = 0;
    for (int j = 0; j < joint_names.size(); j++) {
        for (int i = 0; i < req.name.size(); i++) {
            // нужны только избранные joints
            if (req.name[i] == joint_names[j]) {
                joint_start_pos(index) = req.position[i];
                velocity.push_back(req.velocity[i]);
                effort.push_back(req.effort[i]);

                index++;
            }
        }
    }

    position_vector.resize(joint_start_pos.size());
    Eigen::VectorXd::Map(&position_vector[0], joint_start_pos.size()) = joint_start_pos;

    if (req.gripperUpdate) {
        std::cout << "gripper start: " << req.gripperPose[0].name << " " << req.gripperPose[0].angle << std::endl;
        gripperPosePrev.name = req.gripperPose[0].name;
        gripperPosePrev.angle = req.gripperPose[0].angle;
    }

    if (connect_robot) {
        robotMove(position_vector, messageRobotBusyPub, loop_rate);
        if (req.gripperUpdate) {
            gripperPub.publish(req.gripperPose[0]);
        }
    }

    env->setState(joint_names, joint_start_pos);

    ros::spinOnce();
    loop_rate.sleep();

    res.result = "End publish";
    return true;
}


bool updateFinishJointValue(ur5_husky_main::SetJointState::Request &req,
                      ur5_husky_main::SetJointState::Response &res,
                      const std::shared_ptr<tesseract_environment::Environment> &env,
                      const ros::Publisher &joint_pub_state,
                      const std::vector<std::string> &joint_names,
                      const bool connect_robot) {

    gripperPoseList.insert(gripperPoseList.begin(), std::begin(req.gripperPose), std::end(req.gripperPose));
    std::cout << "gripperPoseList: " << gripperPoseList.size() << std::endl;

    int index = 0;
    for (int j = 0; j < joint_names.size(); j++) {
        for (int i = 0; i < req.name.size(); i++) {
            // нужны только избранные joints
            if (req.name[i] == joint_names[j]) {
                joint_end_pos(index) = req.position[i];
                index++;
            }
        }
    }

    for (int i = 0; i < req.middlePose.size(); i++) {
      index = 0;
      Eigen::VectorXd joint_tmp(joint_names.size());

      for (int k = 0; k < joint_names.size(); k++) {
        for (int j = 0; j < req.middlePose[i].name.size(); j++) {
          if (req.middlePose[i].name[j] == joint_names[k]) {
            joint_tmp(index) = req.middlePose[i].position[j];
            index++;
          }
        }
      }
      joint_middle_pos_list.push_back(joint_tmp);
    }

    env->setState(joint_names, joint_end_pos);

    res.result = "Update Finish Position!";
    return true;
}


void printPoseInRviz(const ros::Publisher &joint_pub_state,
                     const std::vector<std::string> &joint_names,
                     const Eigen::VectorXd &pose) {

  std::vector<double> joint_positions;
  joint_positions.resize(pose.size());
  Eigen::VectorXd::Map(&joint_positions[0], pose.size()) = pose;

  sensor_msgs::JointState joint_state_msg;
  joint_state_msg.name = joint_names;
  joint_state_msg.position = joint_positions;
  joint_pub_state.publish(joint_state_msg);
}

bool getJointValue(ur5_husky_main::GetJointState::Request &req,
                   ur5_husky_main::GetJointState::Response &res,
                   const std::vector<std::string> &joint_names,
                   const ros::Publisher &joint_pub_state,
                   const std::shared_ptr<tesseract_environment::Environment> &env) {

  std::vector<double> joint_positions;

  if (req.from_robot) {
    if (rtde->robotConnect()) {
      auto rtde_receive = rtde->getRtdeReceive();
      ROS_INFO("Connect success!");
      std::vector<double> joint_positions = rtde_receive->getActualQ();

      for (auto i = 0; i < joint_positions.size(); i++) {
        joint_start_pos(i) = joint_positions[i];
      }
    }
    
    env->setState(joint_names, joint_start_pos);
  }

  joint_positions.resize(joint_start_pos.size());
  Eigen::VectorXd::Map(&joint_positions[0], joint_start_pos.size()) = joint_start_pos;

  printPoseInRviz(joint_pub_state, joint_names, joint_start_pos);

  res.name = joint_names;
  res.position = joint_positions;

  return true;
}


bool calculateRobotTrajectory(ur5_husky_main::CalculateTrajectory::Request &req, 
                              ur5_husky_main::CalculateTrajectory::Response &res,
                              const std::shared_ptr<tesseract_environment::Environment> &env,
                              const ROSPlottingPtr &plotter,
                              const std::vector<std::string> &joint_names,
                              const ros::Publisher &messagePosePub,
                              ros::Rate &loop_rate) {

  std::vector<double> start = req.startPose.position;
  std::vector<double> finish = req.finishPose.position;

  Eigen::VectorXd startPose = Eigen::Map<Eigen::VectorXd, Eigen::Unaligned>(start.data(), start.size());
  Eigen::VectorXd finishPose = Eigen::Map<Eigen::VectorXd, Eigen::Unaligned>(finish.data(), finish.size());

  std::vector<Eigen::VectorXd> middlePos;
  middlePos.empty();

  for (int i = 0; i < req.middlePose.size(); i++) {
      std::vector<double> middle = req.middlePose[i].position;
      middlePos.push_back(Eigen::Map<Eigen::VectorXd, Eigen::Unaligned>(middle.data(), middle.size()));
  }

  UR5Trajopt calculate(env, plotter, joint_names, startPose, finishPose, middlePos);
  UR5TrajoptResponce responce = calculate.run();

  tesseract_common::JointTrajectory trajectoryTrajOpt = responce.getTrajectory();

  std::vector<ur5_husky_main::Pose> trajectory;

  TrajectoryPlayer player;
  player.setTrajectory(trajectoryTrajOpt);

  for (int i = 0; i < trajectoryTrajOpt.states.size(); i++) {

    std::vector<double> position;

    tesseract_common::JointState j_state = player.getByIndex(i);
    position.resize(j_state.position.size());
    Eigen::VectorXd::Map(&position[0], j_state.position.size()) = j_state.position;

    ur5_husky_main::Pose msg;
    msg.name = j_state.joint_names;
    msg.position = position;
    trajectory.push_back(msg);
  }

  res.success = responce.isSuccessful();
  res.message = responce.getMessage();
  res.trajectory = trajectory;

  ur5_husky_main::PoseList poses;
  poses.pose_list = trajectory;

  messagePosePub.publish(poses);
  ros::spinOnce();
  loop_rate.sleep();

  return true;
}


void sendMessageToUI(std::string message, ros::Publisher &messageUIPub, ros::Rate &loop_rate, double time = 0.0) {
  ur5_husky_main::InfoUI msg;
  msg.code = message;
  msg.time = time;
  ROS_INFO("Sent message to UI: %s", msg.code.c_str());
  messageUIPub.publish(msg);
  ros::spinOnce();
  loop_rate.sleep();
}


bool robotPlanTrajectoryMethod(ur5_husky_main::RobotPlanTrajectory::Request &req,
      ur5_husky_main::RobotPlanTrajectory::Response &res,
      const ros::Publisher &messageRobotBusyPub,
      ros::Rate &loop_rate,
      ros::Publisher &gripperPub,
      ros::Publisher &joint_pub_state,
      const std::shared_ptr<tesseract_environment::Environment> &env,
      const std::vector<std::string> &joint_names,
      ros::Publisher &messageUIPub) {
  std::cout << "0. START" << std::endl;

  if (req.trajopt) {
    robotPlanTrajectory = true;
    res.result = "Plan Trajectory";
  } else {

    std::cout << "1. joints list = " << joint_middle_pos_list.size() << std::endl;
    std::cout << "2. gripper list = " << gripperPoseList.size() << std::endl;

    for (int i = 0; i < joint_middle_pos_list.size(); i++) {
      std::vector<double> position_vector_arr;
      position_vector_arr.resize(joint_middle_pos_list[i].size());
      Eigen::VectorXd::Map(&position_vector_arr[0], joint_middle_pos_list[i].size()) = joint_middle_pos_list[i];

      robotMove(position_vector_arr, messageRobotBusyPub, loop_rate);
      gripperPub.publish(gripperPoseList[i+1]); // приходит включая стартовое положение гриппера
      std::cout << "Middle Pos" << joint_middle_pos_list[i] << ", gripper = " << gripperPoseList[i+1] << std::endl;
    }

    std::vector<double> position_vector;
    position_vector.resize(joint_end_pos.size());
    Eigen::VectorXd::Map(&position_vector[0], joint_end_pos.size()) = joint_end_pos;

    robotMove(position_vector, messageRobotBusyPub, loop_rate);
    gripperPub.publish(gripperPoseList[gripperPoseList.size()-1]);

    env->setState(joint_names, joint_end_pos);
    printPoseInRviz(joint_pub_state, joint_names, joint_end_pos);
    res.result = "End run robot...";
    sendMessageToUI("move_finish", messageUIPub, loop_rate);
  }

   res.success = true;
   return true;
}

bool robotExecuteTrajectoryMethod(ur5_husky_main::RobotExecuteTrajectory::Request &req, ur5_husky_main::RobotExecuteTrajectory::Response &res) {
  robotExecuteTrajectory = true;
  res.result = "Execute Trajectory";
  res.success = true;
  return true;
}

bool robotRestartMethod(ur5_husky_main::RobotRestart::Request &req, ur5_husky_main::RobotRestart::Response &res) {
  robotRestart = true;

  // Откатить начальное значения
  robotPlanTrajectory = false;
  robotExecuteTrajectory = false;
  setRobotNotConnectErrorMes = false;
  res.result = "Robot restart Trajectory";
  return true;
}


void freedriveControl(ros::Rate &loop_rate) {
  if (!rtde->robotConnect()) {
    return;
  }
  auto rtde_receive = rtde->getRtdeReceive();
  auto rtde_control = rtde->getRtdeControl();
  auto dash_board = rtde->getDashboard();

  dash_board->connect();

  if (rtde_control->isConnected()) {

      rtde_control->teachMode();

      // Если значения ниже - 7 7 2, то freedrive включен
      std::cout << "Robot mode: " << rtde_receive->getRobotMode() << "\n";
      std::cout << "Robot status: " << rtde_receive->getRobotStatus() << "\n";
      std::cout << "RuntimeState: " << rtde_receive->getRuntimeState() << "\n";
      std::cout << dash_board->polyscopeVersion() << "\n";

      ROS_INFO("Freedrive on");

      while(ros::ok()) {
        ros::spinOnce();
        loop_rate.sleep();
        if (!freeDriveOn) {
          break;
        }
      }

      rtde_control->endTeachMode();
      std::cout << "Robot status: " << rtde_receive->getRobotStatus() << "\n";

      ROS_INFO("Freedrive off");
  }

  std::this_thread::sleep_for(std::chrono::milliseconds(1000));
  std::cout<<"stop script..." <<std::endl;
}


bool setSettings(ur5_husky_main::Settings::Request &req,
                 ur5_husky_main::Settings::Response &res) {

  std::cout << "async = " << async_move << " " << req.key<< std::endl;

  if (req.key == "async_move") {
    async_move = req.value;
    res.result = "SUCCESS";
  }

  std::cout << "async = " << async_move << std::endl;

  return true;
}


bool freedriveEnable(ur5_husky_main::Freedrive::Request &req,
                    ur5_husky_main::Freedrive::Response &res) {

  std::string str = req.on ? "true" : "false";
  std::cout << "Freedrive comand: " <<  str << std::endl;
  freeDriveOn = req.on;

  std::string mess = "Freedrive command: " + str + ". Freedrive state changed! ";
  res.result = mess.c_str();

  return true;
}


bool getInfo(ur5_husky_main::GetInfo::Request &req,
                   ur5_husky_main::GetInfo::Response &res,
                   const std::shared_ptr<tesseract_environment::Environment> &env,
                   const std::vector<std::string> &joint_names,
                   ros::Rate &loop_rate) {

  TestIK test(robot_ip, ur_speed, ur_acceleration, ur_blend, req.debug);
  if (rtde->robotConnect()) {
    auto dash_board = rtde->getDashboard();
    dash_board->connect();
    std::cout << "--- Модель робота ---" << dash_board->getRobotModel() << std::endl;
    // getSerialNumber() function is not supported on the dashboard server for PolyScope versions less than 5.6.0
    // std::cout << "--- Серийный номер робота ---" << dash_board->getSerialNumber() << std::endl;
    std::cout << "--- Программа робота: ---" << dash_board->getLoadedProgram() << std::endl;
  }

  if (req.fk || req.ik) {
    if (req.fk) {
      // Получить данные из библиотеки ur_rtde
      auto rtde_control = rtde->getRtdeControl();

      if (rtde_control->isConnected()) {
        std::vector<double> fk = rtde_control->getForwardKinematics();
        if (req.fk) {
          std::cout << "Прямая кинематика (ur_rtde): ";
          for (int i = 0; i < fk.size(); i++) {
            std::cout << fk[i] << " ";
          }
          std::cout << std::endl;              
        }

        if (req.ik) {
            std::vector<double> ik = rtde_control->getInverseKinematics(fk);
            std::cout << "Обратная кинематика (ur_rtde): ";
            for (int i = 0; i < ik.size(); i++) {
              std::cout << ik[i] << " ";
            }
            std::cout << std::endl;
        }
      }

      test.getForwardKinematics(joint_start_pos);
    }

    if (req.ik) {
      test.ikSolverCheck(loop_rate, joint_start_pos);
    }
  }

  res.result = "Info got successfully";
  return true;
}


bool gripperMove(ur5_husky_main::GripperService::Request &req,
                 ur5_husky_main::GripperService::Response &res,
                 ros::ServiceClient &gripperMoveRobotService, 
                 ur5_husky_main::GripperMoveRobot &srv) {

  srv.request.name = "";
  srv.request.speed = 0.10;
  srv.request.angle = req.angle;

  if (gripperMoveRobotService.call(srv)) {
    ROS_INFO("Move gripper send ang=%f", req.angle);
  } else {
    ROS_ERROR("Can not call service 'gripperMoveRobotService'");
    return false;
  }

  return true;
}


bool getGripperState(ur5_husky_main::GetGripperState::Request &req,
                 ur5_husky_main::GetGripperState::Response &res,
                 ros::ServiceClient &gsService, 
                 ur5_husky_main::GetGripperState &gripperStateSrv) {


  if (gsService.call(gripperStateSrv)) {
    res.angle = gripperStateSrv.response.angle;
  } else {
    ROS_ERROR("Can not call service 'gsService'");
    return false;
  }
  return true;
}


void gripperCallback(const ur5_husky_main::Gripper::ConstPtr &msg, ros::Publisher &gripperPub) {
  gripperPub.publish(msg);
}


void gripperStateCallback(const ur5_husky_main::Gripper::ConstPtr &msg) {
  if (std::string(msg->name) == "robotiq") {
    gripperAngle = msg->angle;
  }
}


void robotPoseCallback(const ur5_husky_main::Pose::ConstPtr &msg,
  const std::shared_ptr<tesseract_environment::Environment> &env,
  const ros::Publisher &joint_pub_state, 
  const std::vector<std::string> &joint_names,
  const ros::Publisher &messageRobotBusyPub,
  ros::Rate &loop_rate) {

  if (!msg->rvizOnly) {
    robotBusy = true;
    sendRobotState(messageRobotBusyPub);
  }

  if (!msg->rvizOnly && rtde->robotConnect()) {
    auto rtde_control = rtde->getRtdeControl();
    std::vector<double> position = msg->position;
    robotMove(position, messageRobotBusyPub, loop_rate);
  }

  for (int i = 0; i < msg->position.size(); i++) {
    joint_start_pos(i) = msg->position[i];
  }

  env->setState(joint_names, joint_start_pos);
  ros::spinOnce();
  loop_rate.sleep();
  printPoseInRviz(joint_pub_state, joint_names, joint_start_pos);

  if (!msg->rvizOnly) {
    robotBusy = false;
    sendRobotState(messageRobotBusyPub);
  }
}


bool stopMove(std_srvs::Empty::Request &req, std_srvs::Empty::Response &res, const ros::Publisher &messageRobotBusyPub) {

  std::cout << "STOP!" << std::endl;

  if (rtde->robotConnect()) {
    std::cout << "STOP1!" << std::endl;
    auto rtde_control = rtde->getRtdeControl();
    auto rtde_receive = rtde->getRtdeReceive();
    robotMoveNow = false;
    robotBusy = false;
    sendRobotState(messageRobotBusyPub);
    rtde_control->stopScript();

    std::cout << "STOP2!" << std::endl;
  }

  return true;
}


int main(int argc, char** argv) {
  ros::init(argc, argv, "ur5_trajopt_node");
  ros::NodeHandle pnh("~");
  ros::NodeHandle nh;

  bool plotting = true;
  bool rviz = true;
  bool debug = false;
  bool connect_robot = false;
  std::string script = "";

  // конфиги для робота
  double velocity = 0.5;
  double acceleration = 0.5;
  double dt = 1.0/500; // 2ms
  double lookahead_time = 0.1;
  double gain = 300;

  std::vector<double> velocity_default{0.001, 0.001, 0.001, 0.001, 0.001, 0.001};
  std::vector<double> effort_default{0.0, 0.0, 0.0, 0.0, 0.0, 0,0};
  std::vector<double> accelerations_default{0.1, 0.2, 0.3, 0.4, 0.5, 0,6};
  std::vector<double> position_vector;

  // Get ROS Parameters
  pnh.param("plotting", plotting, plotting);
  pnh.param("rviz", rviz, rviz);
  pnh.param("debug", debug, debug);
  pnh.param("connect_robot", connect_robot, connect_robot);
  pnh.param("script", script, script);
  pnh.param("robot_ip", robot_ip, robot_ip);
  pnh.param("delay_loop_rate", delay_loop_rate, delay_loop_rate);
  pnh.param("ur_speed", ur_speed, ur_speed);
  pnh.param("ur_acceleration", ur_acceleration, ur_acceleration);
  pnh.param("ur_blend", ur_blend, ur_blend);

  ros::Rate loop_rate(delay_loop_rate);

  // Initial setup
  std::string urdf_xml_string, srdf_xml_string;
  nh.getParam(ROBOT_DESCRIPTION_PARAM, urdf_xml_string);
  nh.getParam(ROBOT_SEMANTIC_PARAM, srdf_xml_string);

  ros::Publisher joint_pub_traj = nh.advertise<trajectory_msgs::JointTrajectory>("/joint_trajectory", 1000);
  ros::Publisher joint_pub_state = pnh.advertise<sensor_msgs::JointState>("/joint_states", 1000);

  settingsConfig.update();

  rtde = URRTDEInterface::getInstance(robot_ip);

  auto env = std::make_shared<tesseract_environment::Environment>();

  auto locator = std::make_shared<tesseract_rosutils::ROSResourceLocator>();
  if (!env->init(urdf_xml_string, srdf_xml_string, locator)) {
    exit(1);
  }

  // Create monitor
  auto monitor = std::make_shared<tesseract_monitoring::ROSEnvironmentMonitor>(env, EXAMPLE_MONITOR_NAMESPACE);
  if (rviz) {
    monitor->startPublishingEnvironment();
  }

  ROSPlottingPtr plotter;
  if (plotting) {
    plotter = std::make_shared<ROSPlotting>(env->getSceneGraph()->getRoot());
  }

  std::vector<std::string> joint_names;
  joint_names.emplace_back("ur5_shoulder_pan_joint");
  joint_names.emplace_back("ur5_shoulder_lift_joint");
  joint_names.emplace_back("ur5_elbow_joint");
  joint_names.emplace_back("ur5_wrist_1_joint");
  joint_names.emplace_back("ur5_wrist_2_joint");
  joint_names.emplace_back("ur5_wrist_3_joint");

  joint_start_pos(0) = settingsConfig.joint_start_pos_0;
  joint_start_pos(1) = settingsConfig.joint_start_pos_1;
  joint_start_pos(2) = settingsConfig.joint_start_pos_2;
  joint_start_pos(3) = settingsConfig.joint_start_pos_3;
  joint_start_pos(4) = settingsConfig.joint_start_pos_4;
  joint_start_pos(5) = settingsConfig.joint_start_pos_5;

  joint_end_pos(0) = settingsConfig.joint_end_pos_0;
  joint_end_pos(1) = settingsConfig.joint_end_pos_1;
  joint_end_pos(2) = settingsConfig.joint_end_pos_2;
  joint_end_pos(3) = settingsConfig.joint_end_pos_3;
  joint_end_pos(4) = settingsConfig.joint_end_pos_4;
  joint_end_pos(5) = settingsConfig.joint_end_pos_5;


  // Получает состояние гриппера с робота
  ros::Subscriber sub_gripper = nh.subscribe<ur5_husky_main::Gripper>("gripper_state_robot", 10, boost::bind(gripperStateCallback, _1));

  if (connect_robot) { // Соединение с роботом (в симуляции или с реальным роботом)
    ROS_INFO("Start connect with UR5 to %s ...", robot_ip.c_str());

    if (rtde->robotConnect()) {
      auto rtde_receive = rtde->getRtdeReceive();
      ROS_INFO("Connect success!");
      std::vector<double> joint_positions = rtde_receive->getActualQ();
      joints_state_current = joint_positions;
      if (joint_positions.size() == 6) {

        for (auto i = 0; i < joint_positions.size(); i++ ) {
          joint_start_pos(i) = joint_positions[i];
        }

      } else {
        ROS_ERROR("There should be 6 joints.");
      }
    }

    // TODO инициализируем состояние гриппера с робота
    // gripperAngle

  } else {
      ROS_INFO("I work without connecting to the robot.");
  }

  env->setState(joint_names, joint_start_pos);

  // Установить начальное положение JointState
  position_vector.resize(joint_start_pos.size());
  Eigen::VectorXd::Map(&position_vector[0], joint_start_pos.size()) = joint_start_pos;

  printPoseInRviz(joint_pub_state, joint_names, joint_start_pos);

  ros::Publisher messagePub = nh.advertise<std_msgs::String>("chatter", 1000);
  ros::Publisher messageRobotBusyPub = nh.advertise<std_msgs::String>("robot_busy", 1000);
  ros::Publisher messageUIPub = nh.advertise<ur5_husky_main::InfoUI>("chatter_ui", 1000);
  ros::Publisher messagePosePub = nh.advertise<ur5_husky_main::PoseList>("trajopt_pose", 1000);
  std_msgs::String msg;

  std::thread robotBusyThread(robotBusyPublish);

  ros::Publisher gripperPub = nh.advertise<ur5_husky_main::Gripper>("gripper_state", 1000);

  ros::ServiceServer setStartJointsService = nh.advertiseService<ur5_husky_main::SetJointState::Request, ur5_husky_main::SetJointState::Response>
                      ("set_joint_start_value", boost::bind(updateStartJointValue, _1, _2, env, joint_pub_state, joint_names, connect_robot, loop_rate, gripperPub, messageRobotBusyPub));

  ros::ServiceServer setFinishJointsService = nh.advertiseService<ur5_husky_main::SetJointState::Request, ur5_husky_main::SetJointState::Response>
                      ("set_joint_finish_value", boost::bind(updateFinishJointValue, _1, _2, env, joint_pub_state, joint_names, connect_robot));

  ros::ServiceServer getJointsService = nh.advertiseService<ur5_husky_main::GetJointState::Request, ur5_husky_main::GetJointState::Response>
                      ("get_joint_value", boost::bind(getJointValue, _1, _2, joint_names, joint_pub_state, env));

  ros::ServiceServer robotCalculatePlanService = nh.advertiseService<ur5_husky_main::CalculateTrajectory::Request, ur5_husky_main::CalculateTrajectory::Response>
                      ("calculate_robot_rajectory", boost::bind(calculateRobotTrajectory, _1, _2, env, plotter, joint_names, messagePosePub, loop_rate));

  ros::ServiceServer robotPlanService = nh.advertiseService<ur5_husky_main::RobotPlanTrajectory::Request, ur5_husky_main::RobotPlanTrajectory::Response>
                      ("robot_plan_trajectory", boost::bind(robotPlanTrajectoryMethod, _1, _2, messageRobotBusyPub, 
                      loop_rate, gripperPub, joint_pub_state, env, joint_names, messageUIPub));

  ros::ServiceServer robotExecuteService = nh.advertiseService<ur5_husky_main::RobotExecuteTrajectory::Request, ur5_husky_main::RobotExecuteTrajectory::Response>
                      ("robot_execute_trajectory", boost::bind(robotExecuteTrajectoryMethod, _1, _2));

  ros::ServiceServer robotRestartService = nh.advertiseService<ur5_husky_main::RobotRestart::Request, ur5_husky_main::RobotRestart::Response>
                      ("robot_restart", boost::bind(robotRestartMethod, _1, _2));

  ros::ServiceServer freedriveService = nh.advertiseService<ur5_husky_main::Freedrive::Request, ur5_husky_main::Freedrive::Response>
                      ("freedrive_change", boost::bind(freedriveEnable, _1, _2));

  ros::ServiceServer getInfoService = nh.advertiseService<ur5_husky_main::GetInfo::Request, ur5_husky_main::GetInfo::Response>
                      ("get_info_robot", boost::bind(getInfo, _1, _2, env, joint_names, loop_rate));

  ros::ServiceServer setSettingsService = nh.advertiseService<ur5_husky_main::Settings::Request, ur5_husky_main::Settings::Response>
                      ("set_settings", boost::bind(setSettings, _1, _2));

  ros::ServiceClient gripperMoveRobotService = nh.serviceClient<ur5_husky_main::GripperMoveRobot>("gripper_move_robot");
  ur5_husky_main::GripperMoveRobot srv;

  ros::ServiceServer gripperService = nh.advertiseService<ur5_husky_main::GripperService::Request, ur5_husky_main::GripperService::Response>
                      ("gripper_move", boost::bind(gripperMove, _1, _2, gripperMoveRobotService, srv));

  ros::ServiceServer robotStopService = nh.advertiseService<std_srvs::Empty::Request, std_srvs::Empty::Response>("robot_stop", boost::bind(stopMove, _1, _2, messageRobotBusyPub));

  ros::ServiceClient gsService = nh.serviceClient<ur5_husky_main::GetGripperState>("gripper_state_robot");
  ur5_husky_main::GetGripperState gripperStateSrv;

  ros::ServiceServer gripperStateService = nh.advertiseService<ur5_husky_main::GetGripperState::Request, ur5_husky_main::GetGripperState::Response>
                      ("get_gripper_state", boost::bind(getGripperState, _1, _2, gsService, gripperStateSrv));                    

  ros::Subscriber sub = nh.subscribe<ur5_husky_main::Gripper>("set_gripper_state", 10, boost::bind(gripperCallback, _1, gripperPub));

  ros::Subscriber joint_sub = nh.subscribe<ur5_husky_main::Pose>("set_robot_pose", 1, boost::bind(robotPoseCallback, _1, env, joint_pub_state, joint_names, messageRobotBusyPub, loop_rate));

  ros::ServiceServer createBoxService = nh.advertiseService<ur5_husky_main::Box::Request, ur5_husky_main::Box::Response>
                      ("create_box", boost::bind(createBox, _1, _2, env));

  ros::ServiceServer createMeshService = nh.advertiseService<ur5_husky_main::Mesh::Request, ur5_husky_main::Mesh::Response>
                      ("create_mesh", boost::bind(createMesh, _1, _2, env));

  ros::ServiceServer moveBoxService = nh.advertiseService<ur5_husky_main::Box::Request, ur5_husky_main::Box::Response>
                      ("move_box", boost::bind(moveBox, _1, _2, env));

  ros::ServiceServer moveMeshService = nh.advertiseService<ur5_husky_main::Mesh::Request, ur5_husky_main::Mesh::Response>
                      ("move_mesh", boost::bind(moveMesh, _1, _2, env));

  ros::ServiceServer removeBoxService = nh.advertiseService<ur5_husky_main::Box::Request, ur5_husky_main::Box::Response>
                      ("remove_box", boost::bind(removeBox, _1, _2, env));

  ros::ServiceServer removeMeshService = nh.advertiseService<ur5_husky_main::Mesh::Request, ur5_husky_main::Mesh::Response>
                      ("remove_mesh", boost::bind(removeMesh, _1, _2, env));

  ros::ServiceServer createObstacleGroupService = nh.advertiseService<ur5_husky_main::ObstacleGroup::Request, ur5_husky_main::ObstacleGroup::Response>
                      ("create_obstacle_group", boost::bind(createObstacleGroup, _1, _2, env));

  ros::ServiceServer deleteObstacleGroupService = nh.advertiseService<ur5_husky_main::ObstacleDeleteGroup::Request, ur5_husky_main::ObstacleDeleteGroup::Response>
                      ("delete_obstacle_group", boost::bind(deleteObstacleGroup, _1, _2, env));

  if (debug) {
    console_bridge::setLogLevel(console_bridge::LogLevel::CONSOLE_BRIDGE_LOG_DEBUG);
  }

  while(ros::ok()) {

    bool goToStart = false;
    robotRestart = true;

    // Ждем команды для старта
    std::cout << "Waiting for the command to start ... \n";
    while(ros::ok()) {
      ros::spinOnce();
      loop_rate.sleep();
      if (robotRestart) {
        break;
      }
    }

    robotRestart = false;
    plotter->clear();

    // Ждем команды на планирование траектории
    if (!robotPlanTrajectory) {
      std::cout << "Waiting for the command to plan the trajectory... \n";
      while(ros::ok()) {

        // До начала планирования можно подключить freedrive
        if (freeDriveOn) {
          msg.data = "freedrive_on";
          ROS_INFO("Sent message to UI: %s", msg.data.c_str());
          messagePub.publish(msg);

          freedriveControl(loop_rate);

          msg.data = "freedrive_off";
          ROS_INFO("Sent message to UI: %s", msg.data.c_str());
          messagePub.publish(msg);
        }

        ros::spinOnce();
        loop_rate.sleep();
        if (robotPlanTrajectory) {
          break;
        }

        if (robotRestart) {
          goToStart = true;
          break;
        }
      }

      if (goToStart) {
        continue;
      }
    }



    //////////////////////////////////////////////////
    //////////////////////////////////////////////////
    // Detach the simulated attach from the world and attach to the end effector
    tesseract_environment::Commands cmds;
    Joint joint_attach("joint_attach");
    joint_attach.parent_link_name = "ur5_tool0";
    joint_attach.child_link_name = "attach";

    if (script == "open_lock") {
      joint_attach.child_link_name = "locker_box_1";
    }

    joint_attach.type = JointType::FIXED;
    joint_attach.parent_to_joint_origin_transform = Eigen::Isometry3d::Identity();
    joint_attach.parent_to_joint_origin_transform.translation() = Eigen::Vector3d(settingsConfig.x_pos_correct, settingsConfig.y_pos_correct, settingsConfig.z_pos_correct);
    Eigen::AngleAxisd rotX(settingsConfig.x_orient_correct, Eigen::Vector3d::UnitX());
    Eigen::AngleAxisd rotY(settingsConfig.y_orient_correct, Eigen::Vector3d::UnitY());
    Eigen::AngleAxisd rotZ(settingsConfig.z_orient_correct, Eigen::Vector3d::UnitZ());
    joint_attach.parent_to_joint_origin_transform.rotate(rotX);
    joint_attach.parent_to_joint_origin_transform.rotate(rotY);
    joint_attach.parent_to_joint_origin_transform.rotate(rotZ);

    cmds.push_back(std::make_shared<tesseract_environment::MoveLinkCommand>(joint_attach));
    tesseract_common::AllowedCollisionMatrix add_ac;
    // add_ac.addAllowedCollision("cup", "ur5_tool0", "Never");
    cmds.push_back(std::make_shared<tesseract_environment::ModifyAllowedCollisionsCommand>(
        add_ac, tesseract_environment::ModifyAllowedCollisionsType::ADD));
    env->applyCommands(cmds);
    ////////////////////////////////////////////////////


    ////////////////////////////////////////////////////////////
    //
    //    Проверить, необходимо ли менять положение гриппера
    //
    /////////////////////////////////////////////////////////////
    std::vector<Eigen::VectorXd> joint_middle_pos_list_tmp; // Промежуточные позы, которые будем отправлять на робота

    std::vector<tesseract_common::JointTrajectory> trajectoryList;
    std::vector<ur5_husky_main::Gripper> gripperStates;

    bool changeGripperState = false;
    double time = 0.0;

    std::cout << "Gripper: " << std::endl;

    for (int i = 0; i < joint_middle_pos_list.size(); i++) {
      if (i < gripperPoseList.size()) {

        std::cout << gripperPoseList[i].name << " " << gripperPoseList[i].angle << std::endl;

        joint_middle_pos_list_tmp.push_back(joint_middle_pos_list[i]);
        if (gripperPoseList[i].angle != gripperPosePrev.angle) {
          changeGripperState = true;
          gripperStates.push_back(gripperPoseList[i]);

          UR5Trajopt calculate(env, plotter, joint_names, joint_start_pos, joint_end_pos, joint_middle_pos_list_tmp);
          UR5TrajoptResponce responce = calculate.run();
          bool success = responce.isSuccessful();

          // TODO подумать, что тут делать в случае ошибки (при расчете сложной траектории из множества кусков)
          if (!success) {
            ROS_ERROR("Planning (PLAN №%d) ended with errors!", i+1);
            sendMessageToUI("error_trajopt", messageUIPub, loop_rate, responce.getTime());
            robotPlanTrajectory = false;
            robotExecuteTrajectory = false;
            setRobotNotConnectErrorMes = false;
            continue;

          } else {
            tesseract_common::JointTrajectory trajectory = responce.getTrajectory();
            trajectoryList.push_back(trajectory);
            time += responce.getTime();
          }

          joint_middle_pos_list_tmp.clear();
        }

        gripperPosePrev = gripperPoseList[i];
      } else {
        joint_middle_pos_list_tmp.push_back(joint_middle_pos_list[i]);
      }
    }


    // Если в принципе не было смены состояния гриппера, то тоже считаем план
    if (!changeGripperState) {
      UR5Trajopt calculate(env, plotter, joint_names, joint_start_pos, joint_end_pos, joint_middle_pos_list);
      UR5TrajoptResponce responce = calculate.run();
      bool success = responce.isSuccessful();

      if (!success) {
        ROS_ERROR("Planning ended with errors!");
        sendMessageToUI("error_trajopt", messageUIPub, loop_rate, responce.getTime());
        robotPlanTrajectory = false;
        robotExecuteTrajectory = false;
        setRobotNotConnectErrorMes = false;
        continue;
      }

      tesseract_common::JointTrajectory trajectory = responce.getTrajectory();
      trajectoryList.push_back(trajectory);
      time = responce.getTime();
    }

    sendMessageToUI("plan_finish", messageUIPub, loop_rate, time);

    /////////////////////////////////////////////////
    //
    //   Выполнение траектории в симуляторе rViz и на роботе
    //
    /////////////////////////////////////////////////

    // TODO Проверка флага на подключение connect_robot

    char input_simbol = 'n';

    // Ждем команды на выполнение траектории
    if (!robotExecuteTrajectory) {
      std::cout << "Waiting for the command to execute the trajectory... \n";
      while(ros::ok()) {
        ros::spinOnce();
        loop_rate.sleep();
        if (robotExecuteTrajectory) {
          break;
        }

        if (robotRestart) {
          goToStart = true;
          break;
        }
      }
    }

    if (goToStart) {
      continue;
    }
    
    if (robotExecuteTrajectory || input_simbol == 'y') {
      std::cout << "Executing... \n";

      for (int j = 0; j < trajectoryList.size(); j++) {

          tesseract_common::JointTrajectory trajectory = trajectoryList[j];

          TrajectoryPlayer player;
          player.setTrajectory(trajectory);

          std::vector<tesseract_common::JointState> j_states;
              
          ROS_INFO("Added intermediate joints: ");

          // Список доступных положений робота
          std::vector<std::vector<double>> jointsPath;
          std::vector<double> path_pose;

          for (int i = 0; i < trajectory.states.size(); i++) {

            tesseract_common::JointState j_state = player.getByIndex(i);
            path_pose.resize(j_state.position.size());
            Eigen::VectorXd::Map(&path_pose[0], j_state.position.size()) = j_state.position;
            path_pose.push_back(ur_speed);
            path_pose.push_back(ur_acceleration);
            path_pose.push_back(ur_blend);
            jointsPath.push_back(path_pose);

            ROS_INFO("%d point of traectory: ", i+1);

            std::cout << "joint_names: ";
            for (const auto& name: j_state.joint_names) {
              std::cout << name.c_str() << ' ';
            }
            std::cout << std::endl;
            
            std::cout << "positions: " << j_state.position.transpose() << std::endl;
            std::cout << "velocity: " << j_state.velocity.transpose() << std::endl;
            std::cout << "acceleration: " << j_state.acceleration.transpose() << std::endl;
            std::cout << "effort: " << j_state.effort.transpose() << std::endl;
            std::cout << "====================" << std::endl;
          }

          std::cout << "jointsPath: " << jointsPath.size() << std::endl;

          // Обновить состояние до последней позиции
          position_vector.resize(joint_end_pos.size());
          Eigen::VectorXd::Map(&position_vector[0], joint_end_pos.size()) = joint_end_pos;

          path_pose.resize(position_vector.size());
          Eigen::VectorXd::Map(&path_pose[0], joint_end_pos.size()) = joint_end_pos;
          path_pose.push_back(ur_speed);
          path_pose.push_back(ur_acceleration);
          path_pose.push_back(ur_blend);
          jointsPath.push_back(path_pose);

          // Отправить на робота
          if (rtde->robotConnect()) {
            auto rtde_control = rtde->getRtdeControl();

            robotBusy = true;
            sendRobotState(messageRobotBusyPub);
            rtde_control->moveJ(jointsPath, async_move);
            //// TODO Добавить обработку асинхронного движения
            robotBusy = false;
            sendRobotState(messageRobotBusyPub);
          }

          // Поменять состояние гриппера
          std::cout << "gripperStates = " << gripperStates.size() << std::endl;
          std::cout << "j = " << j << std::endl;
          if (j < gripperStates.size()) {
            std::cout << "PUB GRIPPER STATE = " << gripperStates[j].name << " " << gripperStates[j].angle << std::endl;
            gripperPub.publish(gripperStates[j]);
          }

          // Установить состояние для tesseract
          env->setState(joint_names, joint_end_pos);

          // Сообщение для отправки конечного состояния для обновления в rViz
          sensor_msgs::JointState joint_state_msg;
          joint_state_msg.name = joint_names;
          joint_state_msg.position = position_vector;
          joint_state_msg.velocity = velocity_default; // скорость
          joint_state_msg.effort = effort_default; // усилие
          joint_pub_state.publish(joint_state_msg);
      }

      // Установить итоговое состояние гриппера после выполнения траектории
      gripperPub.publish(gripperPoseList[gripperPoseList.size()-1]);

    } else {
      std::cout << "The trajectory in the simulator will not be executed. \n";
    }

    sendMessageToUI("execute_finish", messageUIPub, loop_rate);

    robotPlanTrajectory = false;
    robotExecuteTrajectory = false;
    setRobotNotConnectErrorMes = false;
    joint_middle_pos_list_tmp.clear();
    gripperPoseList.clear();

    ros::spinOnce();
    loop_rate.sleep();
  }

  robotBusyThread.join();

  return 0;

}
