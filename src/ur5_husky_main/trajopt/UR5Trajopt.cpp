#include <tesseract_common/macros.h>
TESSERACT_COMMON_IGNORE_WARNINGS_PUSH
#include <jsoncpp/json/json.h>
#include <console_bridge/console.h>
TESSERACT_COMMON_IGNORE_WARNINGS_POP

#include "UR5Trajopt.hpp"
#include "UR5TrajoptResponce.hpp"

#include <ros/ros.h>

#include <tesseract_monitoring/environment_monitor.h>
#include <tesseract_rosutils/plotting.h>
#include <trajectory_msgs/JointTrajectory.h>
#include <sensor_msgs/JointState.h>
#include <std_msgs/String.h>
#include <tesseract_environment/utils.h>
#include <tesseract_common/timer.h>
#include <tesseract_command_language/composite_instruction.h>
#include <tesseract_command_language/state_waypoint.h>
#include <tesseract_command_language/cartesian_waypoint.h>
#include <tesseract_command_language/joint_waypoint.h>
#include <tesseract_command_language/move_instruction.h>
#include <tesseract_command_language/utils.h>
#include <tesseract_task_composer/planning/planning_task_composer_problem.h>
#include <tesseract_task_composer/core/task_composer_input.h>
#include <tesseract_task_composer/core/task_composer_plugin_factory.h>
#include <tesseract_task_composer/taskflow/taskflow_task_composer_executor.h>
#include <tesseract_visualization/markers/toolpath_marker.h>

#include <tesseract_motion_planners/core/utils.h>
#include <tesseract_motion_planners/trajopt/trajopt_motion_planner.h>

#include <tesseract_motion_planners/trajopt_ifopt/profile/trajopt_ifopt_default_composite_profile.h>
#include <tesseract_motion_planners/trajopt/profile/trajopt_default_composite_profile.h>

#include <tesseract_motion_planners/trajopt_ifopt/profile/trajopt_ifopt_default_plan_profile.h>

#include <tesseract_motion_planners/trajopt/profile/trajopt_default_plan_profile.h>
#include <tesseract_motion_planners/trajopt/profile/trajopt_default_solver_profile.h>
#include <tesseract_task_composer/planning/profiles/contact_check_profile.h>

#include <tesseract_visualization/trajectory_player.h>

#include <iostream>
#include <thread>
#include <chrono>
#include <vector>
#include <string>
#include <ctime>

using namespace tesseract_rosutils;

using namespace trajopt;
using namespace tesseract_environment;
using namespace tesseract_scene_graph;
using namespace tesseract_collision;
using namespace tesseract_visualization;
using namespace tesseract_planning;
using tesseract_common::ManipulatorInfo;

static const std::string TRAJOPT_DEFAULT_NAMESPACE = "TrajOptMotionPlannerTask";

UR5Trajopt::UR5Trajopt (tesseract_environment::Environment::Ptr env,
                        ROSPlottingPtr plotter,
                        std::vector<std::string> joint_names,
                        Eigen::VectorXd joint_start_pos,
                        Eigen::VectorXd joint_end_pos,
                        std::vector<Eigen::VectorXd> joint_middle_pos_list) {
  env_ = env;
  plotter_ = plotter;
  joint_names_ = joint_names;
  joint_start_pos_ = joint_start_pos;
  joint_end_pos_ = joint_end_pos;
  joint_middle_pos_list_ = joint_middle_pos_list;
}


UR5TrajoptResponce UR5Trajopt::run() {
  // Solve Trajectory
  CONSOLE_BRIDGE_logInform("UR5 trajopt plan");

  // Create Task Composer Plugin Factory
  const std::string share_dir(TESSERACT_TASK_COMPOSER_DIR);
  tesseract_common::fs::path config_path(share_dir + "/config/task_composer_plugins.yaml");
  TaskComposerPluginFactory factory(config_path);

  // Create Program
  CompositeInstruction program("UR5", CompositeInstructionOrder::ORDERED, ManipulatorInfo("manipulator", "ur5_base_link", "ur5_tool0"));

  // Start and End Joint Position for the program
  StateWaypointPoly wp0{ StateWaypoint(joint_names_, joint_start_pos_) };
  StateWaypointPoly wp1{ StateWaypoint(joint_names_, joint_end_pos_) };

  MoveInstruction start_instruction(wp0, MoveInstructionType::FREESPACE, "UR5");
  start_instruction.setDescription("Start Instruction");
  program.appendMoveInstruction(start_instruction);

  // Additional provisions
  for (int i = 0; i < joint_middle_pos_list_.size(); i++) {
    StateWaypointPoly wp_middle{ StateWaypoint(joint_names_, joint_middle_pos_list_[i]) };
    MoveInstruction plan_middle(wp_middle, MoveInstructionType::FREESPACE, "UR5");
    std::string description = "freespace_middle_plan №" + std::to_string(i+1);

    plan_middle.setDescription(description);
    program.appendMoveInstruction(plan_middle);
  }

  MoveInstruction plan_f0(wp1, MoveInstructionType::FREESPACE, "UR5");
  plan_f0.setDescription("freespace_finish_plan");
  program.appendMoveInstruction(plan_f0);

  // Print Diagnostics
  program.print("Program: ");

  // Create Executor
  auto executor = factory.createTaskComposerExecutor("TaskflowExecutor");

  // Create profile dictionary
  auto profiles = std::make_shared<ProfileDictionary>();

  // Тип коллизии задается в настройках
  trajopt::CollisionEvaluatorType collisionCostConfigType;
  trajopt::CollisionEvaluatorType collisionConstraintConfigType;
  // if (settings_config_.collision_cost_config_type == "SINGLE_TIMESTEP") {
  //     collisionCostConfigType = trajopt::CollisionEvaluatorType::SINGLE_TIMESTEP;
  // } else if (settings_config_.collision_cost_config_type == "CAST_CONTINUOUS") {
  //     collisionCostConfigType = trajopt::CollisionEvaluatorType::CAST_CONTINUOUS;
  // } else if (settings_config_.collision_constraint_config_type == "SINGLE_TIMESTEP") {
  //     collisionConstraintConfigType = trajopt::CollisionEvaluatorType::SINGLE_TIMESTEP;
  // } else if (settings_config_.collision_constraint_config_type == "CAST_CONTINUOUS") {
  //     collisionConstraintConfigType = trajopt::CollisionEvaluatorType::CAST_CONTINUOUS;
  // } else { // DISCRETE_CONTINUOUS - вариант по умолчанию
      collisionCostConfigType = trajopt::CollisionEvaluatorType::DISCRETE_CONTINUOUS;
      collisionConstraintConfigType = trajopt::CollisionEvaluatorType::DISCRETE_CONTINUOUS;
  // }

  auto composite_profile = std::make_shared<TrajOptDefaultCompositeProfile>();
  // composite_profile->longest_valid_segment_length = 0.05;
  // composite_profile->collision_cost_config.enabled = true;
  composite_profile->collision_cost_config.type = collisionCostConfigType;
  composite_profile->collision_cost_config.safety_margin = 0.005;
  composite_profile->collision_cost_config.safety_margin_buffer = 0.01;
  composite_profile->collision_cost_config.coeff = 50;

  // composite_profile->collision_constraint_config.enabled = true;
  composite_profile->collision_constraint_config.type = collisionConstraintConfigType;
  composite_profile->collision_constraint_config.safety_margin = 0.0;
  composite_profile->collision_constraint_config.safety_margin_buffer = 0.005;
  composite_profile->collision_constraint_config.coeff = 10;
  // composite_profile->smooth_velocities = true;
  // composite_profile->smooth_accelerations = false;
  // composite_profile->smooth_jerks = false;
  // composite_profile->velocity_coeff = Eigen::VectorXd::Ones(1);
  profiles->addProfile<TrajOptCompositeProfile>(TRAJOPT_DEFAULT_NAMESPACE, "UR5", composite_profile);

  auto plan_profile = std::make_shared<TrajOptDefaultPlanProfile>();
  plan_profile->cartesian_coeff = Eigen::VectorXd::Constant(6, 1, 10);
  // plan_profile->cartesian_coeff(0) = 0;
  // plan_profile->cartesian_coeff(1) = 0;
  // plan_profile->cartesian_coeff(2) = 0;

  // Add profile to Dictionary
  profiles->addProfile<TrajOptPlanProfile>(TRAJOPT_DEFAULT_NAMESPACE, "UR5", plan_profile);


  auto trajopt_solver_profile = std::make_shared<TrajOptDefaultSolverProfile>();
  trajopt_solver_profile->opt_info.max_iter = 100;
  trajopt_solver_profile->opt_info.log_results = true;

  time_t rawtime;
  struct tm * timeinfo;
  char buffer[80];

  time (&rawtime);
  timeinfo = localtime(&rawtime);
  strftime(buffer, sizeof(buffer),"%d-%m-%Y-%H-%M-%S",timeinfo);
  std::string datetime(buffer);

  std::cout << "datetime: " << datetime << std::endl;

  trajopt_solver_profile->opt_info.log_dir = "/home/lena/trajopt/logs/solver";

  profiles->addProfile<TrajOptSolverProfile>(TRAJOPT_DEFAULT_NAMESPACE, "UR5", trajopt_solver_profile);

  auto post_check_profile = std::make_shared<ContactCheckProfile>();
  profiles->addProfile<ContactCheckProfile>(TRAJOPT_DEFAULT_NAMESPACE, "UR5", post_check_profile);

 
  // Create task
  const std::string task_name = "TrajOptPipeline";
  TaskComposerNode::UPtr task = factory.createTaskComposerNode(task_name);
  const std::string input_key = task->getInputKeys().front();
  const std::string output_key = task->getOutputKeys().front();

  // Create Task Input Data
  TaskComposerDataStorage input_data;
  input_data.setData(input_key, program);

  // Create Task Composer Problem
  auto problem = std::make_unique<PlanningTaskComposerProblem>(env_, input_data, profiles);

  // Solve process plan
  tesseract_common::Timer stopwatch;
  stopwatch.start();
  TaskComposerInput input(std::move(problem));
  TaskComposerFuture::UPtr future = executor->run(*task, input);
  future->wait();
  stopwatch.stop();
  CONSOLE_BRIDGE_logInform("Planning took %f seconds.", stopwatch.elapsedSeconds());


  auto ci = input.data_storage.getData(output_key).as<CompositeInstruction>();
  tesseract_common::JointTrajectory trajectory = toJointTrajectory(ci);
  std::vector<tesseract_planning::InstructionPoly> points = ci.getInstructions();

  // Смог ли спланировать
  // Create Planning Request
  PlannerRequest request;
  request.instructions = ci;
  request.env = env_;
  request.env_state = env_->getState();
  request.profiles = profiles;

  // Solve TrajOpt Plan
  TrajOptMotionPlanner planner(TRAJOPT_DEFAULT_NAMESPACE);
  PlannerResponse planResponse = planner.solve(request);

  // Plot Process Trajectory
  if (plotter_ != nullptr && plotter_->isConnected() && planResponse.successful) {
    tesseract_common::Toolpath toolpath = toToolpath(ci, *env_);
    auto state_solver = env_->getStateSolver();
    auto scene_state = env_->getState();

    auto marker = ToolpathMarker(toolpath);
    marker.scale = Eigen::Vector3d::Constant(0.07);

    plotter_->plotMarker(marker);
    plotter_->plotTrajectory(trajectory, *state_solver);
  }

  UR5TrajoptResponce responce(trajectory, planResponse.successful, planResponse.message, stopwatch.elapsedSeconds());

  return responce;
}