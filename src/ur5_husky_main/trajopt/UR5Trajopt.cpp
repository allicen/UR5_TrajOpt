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

#include <tesseract_motion_planners/trajopt/trajopt_collision_config.h>

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
  // DISCRETE_CONTINUOUS - вариант по умолчанию
  collisionCostConfigType = trajopt::CollisionEvaluatorType::DISCRETE_CONTINUOUS;
  collisionConstraintConfigType = trajopt::CollisionEvaluatorType::DISCRETE_CONTINUOUS;


  ////////      SETTINGS START      ///////////

  /////////////////////////////////////////////
  //                                         //
  //   1. TrajOptDefaultCompositeProfile     //
  //                                         //
  /////////////////////////////////////////////

  auto composite_profile = std::make_shared<TrajOptDefaultCompositeProfile>();

  // Тип контактного теста, который необходимо выполнить: FIRST, CLOSEST, ALL
  // FIRST - Возврат при первом контакте для любой пары объектов
  // CLOSEST - Возвращает глобальный минимум для пары объектов
  // ALL - Возвращает все контакты для пары объектов
  // LIMITED - Возвращает ограниченный набор контактов для пары объектов
  composite_profile->contact_test_type = tesseract_collision::ContactTestType::ALL; // default = ALL

  // Если значение true, то для всех временных шагов будет применена общая стоимость скорости с целевым значением 0
  composite_profile->smooth_velocities = true; // default = true

  // Это значение по умолчанию для всех соединений, но позволяет вам взвешивать различные соединения
  composite_profile->velocity_coeff = {}; // default = {}

  // Если значение true, то для всех временных шагов будет применена общая стоимость ускорения с целевым значением 0
  composite_profile->smooth_accelerations = true; // default = true

  // Это значение по умолчанию для всех соединений, но позволяет вам взвешивать различные соединения
  composite_profile->acceleration_coeff = {}; // default = {}

  // Если значение true, то для всех временных шагов будет применена стоимость совместного рывка с целевым значением
  composite_profile->smooth_jerks = true; // default = true

  // Это значение по умолчанию для всех соединений, но позволяет вам взвешивать различные соединения
  composite_profile->jerk_coeff = {}; // default = {}

  // Если true, применяется стоимость, позволяющая избежать кинематических особенностей
  composite_profile->avoid_singularity = false; // default = false

  // Оптимизация веса, связанная с предотвращением кинематической сингулярности
  composite_profile->avoid_singularity_coeff = 5.0; // default = 5.0

  // Установите разрешение, при котором необходимо проверить действительность состояния для того, чтобы движение
  // между двумя состояниями было осуществлено будет считаться действительным при последующей проверке траектории,
  // возвращаемой trajopt Разрешение равно longest_valid_segment_fraction * state_space.getMaximumExtent()
  // Примечание: Планировщик придерживается консервативного подхода либо longest_valid_segment_fraction или
  // longest_valid_segment_length.
  composite_profile->longest_valid_segment_fraction = 0.01; // default = 0.01

  // Установите разрешение, при котором необходимо проверить действительность состояния для того, чтобы движение
  // между двумя состояниями было осуществлено чтобы считаться действительным. Если If norm(state1 - state0) >
  // longest_valid_segment_length. Примечание: Это преобразуется в longest_valid_segment_fraction.
  // longest_valid_segment_fraction = longest_valid_segment_length / state_space.getMaximumExtent()
  composite_profile->longest_valid_segment_length = 0.1; // default = 0.1

  // Расстояния, ограничивающие столкновение специальных звеньев
  composite_profile->special_collision_constraint = nullptr; // default = nullptr

  // Информация о конфигурации для коллизий, которые моделируются как затраты
  /////////////////////////////////////////////
  //                                         //
  //           CollisionCostConfig           //
  //                                         //
  /////////////////////////////////////////////

  // Если значение true, к проблеме будет добавлено условие стоимости столкновения.
  composite_profile->collision_cost_config.enabled = true; // default = true
  
  // Используйте взвешенную сумму для каждой пары связей. Это уменьшает количество уравнений, добавляемых к задаче
  // Если установлено значение true, рекомендуется начинать с коэффициента, установленного равным единице
  composite_profile->collision_cost_config.use_weighted_sum = false; // default = false
  
  // Тип вычислителя, который будет использоваться для проверки на столкновение.
  composite_profile->collision_cost_config.type = collisionCostConfigType; // default = DISCRETE_CONTINUOUS
  
  // Максимальное расстояние, на котором будут оцениваться затраты на столкновение
  composite_profile->collision_cost_config.safety_margin = 0.025; // default = 0.025
  
  // Расстояние за пределами buffer_margin, в котором будет оцениваться оптимизация коллизий.
  // По умолчанию это значение равно 0 (фактически отключено) для учета затрат на коллизии.
  composite_profile->collision_cost_config.safety_margin_buffer = 0.0; // default = 0.0
  
  // Коэффициент столкновения / вес
  composite_profile->collision_cost_config.coeff = 20; // default = 20


  // Информация о конфигурации для коллизий, которые моделируются как ограничения
  /////////////////////////////////////////////
  //                                         //
  //        CollisionConstraintConfig        //
  //                                         //
  /////////////////////////////////////////////

  // Если значение true, к проблеме будет добавлено условие стоимости столкновения
  composite_profile->collision_constraint_config.enabled = true; // default = true

  // Используйте взвешенную сумму для каждой пары связей. Это уменьшает количество уравнений, добавляемых к задаче
  // Если установлено значение true, рекомендуется начинать с коэффициента, равного единице.
  composite_profile->collision_constraint_config.use_weighted_sum = false; // default = false

  // Тип вычислителя, который будет использоваться для проверки коллизий 
  composite_profile->collision_constraint_config.type = collisionConstraintConfigType; // default = DISCRETE_CONTINUOUS
  
  // Максимальное расстояние, на котором будут оцениваться ограничения на столкновение.
  composite_profile->collision_constraint_config.safety_margin = 0.01; // default = 0.01

  // Расстояние за пределами safety_margin, на котором будет оцениваться оптимизация столкновения.
  composite_profile->collision_constraint_config.safety_margin_buffer = 0.05; // default = 0.05

  // Коэффициент столкновения/вес
  composite_profile->collision_constraint_config.coeff = 20; // default = 20

  // Добавление профиля в словарь
  profiles->addProfile<TrajOptCompositeProfile>(TRAJOPT_DEFAULT_NAMESPACE, "UR5", composite_profile);


  /////////////////////////////////////////////
  //                                         //
  //     2. TrajOptDefaultPlanProfile        //
  //                                         //
  /////////////////////////////////////////////
  auto plan_profile = std::make_shared<TrajOptDefaultPlanProfile>();
  
  plan_profile->cartesian_coeff = Eigen::VectorXd::Constant(1, 1, 5); // default = (1, 1, 5)

  plan_profile->joint_coeff = Eigen::VectorXd::Constant(1, 1, 5); // default = (1, 1, 5)

  plan_profile->term_type = trajopt::TermType::TT_CNT; // default = TT_CNT

  // Добавление профиля в словарь
  profiles->addProfile<TrajOptPlanProfile>(TRAJOPT_DEFAULT_NAMESPACE, "UR5", plan_profile);


  /////////////////////////////////////////////
  //                                         //
  //     3. TrajOptDefaultSolverProfile      //
  //                                         //
  /////////////////////////////////////////////

  auto trajopt_solver_profile = std::make_shared<TrajOptDefaultSolverProfile>();

  // Используемый выпуклый решатель
  trajopt_solver_profile->convex_solver = sco::ModelType::OSQP; // default = OSQP

  // Конфигурация convex solver для использования, если NULL используются настройки по умолчанию
  trajopt_solver_profile->convex_solver_config = nullptr; // default = nullptr

  /////////////////////////////////////////////
  //                                         //
  //     BasicTrustRegionSQPParameters       //
  //                                         //
  /////////////////////////////////////////////

  // !! Значения по умолчанию не заданы !!
  // но можно задать значения, перед этим раскомментировать

  // trajopt_solver_profile->opt_info.improve_ratio_threshold = 0;   // минимальное соотношение true_improve/approx_improve
  //                                                                 // принять шаг
  // trajopt_solver_profile->opt_info.min_trust_box_size = 0;        // если область доверия станет еще меньше, выйдите и
  //                                                                 // отчет о сходимости
  // trajopt_solver_profile->opt_info.min_approx_improve = 0;        // если модель улучшается меньше этого, выйдите и
  //                                                                 // отчет о сходимости
  // trajopt_solver_profile->opt_info.min_approx_improve_frac = 0;   // если модель улучшается меньше этого, выйдите и
  //                                                                 // отчет о сходимости
  // trajopt_solver_profile->opt_info.max_iter = 0;                  // Максимальное количество итераций
  // trajopt_solver_profile->opt_info.trust_shrink_ratio = 0;        // если улучшение меньше, чем
  //                                                                 // improve_ratio_threshold, сократите область доверия за счет
  //                                                                 // это соотношения
  // trajopt_solver_profile->opt_info.trust_expand_ratio = 0;        // если улучшение меньше, чем
  //                                                                 // improve_ratio_threshold, сократите область доверия за счет
  //                                                                 // это соотношения
  // trajopt_solver_profile->opt_info.cnt_tolerance = 0;             // после сходимости штрафной подзадачи, если
  //                                                                 // нарушение ограничений - это нечто меньшее, чем это, мы закончили

  // // Максимальное количество раз, в которое будет увеличена стоимость ограничений
  // trajopt_solver_profile->opt_info.max_merit_coeff_increases = 0;

  // // Максимальное количество раз, когда QP-решатель может выйти из строя, прежде чем оптимизация будет прервана
  // trajopt_solver_profile->opt_info.max_qp_solver_failures = 0;

  // trajopt_solver_profile->opt_info.merit_coeff_increase_ratio = 0;// соотношение, при котором мы увеличиваем коэффициент каждый раз

  // // Максимальное время в секундах, в течение которого будет запущен оптимизатор
  // trajopt_solver_profile->opt_info.max_time = 0;

  // // Начальный коэффициент, который используется для масштабирования ограничений. Общая стоимость ограничений равна
  // // constant_value * coeff * merit_coeff
  // trajopt_solver_profile->opt_info.initial_merit_error_coeff = 0;

  // // Если значение true, коэффициенты заслуг будут завышены только для тех ограничений, которые не сработали.
  // // Это может помочь, когда ограничений много
  // trajopt_solver_profile->opt_info.inflate_constraints_individually = true;
  // trajopt_solver_profile->opt_info.trust_box_size = 0;  // текущий размер доверительного региона (по компонентам)

  // Заносите результаты в файл
  trajopt_solver_profile->opt_info.log_results = true;

  // Каталог для хранения результатов журнала (по умолчанию: /tmp)
  trajopt_solver_profile->opt_info.log_dir = "/home/lena/trajopt/logs/solver";

  // Добавление профиля в словарь
  profiles->addProfile<TrajOptSolverProfile>(TRAJOPT_DEFAULT_NAMESPACE, "UR5", trajopt_solver_profile);

  ////////      SETTINGS FINISH      ///////////


  time_t rawtime;
  struct tm * timeinfo;
  char buffer[80];

  time (&rawtime);
  timeinfo = localtime(&rawtime);
  strftime(buffer, sizeof(buffer),"%d-%m-%Y-%H-%M-%S",timeinfo);
  std::string datetime(buffer);

  std::cout << "datetime: " << datetime << std::endl;

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