/**
 * @file puzzle_piece_example.cpp
 * @brief Puzzle piece example implementation
 *
 * @author Levi Armstrong
 * @date July 22, 2019
 * @version TODO
 * @bug No known bugs
 *
 * @copyright Copyright (c) 2017, Southwest Research Institute
 *
 * @par License
 * Software License Agreement (Apache License)
 * @par
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 * http://www.apache.org/licenses/LICENSE-2.0
 * @par
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */
#include <tesseract_common/macros.h>
TESSERACT_COMMON_IGNORE_WARNINGS_PUSH
#include <ros/ros.h>
#include <fstream>
TESSERACT_COMMON_IGNORE_WARNINGS_POP

#include <tesseract_ros_examples/puzzle_piece_example.h>
#include <tesseract_environment/core/utils.h>
#include <tesseract_rosutils/plotting.h>
#include <tesseract_rosutils/utils.h>
#include <tesseract_command_language/command_language.h>
#include <tesseract_command_language/types.h>
#include <tesseract_command_language/utils/utils.h>
#include <tesseract_process_managers/core/process_input.h>
#include <tesseract_process_managers/taskflow_generators/trajopt_taskflow.h>
#include <tesseract_motion_planners/trajopt/profile/trajopt_default_plan_profile.h>
#include <tesseract_motion_planners/trajopt/profile/trajopt_default_composite_profile.h>
#include <tesseract_motion_planners/trajopt/profile/trajopt_default_solver_profile.h>
#include <tesseract_motion_planners/core/utils.h>
#include <tesseract_planning_server/tesseract_planning_server.h>

using namespace trajopt;
using namespace tesseract_environment;
using namespace tesseract_scene_graph;
using namespace tesseract_collision;
using namespace tesseract_rosutils;

/** @brief Default ROS parameter for robot description */
const std::string ROBOT_DESCRIPTION_PARAM = "robot_description";

/** @brief Default ROS parameter for robot description */
const std::string ROBOT_SEMANTIC_PARAM = "robot_description_semantic";

/** @brief RViz Example Namespace */
const std::string EXAMPLE_MONITOR_NAMESPACE = "tesseract_ros_examples";

namespace tesseract_ros_examples
{
tesseract_common::VectorIsometry3d PuzzlePieceExample::makePuzzleToolPoses()
{
  tesseract_common::VectorIsometry3d path;  // results
  std::ifstream indata;                     // input file

  // You could load your parts from anywhere, but we are transporting them with
  // the git repo
  std::string filename = ros::package::getPath("tesseract_ros_examples") + "/config/puzzle_bent.csv";

  // In a non-trivial app, you'll of course want to check that calls like 'open'
  // succeeded
  indata.open(filename);

  std::string line;
  int lnum = 0;
  while (std::getline(indata, line))
  {
    ++lnum;
    if (lnum < 3)
      continue;

    std::stringstream lineStream(line);
    std::string cell;
    Eigen::Matrix<double, 6, 1> xyzijk;
    int i = -2;
    while (std::getline(lineStream, cell, ','))
    {
      ++i;
      if (i == -1)
        continue;

      xyzijk(i) = std::stod(cell);
    }

    Eigen::Vector3d pos = xyzijk.head<3>();
    pos = pos / 1000.0;  // Most things in ROS use meters as the unit of length.
                         // Our part was exported in mm.
    Eigen::Vector3d norm = xyzijk.tail<3>();
    norm.normalize();

    // This code computes two extra directions to turn the normal direction into
    // a full defined frame. Descartes
    // will search around this frame for extra poses, so the exact values do not
    // matter as long they are valid.
    Eigen::Vector3d temp_x = (-1 * pos).normalized();
    Eigen::Vector3d y_axis = (norm.cross(temp_x)).normalized();
    Eigen::Vector3d x_axis = (y_axis.cross(norm)).normalized();
    Eigen::Isometry3d pose;
    pose.matrix().col(0).head<3>() = x_axis;
    pose.matrix().col(1).head<3>() = y_axis;
    pose.matrix().col(2).head<3>() = norm;
    pose.matrix().col(3).head<3>() = pos;

    path.push_back(pose);
  }
  indata.close();

  return path;
}

// ProblemConstructionInfo PuzzlePieceExample::cppMethod()
//{
//  ProblemConstructionInfo pci(tesseract_);

//  tesseract_common::VectorIsometry3d tool_poses = makePuzzleToolPoses();

//  // Populate Basic Info
//  pci.basic_info.n_steps = static_cast<int>(tool_poses.size());
//  pci.basic_info.manip = "manipulator";
//  pci.basic_info.start_fixed = false;
//  pci.basic_info.use_time = false;
//  pci.basic_info.convex_solver = sco::ModelType::BPMPD;

//  pci.opt_info.max_iter = 200;
//  pci.opt_info.min_approx_improve = 1e-3;
//  pci.opt_info.min_trust_box_size = 1e-3;

//  // Create Kinematic Object
//  pci.kin = pci.getManipulator(pci.basic_info.manip);

//  // Populate Init Info
//  EnvState::ConstPtr current_state = pci.env->getCurrentState();
//  Eigen::VectorXd start_pos;
//  start_pos.resize(pci.kin->numJoints());
//  int cnt = 0;
//  for (const auto& j : pci.kin->getJointNames())
//  {
//    start_pos[cnt] = current_state->joints.at(j);
//    ++cnt;
//  }

//  pci.init_info.type = InitInfo::GIVEN_TRAJ;
//  pci.init_info.data = start_pos.transpose().replicate(pci.basic_info.n_steps, 1);
//  //  pci.init_info.data.col(6) = VectorXd::LinSpaced(steps_, start_pos[6],
//  //  end_pos[6]);

//  // Populate Cost Info
//  auto joint_vel = std::make_shared<JointVelTermInfo>();
//  joint_vel->coeffs = std::vector<double>(7, 1.0);
//  joint_vel->targets = std::vector<double>(7, 0.0);
//  joint_vel->first_step = 0;
//  joint_vel->last_step = pci.basic_info.n_steps - 1;
//  joint_vel->name = "joint_vel";
//  joint_vel->term_type = TT_COST;
//  pci.cost_infos.push_back(joint_vel);

//  auto joint_acc = std::make_shared<JointAccTermInfo>();
//  joint_acc->coeffs = std::vector<double>(7, 2.0);
//  joint_acc->targets = std::vector<double>(7, 0.0);
//  joint_acc->first_step = 0;
//  joint_acc->last_step = pci.basic_info.n_steps - 1;
//  joint_acc->name = "joint_acc";
//  joint_acc->term_type = TT_COST;
//  pci.cost_infos.push_back(joint_acc);

//  auto joint_jerk = std::make_shared<JointJerkTermInfo>();
//  joint_jerk->coeffs = std::vector<double>(7, 5.0);
//  joint_jerk->targets = std::vector<double>(7, 0.0);
//  joint_jerk->first_step = 0;
//  joint_jerk->last_step = pci.basic_info.n_steps - 1;
//  joint_jerk->name = "joint_jerk";
//  joint_jerk->term_type = TT_COST;
//  pci.cost_infos.push_back(joint_jerk);

//  auto collision = std::make_shared<CollisionTermInfo>();
//  collision->name = "collision";
//  collision->term_type = TT_COST;
//  collision->evaluator_type = trajopt::CollisionEvaluatorType::SINGLE_TIMESTEP;
//  collision->first_step = 0;
//  collision->last_step = pci.basic_info.n_steps - 1;
//  collision->info = createSafetyMarginDataVector(pci.basic_info.n_steps, 0.025, 20);
//  pci.cost_infos.push_back(collision);

//  // Populate Constraints
//  Eigen::Isometry3d grinder_frame = env_->getLinkTransform("grinder_frame");
//  Eigen::Quaterniond q(grinder_frame.linear());

//  Eigen::Vector3d stationary_xyz = grinder_frame.translation();
//  Eigen::Vector4d stationary_wxyz = Eigen::Vector4d(q.w(), q.x(), q.y(), q.z());

//  for (auto i = 0; i < pci.basic_info.n_steps; ++i)
//  {
//    auto pose = std::make_shared<CartPoseTermInfo>();
//    pose->term_type = TT_CNT;
//    pose->name = "waypoint_cart_" + std::to_string(i);
//    pose->link = "part";
//    pose->tcp = tool_poses[static_cast<unsigned long>(i)];
//    pose->timestep = i;
//    pose->xyz = stationary_xyz;
//    pose->wxyz = stationary_wxyz;
//    pose->pos_coeffs = Eigen::Vector3d(10, 10, 10);
//    pose->rot_coeffs = Eigen::Vector3d(10, 10, 0);

//    pci.cnt_infos.push_back(pose);
//  }

//  return pci;
//}

bool PuzzlePieceExample::run()
{
  using tesseract_planning::CartesianWaypoint;
  using tesseract_planning::CompositeInstruction;
  using tesseract_planning::CompositeInstructionOrder;
  using tesseract_planning::Instruction;
  using tesseract_planning::ManipulatorInfo;
  using tesseract_planning::PlanInstruction;
  using tesseract_planning::PlanInstructionType;
  using tesseract_planning::ProcessPlanningFuture;
  using tesseract_planning::ProcessPlanningRequest;
  using tesseract_planning::ProcessPlanningServer;
  using tesseract_planning::StateWaypoint;
  using tesseract_planning::ToolCenterPoint;
  using tesseract_planning::Waypoint;
  using tesseract_planning_server::ROSProcessEnvironmentCache;

  console_bridge::setLogLevel(console_bridge::LogLevel::CONSOLE_BRIDGE_LOG_DEBUG);

  // Initial setup
  std::string urdf_xml_string, srdf_xml_string;
  nh_.getParam(ROBOT_DESCRIPTION_PARAM, urdf_xml_string);
  nh_.getParam(ROBOT_SEMANTIC_PARAM, srdf_xml_string);

  ResourceLocator::Ptr locator = std::make_shared<tesseract_rosutils::ROSResourceLocator>();
  if (!env_->init<OFKTStateSolver>(urdf_xml_string, srdf_xml_string, locator))
    return false;

  // Create monitor
  monitor_ = std::make_shared<tesseract_monitoring::EnvironmentMonitor>(env_, EXAMPLE_MONITOR_NAMESPACE);
  if (rviz_)
    monitor_->startPublishingEnvironment(tesseract_monitoring::EnvironmentMonitor::UPDATE_ENVIRONMENT);

  // Create plotting tool
  ROSPlottingPtr plotter = std::make_shared<ROSPlotting>(monitor_->getSceneGraph()->getRoot());
  plotter->init(env_);
  if (rviz_)
    plotter->waitForConnection();

  // Set the robot initial state
  std::vector<std::string> joint_names;
  joint_names.push_back("joint_a1");
  joint_names.push_back("joint_a2");
  joint_names.push_back("joint_a3");
  joint_names.push_back("joint_a4");
  joint_names.push_back("joint_a5");
  joint_names.push_back("joint_a6");
  joint_names.push_back("joint_a7");

  Eigen::VectorXd joint_pos(7);
  joint_pos(0) = -0.785398;
  joint_pos(1) = 0.4;
  joint_pos(2) = 0.0;
  joint_pos(3) = -1.9;
  joint_pos(4) = 0.0;
  joint_pos(5) = 1.0;
  joint_pos(6) = 0.0;

  env_->setState(joint_names, joint_pos);

  // Get Tool Poses
  tesseract_common::VectorIsometry3d tool_poses = makePuzzleToolPoses();

  // Create manipulator information for program
  ManipulatorInfo mi;
  mi.manipulator = "manipulator";
  mi.working_frame = "part";
  mi.tcp = ToolCenterPoint("grinder_frame", true);  // true - indicates this is an external TCP

  // Create Program
  CompositeInstruction program("DEFAULT", CompositeInstructionOrder::ORDERED, mi);

  // Create cartesian waypoint
  Waypoint wp = CartesianWaypoint(tool_poses[0]);
  PlanInstruction plan_instruction(wp, PlanInstructionType::START, "CARTESIAN");
  plan_instruction.setDescription("from_start_plan");
  program.setStartInstruction(plan_instruction);

  for (std::size_t i = 1; i < tool_poses.size(); ++i)
  {
    Waypoint wp = CartesianWaypoint(tool_poses[i]);
    PlanInstruction plan_instruction(wp, PlanInstructionType::LINEAR, "CARTESIAN");
    plan_instruction.setDescription("waypoint_" + std::to_string(i));
    program.push_back(plan_instruction);
  }

  // Create Process Planning Server
  ProcessPlanningServer planning_server(std::make_shared<ROSProcessEnvironmentCache>(monitor_), 5);
  planning_server.loadDefaultProcessPlanners();

  // Create a trajopt taskflow without post collision checking
  /** @todo This matches the original example, but should update to include post collision check */
  const std::string new_planner_name = "TRAJOPT_NO_POST_CHECK";
  tesseract_planning::TrajOptTaskflowParams params;
  params.enable_post_contact_discrete_check = false;
  params.enable_post_contact_continuous_check = false;
  planning_server.registerProcessPlanner(new_planner_name,
                                         std::make_unique<tesseract_planning::TrajOptTaskflow>(params));

  // Create TrajOpt Profile
  auto trajopt_plan_profile = std::make_shared<tesseract_planning::TrajOptDefaultPlanProfile>();
  trajopt_plan_profile->cartesian_coeff = Eigen::VectorXd::Constant(6, 1, 10);
  trajopt_plan_profile->cartesian_coeff(5) = 0;

  auto trajopt_composite_profile = std::make_shared<tesseract_planning::TrajOptDefaultCompositeProfile>();
  trajopt_composite_profile->collision_constraint_config.enabled = false;
  trajopt_composite_profile->collision_cost_config.enabled = true;
  trajopt_composite_profile->collision_cost_config.safety_margin = 0.025;
  trajopt_composite_profile->collision_cost_config.type = trajopt::CollisionEvaluatorType::SINGLE_TIMESTEP;
  trajopt_composite_profile->collision_cost_config.coeff = 20;

  auto trajopt_solver_profile = std::make_shared<tesseract_planning::TrajOptDefaultSolverProfile>();
  trajopt_solver_profile->convex_solver = sco::ModelType::BPMPD;
  trajopt_solver_profile->opt_info.max_iter = 200;
  trajopt_solver_profile->opt_info.min_approx_improve = 1e-3;
  trajopt_solver_profile->opt_info.min_trust_box_size = 1e-3;

  // Add profile to Dictionary
  planning_server.getProfiles()->addProfile<tesseract_planning::TrajOptPlanProfile>("CARTESIAN", trajopt_plan_profile);
  planning_server.getProfiles()->addProfile<tesseract_planning::TrajOptCompositeProfile>("DEFAULT",
                                                                                         trajopt_composite_profile);
  planning_server.getProfiles()->addProfile<tesseract_planning::TrajOptSolverProfile>("DEFAULT",
                                                                                      trajopt_solver_profile);
  // Create Process Planning Request
  ProcessPlanningRequest request;
  request.name = new_planner_name;
  request.instructions = Instruction(program);

  // Create Naive Seed
  /** @todo Need to improve simple planners to support external tcp definitions */
  tesseract_planning::CompositeInstruction naive_seed;
  {
    auto lock = monitor_->lockEnvironmentRead();
    naive_seed = tesseract_planning::generateNaiveSeed(program, *(monitor_->getEnvironment()));
  }
  request.seed = Instruction(naive_seed);

  // Print Diagnostics
  request.instructions.print("Program: ");
  request.seed.print("Seed: ");

  if (rviz_)
    plotter->waitForInput();

  // Solve process plan
  ProcessPlanningFuture response = planning_server.run(request);
  planning_server.waitForAll();

  // Plot Process Trajectory
  if (plotter != nullptr && plotter->isConnected())
  {
    plotter->waitForInput();
    plotter->plotTrajectory(*(response.results));
  }

  ROS_INFO("Final trajectory is collision free");
  return true;
}
}  // namespace tesseract_ros_examples
