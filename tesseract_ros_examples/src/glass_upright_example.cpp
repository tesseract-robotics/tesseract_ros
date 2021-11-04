/**
 * @file glass_upright_example.cpp
 * @brief Glass upright example implementation
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
#include <jsoncpp/json/json.h>
#include <ros/ros.h>
TESSERACT_COMMON_IGNORE_WARNINGS_POP

#include <tesseract_ros_examples/glass_upright_example.h>
#include <tesseract_environment/utils.h>
#include <tesseract_rosutils/plotting.h>
#include <tesseract_rosutils/utils.h>
#include <tesseract_command_language/command_language.h>
#include <tesseract_command_language/utils/utils.h>
#include <tesseract_planning_server/tesseract_planning_server.h>
#include <tesseract_motion_planners/core/utils.h>
#include <tesseract_visualization/markers/toolpath_marker.h>

#include <tesseract_motion_planners/trajopt_ifopt/profile/trajopt_ifopt_default_composite_profile.h>
#include <tesseract_motion_planners/trajopt/profile/trajopt_default_composite_profile.h>

#include <tesseract_motion_planners/trajopt_ifopt/profile/trajopt_ifopt_default_plan_profile.h>
#include <tesseract_motion_planners/trajopt/profile/trajopt_default_plan_profile.h>

using namespace trajopt;
using namespace tesseract_environment;
using namespace tesseract_scene_graph;
using namespace tesseract_collision;
using namespace tesseract_rosutils;
using namespace tesseract_visualization;

/** @brief Default ROS parameter for robot description */
const std::string ROBOT_DESCRIPTION_PARAM = "robot_description";

/** @brief Default ROS parameter for robot description */
const std::string ROBOT_SEMANTIC_PARAM = "robot_description_semantic";

/** @brief RViz Example Namespace */
const std::string EXAMPLE_MONITOR_NAMESPACE = "tesseract_ros_examples";

namespace tesseract_ros_examples
{
GlassUprightExample::GlassUprightExample(const ros::NodeHandle& nh,
                                         bool plotting,
                                         bool rviz,
                                         bool write_to_file,
                                         bool ifopt,
                                         bool debug)
  : Example(plotting, rviz), nh_(nh), write_to_file_(write_to_file), ifopt_(ifopt), debug_(debug)
{
}

tesseract_environment::Command::Ptr GlassUprightExample::addSphere()
{
  // Add sphere to environment
  Link link_sphere("sphere_attached");

  Visual::Ptr visual = std::make_shared<Visual>();
  visual->origin = Eigen::Isometry3d::Identity();
  visual->origin.translation() = Eigen::Vector3d(0.5, 0, 0.55);
  visual->geometry = std::make_shared<tesseract_geometry::Sphere>(0.15);
  link_sphere.visual.push_back(visual);

  Collision::Ptr collision = std::make_shared<Collision>();
  collision->origin = visual->origin;
  collision->geometry = visual->geometry;
  link_sphere.collision.push_back(collision);

  Joint joint_sphere("joint_sphere_attached");
  joint_sphere.parent_link_name = "base_link";
  joint_sphere.child_link_name = link_sphere.getName();
  joint_sphere.type = JointType::FIXED;

  return std::make_shared<tesseract_environment::AddLinkCommand>(link_sphere, joint_sphere);
}

bool GlassUprightExample::run()
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
  using tesseract_planning::Waypoint;
  using tesseract_planning_server::ROSProcessEnvironmentCache;

  // Initial setup
  std::string urdf_xml_string, srdf_xml_string;
  nh_.getParam(ROBOT_DESCRIPTION_PARAM, urdf_xml_string);
  nh_.getParam(ROBOT_SEMANTIC_PARAM, srdf_xml_string);

  sleep(10);
  auto locator = std::make_shared<tesseract_rosutils::ROSResourceLocator>();
  if (!env_->init(urdf_xml_string, srdf_xml_string, locator))
    return false;

  // Create monitor
  monitor_ = std::make_shared<tesseract_monitoring::EnvironmentMonitor>(env_, EXAMPLE_MONITOR_NAMESPACE);
  if (rviz_)
    monitor_->startPublishingEnvironment();

  // Create plotting tool
  ROSPlottingPtr plotter = std::make_shared<tesseract_rosutils::ROSPlotting>(env_->getSceneGraph()->getRoot());
  if (rviz_)
    plotter->waitForConnection();

  // Add sphere to environment
  Command::Ptr cmd = addSphere();
  if (!monitor_->applyCommand(cmd))
    return false;

  // Set the robot initial state
  std::vector<std::string> joint_names;
  joint_names.push_back("joint_a1");
  joint_names.push_back("joint_a2");
  joint_names.push_back("joint_a3");
  joint_names.push_back("joint_a4");
  joint_names.push_back("joint_a5");
  joint_names.push_back("joint_a6");
  joint_names.push_back("joint_a7");

  Eigen::VectorXd joint_start_pos(7);
  joint_start_pos(0) = -0.4;
  joint_start_pos(1) = 0.2762;
  joint_start_pos(2) = 0.0;
  joint_start_pos(3) = -1.3348;
  joint_start_pos(4) = 0.0;
  joint_start_pos(5) = 1.4959;
  joint_start_pos(6) = 0.0;

  Eigen::VectorXd joint_end_pos(7);
  joint_end_pos(0) = 0.4;
  joint_end_pos(1) = 0.2762;
  joint_end_pos(2) = 0.0;
  joint_end_pos(3) = -1.3348;
  joint_end_pos(4) = 0.0;
  joint_end_pos(5) = 1.4959;
  joint_end_pos(6) = 0.0;

  env_->setState(joint_names, joint_start_pos);

  if (debug_)
    console_bridge::setLogLevel(console_bridge::LogLevel::CONSOLE_BRIDGE_LOG_DEBUG);

  // Solve Trajectory
  ROS_INFO("glass upright plan example");

  // Create Program
  CompositeInstruction program(
      "UPRIGHT", CompositeInstructionOrder::ORDERED, ManipulatorInfo("manipulator", "base_link", "tool0"));

  // Start and End Joint Position for the program
  Waypoint wp0 = StateWaypoint(joint_names, joint_start_pos);
  Waypoint wp1 = StateWaypoint(joint_names, joint_end_pos);

  PlanInstruction start_instruction(wp0, PlanInstructionType::START);
  program.setStartInstruction(start_instruction);

  // Plan freespace from start
  // Assign a linear motion so cartesian is defined as the target
  PlanInstruction plan_f0(wp1, PlanInstructionType::LINEAR, "UPRIGHT");
  plan_f0.setDescription("freespace_plan");

  // Add Instructions to program
  program.push_back(plan_f0);

  // Create Process Planning Server
  ProcessPlanningServer planning_server(std::make_shared<ROSProcessEnvironmentCache>(monitor_), 5);
  planning_server.loadDefaultProcessPlanners();

  // Create Process Planning Request
  ProcessPlanningRequest request;
  if (ifopt_)
  {
    auto composite_profile = std::make_shared<tesseract_planning::TrajOptIfoptDefaultCompositeProfile>();
    composite_profile->collision_cost_config->type = tesseract_collision::CollisionEvaluatorType::LVS_DISCRETE;
    composite_profile->collision_cost_config->collision_margin_data.setDefaultCollisionMargin(0.01);
    composite_profile->collision_cost_config->collision_margin_buffer = 0.01;
    composite_profile->collision_constraint_config->type = tesseract_collision::CollisionEvaluatorType::LVS_DISCRETE;
    composite_profile->collision_constraint_config->collision_margin_data.setDefaultCollisionMargin(0.01);
    composite_profile->collision_constraint_config->collision_margin_buffer = 0.01;
    composite_profile->smooth_velocities = true;
    planning_server.getProfiles()->addProfile<tesseract_planning::TrajOptIfoptCompositeProfile>("UPRIGHT",
                                                                                                composite_profile);

    auto plan_profile = std::make_shared<tesseract_planning::TrajOptIfoptDefaultPlanProfile>();
    plan_profile->joint_coeff = Eigen::VectorXd::Ones(7);
    plan_profile->cartesian_coeff = Eigen::VectorXd::Constant(6, 1, 5);
    plan_profile->cartesian_coeff(0) = 0;
    plan_profile->cartesian_coeff(1) = 0;
    plan_profile->cartesian_coeff(2) = 0;
    planning_server.getProfiles()->addProfile<tesseract_planning::TrajOptIfoptPlanProfile>("UPRIGHT", plan_profile);

    request.name = tesseract_planning::process_planner_names::TRAJOPT_IFOPT_PLANNER_NAME;
  }
  else
  {
    auto composite_profile = std::make_shared<tesseract_planning::TrajOptDefaultCompositeProfile>();
    composite_profile->collision_cost_config.enabled = true;
    composite_profile->collision_cost_config.type = trajopt::CollisionEvaluatorType::DISCRETE_CONTINUOUS;
    composite_profile->collision_cost_config.safety_margin = 0.01;
    composite_profile->collision_cost_config.coeff = 1;
    composite_profile->collision_constraint_config.enabled = true;
    composite_profile->collision_constraint_config.type = trajopt::CollisionEvaluatorType::DISCRETE_CONTINUOUS;
    composite_profile->collision_constraint_config.safety_margin = 0.01;
    composite_profile->collision_constraint_config.coeff = 1;
    composite_profile->smooth_velocities = true;
    composite_profile->smooth_accelerations = false;
    composite_profile->smooth_jerks = false;
    composite_profile->velocity_coeff = Eigen::VectorXd::Ones(1);
    planning_server.getProfiles()->addProfile<tesseract_planning::TrajOptCompositeProfile>("UPRIGHT",
                                                                                           composite_profile);

    auto plan_profile = std::make_shared<tesseract_planning::TrajOptDefaultPlanProfile>();
    plan_profile->cartesian_coeff = Eigen::VectorXd::Constant(6, 1, 5);
    plan_profile->cartesian_coeff(0) = 0;
    plan_profile->cartesian_coeff(1) = 0;
    plan_profile->cartesian_coeff(2) = 0;

    // Add profile to Dictionary
    planning_server.getProfiles()->addProfile<tesseract_planning::TrajOptPlanProfile>("UPRIGHT", plan_profile);

    request.name = tesseract_planning::process_planner_names::TRAJOPT_PLANNER_NAME;
  }
  request.instructions = Instruction(program);

  // Print Diagnostics
  request.instructions.print("Program: ");

  if (rviz_ && plotter != nullptr && plotter->isConnected())
    plotter->waitForInput("Hit Enter to solve for trajectory.");

  // Solve process plan
  ProcessPlanningFuture response = planning_server.run(request);
  planning_server.waitForAll();

  // Plot Process Trajectory
  if (rviz_ && plotter != nullptr && plotter->isConnected())
  {
    plotter->waitForInput();
    const auto& ci = response.results->as<tesseract_planning::CompositeInstruction>();
    tesseract_common::Toolpath toolpath = tesseract_planning::toToolpath(ci, *env_);
    tesseract_common::JointTrajectory trajectory = tesseract_planning::toJointTrajectory(ci);
    auto state_solver = env_->getStateSolver();

    plotter->plotMarker(ToolpathMarker(toolpath));
    plotter->plotTrajectory(trajectory, *state_solver);
  }

  ROS_INFO("Final trajectory is collision free");
  return true;
}
}  // namespace tesseract_ros_examples
