/**
 * @file pick_and_place_example.cpp
 * @brief Pick and place implementation
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

#include <tesseract_ros_examples/pick_and_place_example.h>
#include <tesseract_rosutils/plotting.h>
#include <tesseract_rosutils/utils.h>
#include <tesseract_motion_planners/trajopt/profile/trajopt_default_plan_profile.h>
#include <tesseract_motion_planners/trajopt/profile/trajopt_default_composite_profile.h>
#include <tesseract_motion_planners/trajopt/profile/trajopt_default_solver_profile.h>
#include <tesseract_motion_planners/core/utils.h>
#include <tesseract_command_language/command_language.h>
#include <tesseract_command_language/utils/utils.h>
#include <tesseract_command_language/utils/get_instruction_utils.h>
#include <tesseract_process_managers/taskflow_generators/trajopt_taskflow.h>
#include <tesseract_planning_server/tesseract_planning_server.h>
#include <tesseract_visualization/markers/toolpath_marker.h>

using namespace tesseract_environment;
using namespace tesseract_kinematics;
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

const double OFFSET = 0.005;

const std::string LINK_BOX_NAME = "box";
const std::string LINK_BASE_NAME = "world";
const std::string LINK_END_EFFECTOR_NAME = "iiwa_link_ee";

namespace tesseract_ros_examples
{
PickAndPlaceExample::PickAndPlaceExample(const ros::NodeHandle& nh, bool plotting, bool rviz)
  : Example(plotting, rviz), nh_(nh)
{
}

Command::Ptr PickAndPlaceExample::addBox(double box_x, double box_y, double box_side)
{
  Link link_box(LINK_BOX_NAME);

  Visual::Ptr visual = std::make_shared<Visual>();
  visual->origin = Eigen::Isometry3d::Identity();
  visual->geometry = std::make_shared<tesseract_geometry::Box>(box_side, box_side, box_side);
  link_box.visual.push_back(visual);

  Collision::Ptr collision = std::make_shared<Collision>();
  collision->origin = visual->origin;
  collision->geometry = visual->geometry;
  link_box.collision.push_back(collision);

  Joint joint_box("joint_box");
  joint_box.parent_link_name = "workcell_base";
  joint_box.child_link_name = LINK_BOX_NAME;
  joint_box.type = JointType::FIXED;
  joint_box.parent_to_joint_origin_transform = Eigen::Isometry3d::Identity();
  joint_box.parent_to_joint_origin_transform.translation() += Eigen::Vector3d(box_x, box_y, (box_side / 2.0) + OFFSET);

  return std::make_shared<tesseract_environment::AddLinkCommand>(link_box, joint_box);
}

bool PickAndPlaceExample::run()
{
  using tesseract_planning::CartesianWaypoint;
  using tesseract_planning::CompositeInstruction;
  using tesseract_planning::CompositeInstructionOrder;
  using tesseract_planning::Instruction;
  using tesseract_planning::ManipulatorInfo;
  using tesseract_planning::MoveInstruction;
  using tesseract_planning::PlanInstruction;
  using tesseract_planning::PlanInstructionType;
  using tesseract_planning::ProcessPlanningFuture;
  using tesseract_planning::ProcessPlanningRequest;
  using tesseract_planning::ProcessPlanningServer;
  using tesseract_planning::StateWaypoint;
  using tesseract_planning::Waypoint;
  using tesseract_planning_server::ROSProcessEnvironmentCache;

  /////////////
  /// SETUP ///
  /////////////

  console_bridge::setLogLevel(console_bridge::LogLevel::CONSOLE_BRIDGE_LOG_DEBUG);

  // Pull ROS params
  std::string urdf_xml_string, srdf_xml_string;
  double box_side, box_x, box_y;
  nh_.getParam(ROBOT_DESCRIPTION_PARAM, urdf_xml_string);
  nh_.getParam(ROBOT_SEMANTIC_PARAM, srdf_xml_string);
  nh_.getParam("box_side", box_side);
  nh_.getParam("box_x", box_x);
  nh_.getParam("box_y", box_y);

  // Initialize the environment
  auto locator = std::make_shared<tesseract_rosutils::ROSResourceLocator>();
  if (!env_->init(urdf_xml_string, srdf_xml_string, locator))
    return false;

  // Create monitor
  monitor_ = std::make_shared<tesseract_monitoring::EnvironmentMonitor>(env_, EXAMPLE_MONITOR_NAMESPACE);
  if (rviz_)
    monitor_->startPublishingEnvironment();

  // Set default contact distance
  Command::Ptr cmd_default_dist = std::make_shared<tesseract_environment::ChangeCollisionMarginsCommand>(0.005);
  if (!monitor_->applyCommand(cmd_default_dist))
    return false;

  // Create plotting tool
  ROSPlottingPtr plotter = std::make_shared<tesseract_rosutils::ROSPlotting>(env_->getSceneGraph()->getRoot());
  if (rviz_)
    plotter->waitForConnection();

  // Set the robot initial state
  std::vector<std::string> joint_names;
  joint_names.push_back("iiwa_joint_1");
  joint_names.push_back("iiwa_joint_2");
  joint_names.push_back("iiwa_joint_3");
  joint_names.push_back("iiwa_joint_4");
  joint_names.push_back("iiwa_joint_5");
  joint_names.push_back("iiwa_joint_6");
  joint_names.push_back("iiwa_joint_7");

  Eigen::VectorXd joint_pos(7);
  joint_pos(0) = 0.0;
  joint_pos(1) = 0.0;
  joint_pos(2) = 0.0;
  joint_pos(3) = -1.57;
  joint_pos(4) = 0.0;
  joint_pos(5) = 0.0;
  joint_pos(6) = 0.0;

  env_->setState(joint_names, joint_pos);

  // Add simulated box to environment
  Command::Ptr cmd = addBox(box_x, box_y, box_side);
  if (!monitor_->applyCommand(cmd))
    return false;

  ////////////
  /// PICK ///
  ////////////
  if (rviz_)
    plotter->waitForInput();

  // Create Program
  CompositeInstruction pick_program("DEFAULT",
                                    CompositeInstructionOrder::ORDERED,
                                    ManipulatorInfo("manipulator", LINK_BASE_NAME, LINK_END_EFFECTOR_NAME));

  Waypoint pick_swp = StateWaypoint(joint_names, joint_pos);
  PlanInstruction start_instruction(pick_swp, PlanInstructionType::START);
  pick_program.setStartInstruction(start_instruction);

  // Define the final pose (on top of the box)
  Eigen::Isometry3d pick_final_pose;
  pick_final_pose.linear() = Eigen::Quaterniond(0.0, 0.0, 1.0, 0.0).matrix();
  pick_final_pose.translation() = Eigen::Vector3d(box_x, box_y, box_side + 0.77153 + OFFSET);  // Offset for the table
  Waypoint pick_wp1 = CartesianWaypoint(pick_final_pose);

  // Define the approach pose
  Eigen::Isometry3d pick_approach_pose = pick_final_pose;
  pick_approach_pose.translation() += Eigen::Vector3d(0.0, 0.0, 0.15);
  Waypoint pick_wp0 = CartesianWaypoint(pick_approach_pose);

  // Plan freespace from start
  PlanInstruction pick_plan_a0(pick_wp0, PlanInstructionType::FREESPACE, "FREESPACE");
  pick_plan_a0.setDescription("From start to pick Approach");

  // Plan cartesian approach
  PlanInstruction pick_plan_a1(pick_wp1, PlanInstructionType::LINEAR, "CARTESIAN");
  pick_plan_a1.setDescription("Pick Approach");

  // Add Instructions to program
  pick_program.push_back(pick_plan_a0);
  pick_program.push_back(pick_plan_a1);

  // Create Process Planning Server
  ProcessPlanningServer planning_server(std::make_shared<ROSProcessEnvironmentCache>(monitor_), 5);
  planning_server.loadDefaultProcessPlanners();

  // Create TrajOpt Profile
  auto trajopt_plan_profile = std::make_shared<tesseract_planning::TrajOptDefaultPlanProfile>();
  trajopt_plan_profile->cartesian_coeff = Eigen::VectorXd::Constant(6, 1, 10);

  auto trajopt_composite_profile = std::make_shared<tesseract_planning::TrajOptDefaultCompositeProfile>();
  trajopt_composite_profile->collision_constraint_config.enabled = false;
  trajopt_composite_profile->collision_cost_config.safety_margin = 0.005;
  trajopt_composite_profile->collision_cost_config.coeff = 50;

  auto trajopt_solver_profile = std::make_shared<tesseract_planning::TrajOptDefaultSolverProfile>();
  trajopt_solver_profile->opt_info.max_iter = 100;

  // Add profile to Dictionary
  planning_server.getProfiles()->addProfile<tesseract_planning::TrajOptPlanProfile>("CARTESIAN", trajopt_plan_profile);
  planning_server.getProfiles()->addProfile<tesseract_planning::TrajOptCompositeProfile>("DEFAULT",
                                                                                         trajopt_composite_profile);
  planning_server.getProfiles()->addProfile<tesseract_planning::TrajOptSolverProfile>("DEFAULT",
                                                                                      trajopt_solver_profile);

  ROS_INFO("Pick plan");
  // Create Process Planning Request
  ProcessPlanningRequest pick_request;
  pick_request.name = tesseract_planning::process_planner_names::TRAJOPT_PLANNER_NAME;
  pick_request.instructions = Instruction(pick_program);

  // Print Diagnostics
  pick_request.instructions.print("Program: ");

  // Solve process plan
  ProcessPlanningFuture pick_response = planning_server.run(pick_request);
  planning_server.waitForAll();

  // Plot Process Trajectory
  if (rviz_ && plotter != nullptr && plotter->isConnected())
  {
    plotter->waitForInput();
    const auto& cp = pick_response.results->as<CompositeInstruction>();
    tesseract_common::Toolpath toolpath = tesseract_planning::toToolpath(cp, *env_);
    tesseract_common::JointTrajectory trajectory = tesseract_planning::toJointTrajectory(cp);
    auto state_solver = env_->getStateSolver();
    plotter->plotMarker(ToolpathMarker(toolpath));
    plotter->plotTrajectory(trajectory, *state_solver);
  }

  /////////////
  /// PLACE ///
  /////////////

  if (rviz_)
    plotter->waitForInput();

  // Detach the simulated box from the world and attach to the end effector
  tesseract_environment::Commands cmds;
  Joint joint_box2("joint_box2");
  joint_box2.parent_link_name = LINK_END_EFFECTOR_NAME;
  joint_box2.child_link_name = LINK_BOX_NAME;
  joint_box2.type = JointType::FIXED;
  joint_box2.parent_to_joint_origin_transform = Eigen::Isometry3d::Identity();
  joint_box2.parent_to_joint_origin_transform.translation() += Eigen::Vector3d(0, 0, box_side / 2.0);
  cmds.push_back(std::make_shared<tesseract_environment::MoveLinkCommand>(joint_box2));
  cmds.push_back(
      std::make_shared<tesseract_environment::AddAllowedCollisionCommand>(LINK_BOX_NAME, "iiwa_link_ee", "Never"));
  cmds.push_back(
      std::make_shared<tesseract_environment::AddAllowedCollisionCommand>(LINK_BOX_NAME, "iiwa_link_7", "Never"));
  cmds.push_back(
      std::make_shared<tesseract_environment::AddAllowedCollisionCommand>(LINK_BOX_NAME, "iiwa_link_6", "Never"));
  monitor_->applyCommands(cmds);

  // Get the last move instruction
  const CompositeInstruction& pick_composite = pick_response.results->as<CompositeInstruction>();
  const MoveInstruction* pick_final_state = tesseract_planning::getLastMoveInstruction(pick_composite);

  // Retreat to the approach pose
  Eigen::Isometry3d retreat_pose = pick_approach_pose;

  // Define some place locations.
  Eigen::Isometry3d bottom_right_shelf, bottom_left_shelf, middle_right_shelf, middle_left_shelf, top_right_shelf,
      top_left_shelf;
  bottom_right_shelf.linear() = Eigen::Quaterniond(0, 0, 0.7071068, 0.7071068).matrix();
  bottom_right_shelf.translation() = Eigen::Vector3d(0.148856, 0.73085, 0.906);
  bottom_left_shelf.linear() = Eigen::Quaterniond(0, 0, 0.7071068, 0.7071068).matrix();
  bottom_left_shelf.translation() = Eigen::Vector3d(-0.148856, 0.73085, 0.906);
  middle_right_shelf.linear() = Eigen::Quaterniond(0, 0, 0.7071068, 0.7071068).matrix();
  middle_right_shelf.translation() = Eigen::Vector3d(0.148856, 0.73085, 1.16);
  middle_left_shelf.linear() = Eigen::Quaterniond(0, 0, 0.7071068, 0.7071068).matrix();
  middle_left_shelf.translation() = Eigen::Vector3d(-0.148856, 0.73085, 1.16);
  top_right_shelf.linear() = Eigen::Quaterniond(0, 0, 0.7071068, 0.7071068).matrix();
  top_right_shelf.translation() = Eigen::Vector3d(0.148856, 0.73085, 1.414);
  top_left_shelf.linear() = Eigen::Quaterniond(0, 0, 0.7071068, 0.7071068).matrix();
  top_left_shelf.translation() = Eigen::Vector3d(-0.148856, 0.73085, 1.414);

  // Set the target pose to middle_left_shelf
  Eigen::Isometry3d place_pose = middle_left_shelf;

  // Setup approach for place move
  Eigen::Isometry3d place_approach_pose = place_pose;
  place_approach_pose.translation() += Eigen::Vector3d(0.0, -0.25, 0);

  // Create Program
  CompositeInstruction place_program("DEFAULT",
                                     CompositeInstructionOrder::ORDERED,
                                     ManipulatorInfo("manipulator", LINK_BASE_NAME, LINK_END_EFFECTOR_NAME));

  PlanInstruction place_start_instruction(pick_final_state->getWaypoint(), PlanInstructionType::START);
  place_program.setStartInstruction(place_start_instruction);

  // Define the approach pose
  Waypoint place_wp0 = CartesianWaypoint(retreat_pose);

  // Define the final pose approach
  Waypoint place_wp1 = CartesianWaypoint(place_approach_pose);

  // Define the final pose
  Waypoint place_wp2 = CartesianWaypoint(place_pose);

  // Plan cartesian retraction from picking up the box
  PlanInstruction place_plan_a0(place_wp0, PlanInstructionType::LINEAR, "CARTESIAN");
  place_plan_a0.setDescription("Place retraction");

  // Plan freespace to approach for box drop off
  PlanInstruction place_plan_a1(place_wp1, PlanInstructionType::FREESPACE, "FREESPACE");
  place_plan_a1.setDescription("Place Freespace");

  // Plan cartesian approach to box drop location
  PlanInstruction place_plan_a2(place_wp2, PlanInstructionType::LINEAR, "CARTESIAN");
  place_plan_a2.setDescription("Place approach");

  // Add Instructions to program
  place_program.push_back(place_plan_a0);
  place_program.push_back(place_plan_a1);
  place_program.push_back(place_plan_a2);

  // Create Process Planning Request
  ProcessPlanningRequest place_request;
  place_request.name = tesseract_planning::process_planner_names::TRAJOPT_PLANNER_NAME;
  place_request.instructions = Instruction(place_program);

  // Print Diagnostics
  place_request.instructions.print("Program: ");

  // Solve process plan
  ProcessPlanningFuture place_response = planning_server.run(place_request);
  planning_server.waitForAll();

  // Plot Process Trajectory
  if (rviz_ && plotter != nullptr && plotter->isConnected())
  {
    plotter->waitForInput();
    const auto& ci = place_response.results->as<tesseract_planning::CompositeInstruction>();
    tesseract_common::Toolpath toolpath = tesseract_planning::toToolpath(ci, *env_);
    tesseract_common::JointTrajectory trajectory = tesseract_planning::toJointTrajectory(ci);
    auto state_solver = env_->getStateSolver();
    plotter->plotMarker(ToolpathMarker(toolpath));
    plotter->plotTrajectory(trajectory, *state_solver);
  }

  if (rviz_)
    plotter->waitForInput();

  ROS_INFO("Done");
  return true;
}
}  // namespace tesseract_ros_examples
