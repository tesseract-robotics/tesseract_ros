/**
 * @file car_seat_example.cpp
 * @brief Car seat example implementation
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
#include <memory>
TESSERACT_COMMON_IGNORE_WARNINGS_POP

#include <tesseract_ros_examples/car_seat_example.h>
#include <tesseract_environment/core/utils.h>
#include <tesseract_environment/core/commands.h>
#include <tesseract_rosutils/plotting.h>
#include <tesseract_rosutils/utils.h>
#include <tesseract_geometry/mesh_parser.h>
#include <tesseract_command_language/command_language.h>
#include <tesseract_command_language/profile_dictionary.h>
#include <tesseract_command_language/utils/utils.h>
#include <tesseract_process_managers/core/process_planning_server.h>
#include <tesseract_process_managers/core/default_process_planners.h>
#include <tesseract_motion_planners/trajopt/profile/trajopt_default_composite_profile.h>
#include <tesseract_motion_planners/trajopt/profile/trajopt_default_solver_profile.h>
#include <tesseract_motion_planners/core/utils.h>
#include <tesseract_planning_server/tesseract_planning_server.h>
#include <tesseract_visualization/markers/toolpath_marker.h>

using namespace tesseract_environment;
using namespace tesseract_kinematics;
using namespace tesseract_scene_graph;
using namespace tesseract_collision;
using namespace tesseract_rosutils;
using namespace tesseract_visualization;

/**@ Default ROS parameter for robot description */
const std::string ROBOT_DESCRIPTION_PARAM = "robot_description";

/**@ Default ROS parameter for robot description */
const std::string ROBOT_SEMANTIC_PARAM = "robot_description_semantic";

/** @brief RViz Example Namespace */
const std::string EXAMPLE_MONITOR_NAMESPACE = "tesseract_ros_examples";

namespace tesseract_ros_examples
{
Commands addSeats(const ResourceLocator::Ptr& locator)
{
  Commands cmds;

  for (int i = 0; i < 3; ++i)
  {
    Link link_seat("seat_" + std::to_string(i + 1));

    Visual::Ptr visual = std::make_shared<Visual>();
    visual->origin = Eigen::Isometry3d::Identity();
    visual->geometry =
        tesseract_geometry::createMeshFromResource<tesseract_geometry::Mesh>(locator->locateResource("package://"
                                                                                                     "tesseract_ros_"
                                                                                                     "examples/meshes/"
                                                                                                     "car_seat/visual/"
                                                                                                     "seat.dae"),
                                                                             Eigen::Vector3d(1, 1, 1),
                                                                             true)[0];
    link_seat.visual.push_back(visual);

    for (int m = 1; m <= 10; ++m)
    {
      std::vector<tesseract_geometry::Mesh::Ptr> meshes =
          tesseract_geometry::createMeshFromResource<tesseract_geometry::Mesh>(
              locator->locateResource("package://tesseract_ros_examples/"
                                      "meshes/car_seat/collision/seat_" +
                                      std::to_string(m) + ".stl"));
      for (auto& mesh : meshes)
      {
        Collision::Ptr collision = std::make_shared<Collision>();
        collision->origin = visual->origin;
        collision->geometry = makeConvexMesh(*mesh);
        link_seat.collision.push_back(collision);
      }
    }

    Joint joint_seat("joint_seat_" + std::to_string(i + 1));
    joint_seat.parent_link_name = "world";
    joint_seat.child_link_name = link_seat.getName();
    joint_seat.type = JointType::FIXED;
    joint_seat.parent_to_joint_origin_transform = Eigen::AngleAxisd(0.0, Eigen::Vector3d::UnitX()) *
                                                  Eigen::AngleAxisd(0.0, Eigen::Vector3d::UnitY()) *
                                                  Eigen::AngleAxisd(3.14159, Eigen::Vector3d::UnitZ());
    joint_seat.parent_to_joint_origin_transform.translation() = Eigen::Vector3d(0.5 + i, 2.15, 0.45);

    cmds.push_back(std::make_shared<tesseract_environment::AddLinkCommand>(link_seat, joint_seat));
  }
  return cmds;
}

CarSeatExample::CarSeatExample(const ros::NodeHandle& nh, bool plotting, bool rviz) : Example(plotting, rviz), nh_(nh)
{
}

std::unordered_map<std::string, std::unordered_map<std::string, double>> CarSeatExample::getPredefinedPosition()
{
  std::unordered_map<std::string, std::unordered_map<std::string, double>> result;

  std::unordered_map<std::string, double> default_pos;
  default_pos["carriage_rail"] = 1.0;
  default_pos["joint_b"] = 0.0;
  default_pos["joint_e"] = 0.0;
  default_pos["joint_l"] = 0.0;
  default_pos["joint_r"] = 0.0;
  default_pos["joint_s"] = -1.5707;
  default_pos["joint_t"] = 0.0;
  default_pos["joint_u"] = -1.5707;
  result["Default"] = default_pos;

  std::unordered_map<std::string, double> pick1;
  pick1["carriage_rail"] = 2.22;
  pick1["joint_b"] = 0.39;
  pick1["joint_e"] = 0.0;
  pick1["joint_l"] = 0.5;
  pick1["joint_r"] = 0.0;
  pick1["joint_s"] = -3.14;
  pick1["joint_t"] = -0.29;
  pick1["joint_u"] = -1.45;
  result["Pick1"] = pick1;

  std::unordered_map<std::string, double> pick2;
  pick2["carriage_rail"] = 1.22;
  pick1["joint_b"] = 0.39;
  pick1["joint_e"] = 0.0;
  pick1["joint_l"] = 0.5;
  pick1["joint_r"] = 0.0;
  pick1["joint_s"] = -3.14;
  pick1["joint_t"] = -0.29;
  pick1["joint_u"] = -1.45;
  result["Pick2"] = pick2;

  std::unordered_map<std::string, double> pick3;
  pick3["carriage_rail"] = 0.22;
  pick1["joint_b"] = 0.39;
  pick1["joint_e"] = 0.0;
  pick1["joint_l"] = 0.5;
  pick1["joint_r"] = 0.0;
  pick1["joint_s"] = -3.14;
  pick1["joint_t"] = -0.29;
  pick1["joint_u"] = -1.45;
  result["Pick3"] = pick3;

  std::unordered_map<std::string, double> place1;
  place1["carriage_rail"] = 4.15466;
  place1["joint_b"] = 0.537218;
  place1["joint_e"] = 0.0189056;
  place1["joint_l"] = 0.801223;
  place1["joint_r"] = 0.0580309;
  place1["joint_s"] = -0.0481182;
  place1["joint_t"] = -0.325783;
  place1["joint_u"] = -1.2813;
  result["Place1"] = place1;

  std::unordered_map<std::string, double> home;
  home["carriage_rail"] = 0.0;
  home["joint_b"] = 0.0;
  home["joint_e"] = 0.0;
  home["joint_l"] = 0.0;
  home["joint_r"] = 0.0;
  home["joint_s"] = 0.0;
  home["joint_t"] = 0.0;
  home["joint_u"] = 0.0;
  result["Home"] = home;

  return result;
}

std::vector<double> CarSeatExample::getPositionVector(const ForwardKinematics::ConstPtr& kin,
                                                      const std::unordered_map<std::string, double>& pos)
{
  std::vector<double> result;
  for (const auto& joint_name : kin->getJointNames())
    result.push_back(pos.at(joint_name));

  return result;
}

Eigen::VectorXd CarSeatExample::getPositionVectorXd(const ForwardKinematics::ConstPtr& kin,
                                                    const std::unordered_map<std::string, double>& pos)
{
  Eigen::VectorXd result;
  result.resize(kin->numJoints());
  int cnt = 0;
  for (const auto& joint_name : kin->getJointNames())
    result[cnt++] = pos.at(joint_name);

  return result;
}

bool CarSeatExample::run()
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
    monitor_->startPublishingEnvironment();

  // Get manipulator
  ForwardKinematics::Ptr fwd_kin;
  {  // Need to lock monitor for read
    auto lock = monitor_->lockEnvironmentRead();
    fwd_kin = monitor_->getEnvironment()->getManipulatorManager()->getFwdKinematicSolver("manipulator");
  }

  // Create seats and add it to the local environment
  Commands cmds = addSeats(locator);
  if (!monitor_->applyCommands(cmds))
    return false;

  // Create plotting tool
  ROSPlottingPtr plotter = std::make_shared<ROSPlotting>(monitor_->getSceneGraph()->getRoot());
  if (rviz_)
    plotter->waitForConnection();

  if (rviz_ && plotter != nullptr && plotter->isConnected())
    plotter->waitForInput();

  // Get predefined positions
  saved_positions_ = getPredefinedPosition();

  // Move to home position
  env_->setState(saved_positions_["Home"]);

  // Create Process Planning Server
  ProcessPlanningServer planning_server(std::make_shared<ROSProcessEnvironmentCache>(monitor_), 5);
  planning_server.loadDefaultProcessPlanners();

  // Create TrajOpt Profile
  auto trajopt_composite_profile = std::make_shared<tesseract_planning::TrajOptDefaultCompositeProfile>();
  trajopt_composite_profile->collision_constraint_config.enabled = false;
  trajopt_composite_profile->collision_cost_config.safety_margin = 0.005;
  trajopt_composite_profile->collision_cost_config.coeff = 50;

  auto trajopt_solver_profile = std::make_shared<tesseract_planning::TrajOptDefaultSolverProfile>();
  trajopt_solver_profile->opt_info.max_iter = 200;
  trajopt_solver_profile->opt_info.min_approx_improve = 1e-3;
  trajopt_solver_profile->opt_info.min_trust_box_size = 1e-3;

  // Add profile to Dictionary
  planning_server.getProfiles()->addProfile<tesseract_planning::TrajOptCompositeProfile>("FREESPACE",
                                                                                         trajopt_composite_profile);
  planning_server.getProfiles()->addProfile<tesseract_planning::TrajOptSolverProfile>("FREESPACE",
                                                                                      trajopt_solver_profile);

  // Solve Trajectory
  ROS_INFO("Car Seat Demo Started");

  {  // Create Program to pick up first seat
    CompositeInstruction program("FREESPACE", CompositeInstructionOrder::ORDERED, ManipulatorInfo("manipulator"));
    program.setDescription("Pick up the first seat!");

    // Start and End Joint Position for the program
    Eigen::VectorXd start_pos = getPositionVectorXd(fwd_kin, saved_positions_["Home"]);
    Eigen::VectorXd pick_pose = getPositionVectorXd(fwd_kin, saved_positions_["Pick1"]);
    Waypoint wp0 = StateWaypoint(fwd_kin->getJointNames(), start_pos);
    Waypoint wp1 = StateWaypoint(fwd_kin->getJointNames(), pick_pose);

    // Start Joint Position for the program
    PlanInstruction start_instruction(wp0, PlanInstructionType::START);
    program.setStartInstruction(start_instruction);

    // Plan freespace from start
    PlanInstruction plan_f0(wp1, PlanInstructionType::FREESPACE, "FREESPACE");
    plan_f0.setDescription("Freespace pick seat 1");

    // Add Instructions to program
    program.push_back(plan_f0);

    ROS_INFO("Freespace plan to pick seat 1 example");

    // Create Process Planning Request
    ProcessPlanningRequest request;
    request.name = tesseract_planning::process_planner_names::TRAJOPT_PLANNER_NAME;
    request.instructions = Instruction(program);

    // Print Diagnostics
    request.instructions.print("Program: ");

    // Solve process plan
    ProcessPlanningFuture response = planning_server.run(request);
    planning_server.waitForAll();

    // Plot Process Trajectory
    if (rviz_ && plotter != nullptr && plotter->isConnected())
    {
      const auto& ci = response.results->as<tesseract_planning::CompositeInstruction>();
      tesseract_common::Toolpath toolpath = tesseract_planning::toToolpath(ci, env_);
      tesseract_common::JointTrajectory trajectory = tesseract_planning::toJointTrajectory(ci);
      plotter->plotMarker(ToolpathMarker(toolpath));
      plotter->plotTrajectory(trajectory, env_->getStateSolver());
      plotter->waitForInput();
    }
  }

  // Get the state at the end of pick 1 trajectory
  EnvState::Ptr state = env_->getState(saved_positions_["Pick1"]);

  // Now we to detach seat_1 and attach it to the robot end_effector
  Joint joint_seat_1_robot("joint_seat_1_robot");
  joint_seat_1_robot.parent_link_name = "end_effector";
  joint_seat_1_robot.child_link_name = "seat_1";
  joint_seat_1_robot.type = JointType::FIXED;
  joint_seat_1_robot.parent_to_joint_origin_transform =
      state->link_transforms["end_effector"].inverse() * state->link_transforms["seat_1"];

  cmds.clear();
  cmds.push_back(std::make_shared<MoveLinkCommand>(joint_seat_1_robot));
  cmds.push_back(std::make_shared<AddAllowedCollisionCommand>("seat_1", "end_effector", "Adjacent"));
  cmds.push_back(std::make_shared<AddAllowedCollisionCommand>("seat_1", "cell_logo", "Never"));
  cmds.push_back(std::make_shared<AddAllowedCollisionCommand>("seat_1", "fence", "Never"));
  cmds.push_back(std::make_shared<AddAllowedCollisionCommand>("seat_1", "link_b", "Never"));
  cmds.push_back(std::make_shared<AddAllowedCollisionCommand>("seat_1", "link_r", "Never"));
  cmds.push_back(std::make_shared<AddAllowedCollisionCommand>("seat_1", "link_t", "Never"));

  // Apply the commands to the environment
  if (!monitor_->applyCommands(cmds))
    return false;

  {  // Create Program to place first seat
    CompositeInstruction program("FREESPACE", CompositeInstructionOrder::ORDERED, ManipulatorInfo("manipulator"));
    program.setDescription("Place the first seat!");

    // Start and End Joint Position for the program
    Eigen::VectorXd start_pos = getPositionVectorXd(fwd_kin, saved_positions_["Pick1"]);
    Eigen::VectorXd pick_pose = getPositionVectorXd(fwd_kin, saved_positions_["Place1"]);
    Waypoint wp0 = StateWaypoint(fwd_kin->getJointNames(), start_pos);
    Waypoint wp1 = StateWaypoint(fwd_kin->getJointNames(), pick_pose);

    // Start Joint Position for the program
    PlanInstruction start_instruction(wp0, PlanInstructionType::START);
    program.setStartInstruction(start_instruction);

    // Plan freespace from start
    PlanInstruction plan_f0(wp1, PlanInstructionType::FREESPACE, "FREESPACE");
    plan_f0.setDescription("Freespace pick seat 1");

    // Add Instructions to program
    program.push_back(plan_f0);

    ROS_INFO("Freespace plan to pick seat 1 example");

    // Create Process Planning Request
    ProcessPlanningRequest request;
    request.name = tesseract_planning::process_planner_names::TRAJOPT_PLANNER_NAME;
    request.instructions = Instruction(program);

    // Print Diagnostics
    request.instructions.print("Program: ");

    // Solve process plan
    ProcessPlanningFuture response = planning_server.run(request);
    planning_server.waitForAll();

    // Plot Process Trajectory
    if (rviz_ && plotter != nullptr && plotter->isConnected())
    {
      const auto& ci = response.results->as<tesseract_planning::CompositeInstruction>();
      tesseract_common::Toolpath toolpath = tesseract_planning::toToolpath(ci, env_);
      tesseract_common::JointTrajectory trajectory = tesseract_planning::toJointTrajectory(ci);
      plotter->plotMarker(ToolpathMarker(toolpath));
      plotter->plotTrajectory(trajectory, env_->getStateSolver());
      plotter->waitForInput();
    }
  }

  return true;
}

}  // namespace tesseract_ros_examples
