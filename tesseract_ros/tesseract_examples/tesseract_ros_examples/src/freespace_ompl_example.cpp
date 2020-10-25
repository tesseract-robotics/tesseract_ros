/**
 * @file freespace_ompl_example.cpp
 * @brief An example of a feespace motion planning with OMPL.
 *
 * @author Levi Armstrong
 * @date March 16, 2020
 * @version TODO
 * @bug No known bugs
 *
 * @copyright Copyright (c) 2020, Southwest Research Institute
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
TESSERACT_COMMON_IGNORE_WARNINGS_POP

#include <tesseract_ros_examples/freespace_ompl_example.h>
#include <tesseract_environment/core/utils.h>
#include <tesseract_environment/core/commands.h>
#include <tesseract_rosutils/plotting.h>
#include <tesseract_rosutils/utils.h>
#include <tesseract_command_language/command_language.h>
#include <tesseract_command_language/utils/utils.h>
#include <tesseract_process_managers/process_input.h>
#include <tesseract_process_managers/taskflows/ompl_taskflow.h>
#include <tesseract_process_managers/process_managers/simple_process_manager.h>
#include <tesseract_motion_planners/ompl/profile/ompl_default_plan_profile.h>

using namespace tesseract;
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
FreespaceOMPLExample::FreespaceOMPLExample(const ros::NodeHandle& nh,
                                           bool plotting,
                                           bool rviz,
                                           double range,
                                           double planning_time)
  : Example(plotting, rviz), nh_(nh), range_(range), planning_time_(planning_time)
{
}

Command::Ptr FreespaceOMPLExample::addSphere()
{
  // Add sphere to environment
  auto link_sphere = std::make_shared<Link>("sphere_attached");

  Visual::Ptr visual = std::make_shared<Visual>();
  visual->origin = Eigen::Isometry3d::Identity();
  visual->origin.translation() = Eigen::Vector3d(0.5, 0, 0.55);
  visual->geometry = std::make_shared<tesseract_geometry::Sphere>(0.15);
  link_sphere->visual.push_back(visual);

  Collision::Ptr collision = std::make_shared<Collision>();
  collision->origin = visual->origin;
  collision->geometry = visual->geometry;
  link_sphere->collision.push_back(collision);

  auto joint_sphere = std::make_shared<Joint>("joint_sphere_attached");
  joint_sphere->parent_link_name = "base_link";
  joint_sphere->child_link_name = link_sphere->getName();
  joint_sphere->type = JointType::FIXED;

  return std::make_shared<tesseract_environment::AddCommand>(link_sphere, joint_sphere);
}

bool FreespaceOMPLExample::run()
{
  using tesseract_planning::CartesianWaypoint;
  using tesseract_planning::CompositeInstruction;
  using tesseract_planning::CompositeInstructionOrder;
  using tesseract_planning::Instruction;
  using tesseract_planning::ManipulatorInfo;
  using tesseract_planning::PlanInstruction;
  using tesseract_planning::PlanInstructionType;
  using tesseract_planning::StateWaypoint;
  using tesseract_planning::Waypoint;

  // Initial setup
  std::string urdf_xml_string, srdf_xml_string;
  nh_.getParam(ROBOT_DESCRIPTION_PARAM, urdf_xml_string);
  nh_.getParam(ROBOT_SEMANTIC_PARAM, srdf_xml_string);

  ResourceLocator::Ptr locator = std::make_shared<tesseract_rosutils::ROSResourceLocator>();
  if (!tesseract_->init(urdf_xml_string, srdf_xml_string, locator))
    return false;

  // Create monitor
  monitor_ = std::make_shared<tesseract_monitoring::EnvironmentMonitor>(tesseract_, EXAMPLE_MONITOR_NAMESPACE);
  if (rviz_)
    monitor_->startPublishingEnvironment(tesseract_monitoring::EnvironmentMonitor::UPDATE_ENVIRONMENT);

  // Add sphere to environment
  Command::Ptr cmd = addSphere();
  if (!monitor_->applyCommand(*cmd))
    return false;

  // Create plotting tool
  ROSPlottingPtr plotter =
      std::make_shared<tesseract_rosutils::ROSPlotting>(tesseract_->getEnvironment()->getSceneGraph()->getRoot());
  plotter->init(tesseract_);

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

  tesseract_->getEnvironment()->setState(joint_names, joint_start_pos);

  // Create Program
  CompositeInstruction program("FREESPACE", CompositeInstructionOrder::ORDERED, ManipulatorInfo("manipulator"));

  // Start and End Joint Position for the program
  Waypoint wp0 = StateWaypoint(joint_names, joint_start_pos);
  Waypoint wp1 = StateWaypoint(joint_names, joint_end_pos);

  PlanInstruction start_instruction(wp0, PlanInstructionType::START);
  program.setStartInstruction(start_instruction);

  // Plan freespace from start
  PlanInstruction plan_f0(wp1, PlanInstructionType::FREESPACE, "FREESPACE");
  plan_f0.setDescription("freespace_plan");

  // Add Instructions to program
  program.push_back(plan_f0);

  // Create seed data structure
  const Instruction program_instruction{ program };
  Instruction seed = tesseract_planning::generateSkeletonSeed(program);

  // Define the Process Input
  tesseract_planning::ProcessInput input(tesseract_, &program_instruction, program.getManipulatorInfo(), &seed);

  // Create OMPL Profile
  auto ompl_profile = std::make_shared<tesseract_planning::OMPLDefaultPlanProfile>();
  auto ompl_planner_config = std::make_shared<tesseract_planning::RRTConnectConfigurator>();
  ompl_planner_config->range = range_;
  ompl_profile->planning_time = planning_time_;
  ompl_profile->planners = { ompl_planner_config, ompl_planner_config };

  // Initialize Process Manager
  ROS_INFO("Freespace plan example");
  tesseract_planning::OMPLTaskflowParams params;
  params.ompl_plan_profiles["FREESPACE"] = ompl_profile;
  tesseract_planning::GraphTaskflow::UPtr ompl_taskflow = tesseract_planning::createOMPLTaskflow(params);
  tesseract_planning::SimpleProcessManager pm(std::move(ompl_taskflow));

  if (!pm.init(input))
  {
    ROS_ERROR("Initialization Failed");
    return false;
  }

  // Solve
  if (!pm.execute())
  {
    ROS_ERROR("Execution Failed");
    return false;
  }

  // Plot Trajectory
  if (plotter)
  {
    plotter->waitForInput();
    plotter->plotToolPath(*(input.getResults()));
    plotter->plotTrajectory(*(input.getResults()));
  }

  ROS_INFO("Final trajectory is collision free");
  return true;
}
}  // namespace tesseract_ros_examples
