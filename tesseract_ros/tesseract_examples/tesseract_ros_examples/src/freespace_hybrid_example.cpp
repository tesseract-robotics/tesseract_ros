/**
 * @file freespace_hybrid_example.cpp
 * @brief An example of a feespace motion planning with OMPL then TrajOpt.
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

#include <tesseract_ros_examples/freespace_hybrid_example.h>
#include <tesseract_environment/core/utils.h>
#include <tesseract_rosutils/plotting.h>
#include <tesseract_rosutils/utils.h>
#include <tesseract_motion_planners/ompl/config/ompl_planner_freespace_config.h>
#include <tesseract_motion_planners/hybrid/ompl_trajopt_freespace_planner.h>
#include <tesseract_motion_planners/ompl/ompl_motion_planner.h>
#include <tesseract_motion_planners/trajopt/trajopt_motion_planner.h>

using namespace tesseract;
using namespace tesseract_environment;
using namespace tesseract_scene_graph;
using namespace tesseract_collision;
using namespace tesseract_rosutils;

const std::string ROBOT_DESCRIPTION_PARAM = "robot_description"; /**< Default ROS parameter for robot description */
const std::string ROBOT_SEMANTIC_PARAM = "robot_description_semantic"; /**< Default ROS parameter for robot
                                                                          description */
const std::string GET_ENVIRONMENT_CHANGES_SERVICE = "get_tesseract_changes_rviz";
const std::string MODIFY_ENVIRONMENT_SERVICE = "modify_tesseract_rviz";

namespace tesseract_ros_examples
{
FreespaceHybridExample::FreespaceHybridExample(const ros::NodeHandle& nh,
                                               bool plotting,
                                               bool rviz,
                                               double range,
                                               double planning_time)
  : Example(plotting, rviz), nh_(nh), range_(range), planning_time_(planning_time)
{
}

bool FreespaceHybridExample::run()
{
  // Initial setup
  std::string urdf_xml_string, srdf_xml_string;
  nh_.getParam(ROBOT_DESCRIPTION_PARAM, urdf_xml_string);
  nh_.getParam(ROBOT_SEMANTIC_PARAM, srdf_xml_string);

  ResourceLocator::Ptr locator = std::make_shared<tesseract_rosutils::ROSResourceLocator>();
  if (!tesseract_->init(urdf_xml_string, srdf_xml_string, locator))
    return false;

  // Create plotting tool
  tesseract_rosutils::ROSPlottingPtr plotter =
      std::make_shared<tesseract_rosutils::ROSPlotting>(tesseract_->getEnvironment()->getSceneGraph()->getRoot());

  if (rviz_)
  {
    // These are used to keep visualization updated
    modify_env_rviz_ = nh_.serviceClient<tesseract_msgs::ModifyEnvironment>(MODIFY_ENVIRONMENT_SERVICE, false);
    get_env_changes_rviz_ =
        nh_.serviceClient<tesseract_msgs::GetEnvironmentChanges>(GET_ENVIRONMENT_CHANGES_SERVICE, false);

    // Check RViz to make sure nothing has changed
    if (!checkRviz())
      return false;

    ROS_ERROR("Press enter to continue");
    std::cin.ignore(std::numeric_limits<std::streamsize>::max(), '\n');
  }

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

  tesseract_->getEnvironment()->addLink(std::move(link_sphere), std::move(joint_sphere));

  if (rviz_)
  {
    // Now update rviz environment
    if (!sendRvizChanges(0))
      return false;
  }

  // Set the robot initial state
  auto kin = tesseract_->getFwdKinematicsManagerConst()->getFwdKinematicSolver("manipulator");
  std::vector<double> swp = { -0.4, 0.2762, 0.0, -1.3348, 0.0, 1.4959, 0.0 };
  std::vector<double> ewp = { 0.4, 0.2762, 0.0, -1.3348, 0.0, 1.4959, 0.0 };

  tesseract_->getEnvironment()->setState(kin->getJointNames(), swp);

  //  plotter->plotScene();

  // Setup Problem
  tesseract_motion_planners::OMPLTrajOptFreespacePlanner hybrid_planner;

  auto ompl_config = std::make_shared<tesseract_motion_planners::OMPLPlannerFreespaceConfig>(tesseract_, "manipulator");

  ompl_config->start_waypoint = std::make_shared<tesseract_motion_planners::JointWaypoint>(swp, kin->getJointNames());
  ompl_config->end_waypoint = std::make_shared<tesseract_motion_planners::JointWaypoint>(ewp, kin->getJointNames());
  ompl_config->collision_safety_margin = 0.01;
  ompl_config->planning_time = planning_time_;
  ompl_config->max_solutions = 1;
  ompl_config->longest_valid_segment_fraction = 0.01;

  ompl_config->collision_continuous = true;
  ompl_config->collision_check = true;
  ompl_config->simplify = true;
  ompl_config->n_output_states = 2;

  for (int i = 0; i < 4; ++i)
  {
    auto rrtconnect_planner = std::make_shared<tesseract_motion_planners::RRTConnectConfigurator>();
    rrtconnect_planner->range = range_;
    ompl_config->planners.push_back(rrtconnect_planner);
  }

  auto trajopt_config = std::make_shared<tesseract_motion_planners::TrajOptPlannerFreespaceConfig>(
      tesseract_, "manipulator", "tool0", Eigen::Isometry3d::Identity());

  trajopt_config->target_waypoints.push_back(ompl_config->start_waypoint);
  trajopt_config->target_waypoints.push_back(ompl_config->end_waypoint);

  // Set the planner configuration
  hybrid_planner.setConfiguration(ompl_config, trajopt_config);

  ros::Time tStart = ros::Time::now();
  tesseract_motion_planners::PlannerResponse ompl_planning_response;
  auto status = hybrid_planner.solve(ompl_planning_response);
  ROS_ERROR("planning time: %.3f", (ros::Time::now() - tStart).toSec());

  if (plotting_)
    plotter->clear();

  if (status)
  {
    double d = 0;
    auto traj = ompl_planning_response.joint_trajectory.trajectory;
    for (unsigned i = 1; i < traj.rows(); ++i)
    {
      for (unsigned j = 0; j < traj.cols(); ++j)
      {
        d += std::abs(traj(i, j) - traj(i - 1, j));
      }
    }
    ROS_ERROR("trajectory norm: %.3f", d);

    auto env = tesseract_->getEnvironmentConst();
    std::vector<ContactResultMap> collisions;
    tesseract_environment::StateSolver::Ptr state_solver = env->getStateSolver();
    ContinuousContactManager::Ptr manager = env->getContinuousContactManager();
    AdjacencyMap::Ptr adjacency_map = std::make_shared<tesseract_environment::AdjacencyMap>(
        env->getSceneGraph(), env->getActiveLinkNames(), env->getCurrentState()->link_transforms);

    manager->setActiveCollisionObjects(adjacency_map->getActiveLinkNames());
    manager->setContactDistanceThreshold(0);
    bool found = checkTrajectory(collisions, *manager, *state_solver, kin->getJointNames(), traj);

    ROS_INFO((found) ? ("Final trajectory is in collision") : ("Final trajectory is collision free"));

    plotter->plotTrajectory(kin->getJointNames(), traj);
  }
  else
  {
    ROS_ERROR("Failed to find a valid trajector: %s", status.message().c_str());
  }

  return true;
}
}  // namespace tesseract_ros_examples
