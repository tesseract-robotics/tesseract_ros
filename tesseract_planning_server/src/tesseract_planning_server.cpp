/**
 * @file tesseract_planning_server.cpp
 * @brief A planning server with a default set of motion planners
 *
 * @author Levi Armstrong
 * @date August 18, 2020
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
#include <tf2_eigen/tf2_eigen.h>
#include <console_bridge/console.h>
TESSERACT_COMMON_IGNORE_WARNINGS_POP

#include <tesseract_planning_server/tesseract_planning_server.h>

#include <tesseract_motion_planners/default_planner_namespaces.h>
#include <tesseract_motion_planners/descartes/profile/descartes_profile.h>
#include <tesseract_motion_planners/trajopt/profile/trajopt_profile.h>
#include <tesseract_motion_planners/trajopt_ifopt/profile/trajopt_ifopt_profile.h>
#include <tesseract_motion_planners/ompl/profile/ompl_profile.h>
#include <tesseract_motion_planners/descartes/profile/descartes_profile.h>
#include <tesseract_motion_planners/simple/profile/simple_planner_profile.h>

#include <tesseract_motion_planners/simple/profile/simple_planner_lvs_no_ik_legacy_plan_profile.h>
#include <tesseract_motion_planners/ompl/profile/ompl_default_plan_profile.h>
#include <tesseract_motion_planners/descartes/profile/descartes_default_plan_profile.h>
#include <tesseract_motion_planners/trajopt/profile/trajopt_default_composite_profile.h>
#include <tesseract_motion_planners/trajopt/profile/trajopt_default_plan_profile.h>
#include <tesseract_motion_planners/trajopt/profile/trajopt_default_solver_profile.h>
#include <tesseract_motion_planners/trajopt_ifopt/profile/trajopt_ifopt_default_composite_profile.h>
#include <tesseract_motion_planners/trajopt_ifopt/profile/trajopt_ifopt_default_plan_profile.h>

#include <tesseract_monitoring/environment_monitor.h>
#include <tesseract_rosutils/utils.h>
#include <tesseract_common/serialization.h>
#include <tesseract_common/timer.h>

using tesseract_common::Serialization;
using tesseract_rosutils::processMsg;

namespace tesseract_planning_server
{
const std::string TesseractPlanningServer::DEFAULT_GET_MOTION_PLAN_ACTION = "tesseract_get_motion_plan";

TesseractPlanningServer::TesseractPlanningServer(const std::string& robot_description, std::string name, size_t n)
  : nh_("~")
  , monitor_(std::make_shared<tesseract_monitoring::ROSEnvironmentMonitor>(robot_description, name))
  , environment_cache_(std::make_shared<tesseract_planning::ProcessEnvironmentCache>(monitor_->getEnvironment()))
  , planning_server_(std::make_unique<tesseract_planning::ProcessPlanningServer>(environment_cache_, n))
  , motion_plan_server_(nh_,
                        DEFAULT_GET_MOTION_PLAN_ACTION,
                        boost::bind(&TesseractPlanningServer::onMotionPlanningCallback, this, _1),
                        true)
  , tf_buffer_(std::make_shared<tf2_ros::Buffer>())
  , tf_listener_(*tf_buffer_)
{
  ctor();
}

TesseractPlanningServer::TesseractPlanningServer(tesseract_environment::Environment::UPtr env,
                                                 std::string name,
                                                 size_t n)
  : nh_("~")
  , monitor_(std::make_shared<tesseract_monitoring::ROSEnvironmentMonitor>(std::move(env), name))
  , environment_cache_(std::make_shared<tesseract_planning::ProcessEnvironmentCache>(monitor_->getEnvironment()))
  , planning_server_(std::make_unique<tesseract_planning::ProcessPlanningServer>(environment_cache_, n))
  , motion_plan_server_(nh_,
                        DEFAULT_GET_MOTION_PLAN_ACTION,
                        boost::bind(&TesseractPlanningServer::onMotionPlanningCallback, this, _1),
                        true)
  , tf_buffer_(std::make_shared<tf2_ros::Buffer>())
  , tf_listener_(*tf_buffer_)
{
  ctor();
}

void TesseractPlanningServer::ctor()
{
  planning_server_->loadDefaultProcessPlanners();
  loadDefaultPlannerProfiles();
  monitor_->environment().addFindTCPOffsetCallback(
      std::bind(&TesseractPlanningServer::tfFindTCPOffset, this, std::placeholders::_1));
}

tesseract_environment::EnvironmentMonitor& TesseractPlanningServer::getEnvironmentMonitor() { return *monitor_; }
const tesseract_environment::EnvironmentMonitor& TesseractPlanningServer::getEnvironmentMonitor() const
{
  return *monitor_;
}

tesseract_planning::ProcessPlanningServer& TesseractPlanningServer::getProcessPlanningServer()
{
  return *planning_server_;
}
const tesseract_planning::ProcessPlanningServer& TesseractPlanningServer::getProcessPlanningServer() const
{
  return *planning_server_;
}

tesseract_planning::EnvironmentCache& TesseractPlanningServer::getEnvironmentCache() { return *environment_cache_; }
const tesseract_planning::EnvironmentCache& TesseractPlanningServer::getEnvironmentCache() const
{
  return *environment_cache_;
}

void TesseractPlanningServer::onMotionPlanningCallback(const tesseract_msgs::GetMotionPlanGoalConstPtr& goal)
{
  ROS_INFO("Tesseract Planning Server Received Request!");
  tesseract_msgs::GetMotionPlanResult result;

  // Check if process planner exist
  if (!planning_server_->hasProcessPlanner(goal->request.name))
  {
    result.response.successful = false;
    std::ostringstream oss;
    oss << "Requested process planner '" << goal->request.name << "' is not supported!" << std::endl;
    oss << "   Available Process Planners:" << std::endl;
    for (const auto& planner : planning_server_->getAvailableProcessPlanners())
      oss << "      - " << planner << std::endl;
    ROS_ERROR_STREAM(oss.str());
    motion_plan_server_.setSucceeded(result);
    return;
  }

  tesseract_planning::ProcessPlanningRequest process_request;
  process_request.name = goal->request.name;

  try
  {
    process_request.instructions =
        Serialization::fromArchiveStringXML<tesseract_planning::InstructionPoly>(goal->request.instructions);
  }
  catch (const std::exception& e)
  {
    result.response.successful = false;
    std::ostringstream oss;
    oss << "Failed to deserialize program instruction with error: '" << e.what() << "'!" << std::endl;
    oss << "   Make sure the program was serialized from an Instruction type and not a CompositeInstruction type."
        << std::endl;
    ROS_ERROR_STREAM(oss.str());
    motion_plan_server_.setSucceeded(result);
    return;
  }

  if (!goal->request.seed.empty())
    process_request.seed = Serialization::fromArchiveStringXML<tesseract_planning::InstructionPoly>(goal->request.seed);

  tesseract_scene_graph::SceneState env_state;
  tesseract_rosutils::fromMsg(env_state.joints, goal->request.environment_state.joint_state);

  process_request.env_state = env_state;
  process_request.commands = tesseract_rosutils::fromMsg(goal->request.commands);
  process_request.profile = goal->request.profile;
  process_request.save_io = goal->request.save_io;
  process_request.plan_profile_remapping = tesseract_rosutils::fromMsg(goal->request.plan_profile_remapping);
  process_request.composite_profile_remapping = tesseract_rosutils::fromMsg(goal->request.composite_profile_remapping);

  tesseract_common::Timer timer;
  timer.start();
  tesseract_planning::ProcessPlanningFuture plan_future = planning_server_->run(process_request);
  plan_future.wait();  // Wait for results
  timer.stop();

  // Store the initial state in the response for publishing trajectories
  tesseract_scene_graph::SceneState initial_state = plan_future.problem->env->getState();
  tesseract_rosutils::toMsg(result.response.initial_state, initial_state.joints);

  result.response.successful = plan_future.interface->isSuccessful();
  result.response.results =
      Serialization::toArchiveStringXML<tesseract_planning::InstructionPoly>(*(plan_future.problem->results));
  plan_future.clear();

  ROS_INFO("Tesseract Planning Server Finished Request in %f seconds!", timer.elapsedSeconds());
  motion_plan_server_.setSucceeded(result);
}

void TesseractPlanningServer::loadDefaultPlannerProfiles()
{
  tesseract_planning::ProfileDictionary::Ptr profiles = planning_server_->getProfiles();

  // Add TrajOpt Default Profiles
  profiles->addProfile<tesseract_planning::TrajOptPlanProfile>(
      tesseract_planning::profile_ns::TRAJOPT_DEFAULT_NAMESPACE,
      tesseract_planning::DEFAULT_PROFILE_KEY,
      std::make_shared<tesseract_planning::TrajOptDefaultPlanProfile>());
  profiles->addProfile<tesseract_planning::TrajOptCompositeProfile>(
      tesseract_planning::profile_ns::TRAJOPT_DEFAULT_NAMESPACE,
      tesseract_planning::DEFAULT_PROFILE_KEY,
      std::make_shared<tesseract_planning::TrajOptDefaultCompositeProfile>());
  profiles->addProfile<tesseract_planning::TrajOptSolverProfile>(
      tesseract_planning::profile_ns::TRAJOPT_DEFAULT_NAMESPACE,
      tesseract_planning::DEFAULT_PROFILE_KEY,
      std::make_shared<tesseract_planning::TrajOptDefaultSolverProfile>());

  // Add TrajOpt IFOPT Default Profiles
  profiles->addProfile<tesseract_planning::TrajOptIfoptPlanProfile>(
      tesseract_planning::profile_ns::TRAJOPT_IFOPT_DEFAULT_NAMESPACE,
      tesseract_planning::DEFAULT_PROFILE_KEY,
      std::make_shared<tesseract_planning::TrajOptIfoptDefaultPlanProfile>());
  profiles->addProfile<tesseract_planning::TrajOptIfoptCompositeProfile>(
      tesseract_planning::profile_ns::TRAJOPT_IFOPT_DEFAULT_NAMESPACE,
      tesseract_planning::DEFAULT_PROFILE_KEY,
      std::make_shared<tesseract_planning::TrajOptIfoptDefaultCompositeProfile>());

  // Add Descartes Default Profiles
  profiles->addProfile<tesseract_planning::DescartesPlanProfile<double>>(
      tesseract_planning::profile_ns::DESCARTES_DEFAULT_NAMESPACE,
      tesseract_planning::DEFAULT_PROFILE_KEY,
      std::make_shared<tesseract_planning::DescartesDefaultPlanProfile<double>>());

  // Add OMPL Default Profiles
  profiles->addProfile<tesseract_planning::OMPLPlanProfile>(
      tesseract_planning::profile_ns::OMPL_DEFAULT_NAMESPACE,
      tesseract_planning::DEFAULT_PROFILE_KEY,
      std::make_shared<tesseract_planning::OMPLDefaultPlanProfile>());

  // Add Simple Default Profiles
  profiles->addProfile<tesseract_planning::SimplePlannerLegacyPlanProfile>(
      tesseract_planning::profile_ns::SIMPLE_DEFAULT_NAMESPACE,
      tesseract_planning::DEFAULT_PROFILE_KEY,
      std::make_shared<tesseract_planning::SimplePlannerLVSNoIKLegacyPlanProfile>());
}

Eigen::Isometry3d TesseractPlanningServer::tfFindTCPOffset(const tesseract_common::ManipulatorInfo& manip_info)
{
  if (manip_info.tcp_offset.index() == 1)
    throw std::runtime_error("tfFindTCPOffset: TCP offset is not a string!");

  if (manip_info.tcp_frame.empty())
    throw std::runtime_error("tfFindTCPOffset: TCP offset is empty!");

  const std::string& tcp_frame = manip_info.tcp_frame;
  const std::string& tcp_name = std::get<0>(manip_info.tcp_offset);

  auto tcp_msg = tf_buffer_->lookupTransform(tcp_frame, tcp_name, ros::Time(0), ros::Duration(2));
  return tf2::transformToEigen(tcp_msg.transform);
}

}  // namespace tesseract_planning_server
