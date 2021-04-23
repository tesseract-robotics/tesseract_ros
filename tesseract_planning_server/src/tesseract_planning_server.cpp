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

#include <tesseract_motion_planners/descartes/profile/descartes_profile.h>
#include <tesseract_motion_planners/trajopt/profile/trajopt_profile.h>
#include <tesseract_motion_planners/ompl/profile/ompl_profile.h>
#include <tesseract_motion_planners/descartes/profile/descartes_profile.h>
#include <tesseract_motion_planners/simple/profile/simple_planner_profile.h>

#include <tesseract_motion_planners/simple/profile/simple_planner_lvs_plan_profile.h>
#include <tesseract_motion_planners/ompl/profile/ompl_default_plan_profile.h>
#include <tesseract_motion_planners/descartes/profile/descartes_default_plan_profile.h>
#include <tesseract_motion_planners/trajopt/profile/trajopt_default_composite_profile.h>
#include <tesseract_motion_planners/trajopt/profile/trajopt_default_plan_profile.h>

#include <tesseract_command_language/utils/utils.h>
#include <tesseract_command_language/core/serialization.h>

#include <tesseract_rosutils/utils.h>
#include <tesseract_common/timer.h>

using tesseract_planning::Serialization;
using tesseract_rosutils::processMsg;

namespace tesseract_planning_server
{
const std::string TesseractPlanningServer::DEFAULT_GET_MOTION_PLAN_ACTION = "tesseract_get_motion_plan";

ROSProcessEnvironmentCache::ROSProcessEnvironmentCache(tesseract_monitoring::EnvironmentMonitor::Ptr env)
  : environment_(std::move(env))
{
}

void ROSProcessEnvironmentCache::setCacheSize(long size)
{
  std::unique_lock<std::shared_mutex> lock(cache_mutex_);
  cache_size_ = static_cast<std::size_t>(size);
}

long ROSProcessEnvironmentCache::getCacheSize() const { return static_cast<long>(cache_size_); }

void ROSProcessEnvironmentCache::refreshCache()
{
  std::unique_lock<std::shared_mutex> lock(cache_mutex_);
  tesseract_environment::Environment::Ptr env;
  {
    auto lock = environment_->lockEnvironmentRead();
    int rev = environment_->getEnvironment()->getRevision();
    if (rev != cache_env_revision_ || cache_.empty())
    {
      env = environment_->getEnvironment()->clone();
      cache_env_revision_ = rev;
    }
  }

  if (env != nullptr)
  {
    cache_.clear();
    for (std::size_t i = 0; i < cache_size_; ++i)
      cache_.push_back(env->clone());
  }
  else if (cache_.size() <= 2)
  {
    for (std::size_t i = (cache_.size() - 1); i < cache_size_; ++i)
      cache_.push_back(cache_.front()->clone());
  }
}

tesseract_environment::Environment::Ptr ROSProcessEnvironmentCache::getCachedEnvironment()
{
  // This is to make sure the cached items are updated if needed
  refreshCache();

  tesseract_environment::EnvState current_state;
  {
    auto lock = environment_->lockEnvironmentRead();
    current_state = *(environment_->getEnvironment()->getCurrentState());
  }

  std::unique_lock<std::shared_mutex> lock(cache_mutex_);
  tesseract_environment::Environment::Ptr env = cache_.back();

  // Update to the current joint values
  env->setState(current_state.joints);

  cache_.pop_back();

  return env;
}

TesseractPlanningServer::TesseractPlanningServer(const std::string& robot_description,
                                                 std::string name,
                                                 size_t n,
                                                 std::string discrete_plugin,
                                                 std::string continuous_plugin)
  : nh_("~")
  , environment_(std::make_shared<tesseract_monitoring::EnvironmentMonitor>(robot_description,
                                                                            name,
                                                                            discrete_plugin,
                                                                            continuous_plugin))
  , environment_cache_(std::make_shared<ROSProcessEnvironmentCache>(environment_))
  , planning_server_(std::make_shared<tesseract_planning::ProcessPlanningServer>(environment_cache_, n))
  , motion_plan_server_(nh_,
                        DEFAULT_GET_MOTION_PLAN_ACTION,
                        boost::bind(&TesseractPlanningServer::onMotionPlanningCallback, this, _1),
                        true)
  , tf_buffer_(std::make_shared<tf2_ros::Buffer>())
  , tf_listener_(*tf_buffer_)
{
  ctor();
}

TesseractPlanningServer::TesseractPlanningServer(tesseract_environment::Environment::Ptr env,
                                                 std::string name,
                                                 size_t n,
                                                 std::string discrete_plugin,
                                                 std::string continuous_plugin)
  : nh_("~")
  , environment_(
        std::make_shared<tesseract_monitoring::EnvironmentMonitor>(env, name, discrete_plugin, continuous_plugin))
  , environment_cache_(std::make_shared<ROSProcessEnvironmentCache>(environment_))
  , planning_server_(std::make_shared<tesseract_planning::ProcessPlanningServer>(environment_cache_, n))
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
  auto lock = environment_->lockEnvironmentWrite();
  environment_->getEnvironment()->addFindTCPCallback(
      std::bind(&TesseractPlanningServer::tfFindTCP, this, std::placeholders::_1));
}

tesseract_monitoring::EnvironmentMonitor& TesseractPlanningServer::getEnvironmentMonitor() { return *environment_; }
const tesseract_monitoring::EnvironmentMonitor& TesseractPlanningServer::getEnvironmentMonitor() const
{
  return *environment_;
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
  ROS_INFO("Tesseract Planning Server Recieved Request!");
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
        Serialization::fromArchiveStringXML<tesseract_planning::Instruction>(goal->request.instructions);
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
    process_request.seed = Serialization::fromArchiveStringXML<tesseract_planning::Instruction>(goal->request.seed);

  auto env_state = std::make_shared<tesseract_environment::EnvState>();
  tesseract_rosutils::fromMsg(env_state->joints, goal->request.environment_state.joint_state);

  process_request.env_state = env_state;
  process_request.commands = tesseract_rosutils::fromMsg(goal->request.commands);
  process_request.profile = goal->request.profile;
  process_request.plan_profile_remapping = tesseract_rosutils::fromMsg(goal->request.plan_profile_remapping);
  process_request.composite_profile_remapping = tesseract_rosutils::fromMsg(goal->request.composite_profile_remapping);

  tesseract_common::Timer timer;
  timer.start();
  tesseract_planning::ProcessPlanningFuture plan_future = planning_server_->run(process_request);
  plan_future.wait();  // Wait for results
  timer.stop();

  result.response.successful = plan_future.interface->isSuccessful();
  result.response.results = Serialization::toArchiveStringXML<tesseract_planning::Instruction>(*(plan_future.results));
  plan_future.clear();

  ROS_INFO("Tesseract Planning Server Finished Request in %f seconds!", timer.elapsedSeconds());
  motion_plan_server_.setSucceeded(result);
}

void TesseractPlanningServer::loadDefaultPlannerProfiles()
{
  tesseract_planning::ProfileDictionary::Ptr profiles = planning_server_->getProfiles();
  profiles->addProfile<tesseract_planning::TrajOptPlanProfile>(
      tesseract_planning::DEFAULT_PROFILE_KEY, std::make_shared<tesseract_planning::TrajOptDefaultPlanProfile>());
  profiles->addProfile<tesseract_planning::TrajOptCompositeProfile>(
      tesseract_planning::DEFAULT_PROFILE_KEY, std::make_shared<tesseract_planning::TrajOptDefaultCompositeProfile>());
  profiles->addProfile<tesseract_planning::DescartesPlanProfile<double>>(
      tesseract_planning::DEFAULT_PROFILE_KEY,
      std::make_shared<tesseract_planning::DescartesDefaultPlanProfile<double>>());
  profiles->addProfile<tesseract_planning::OMPLPlanProfile>(
      tesseract_planning::DEFAULT_PROFILE_KEY, std::make_shared<tesseract_planning::OMPLDefaultPlanProfile>());
  profiles->addProfile<tesseract_planning::SimplePlannerPlanProfile>(
      tesseract_planning::DEFAULT_PROFILE_KEY, std::make_shared<tesseract_planning::SimplePlannerLVSPlanProfile>());
}

Eigen::Isometry3d TesseractPlanningServer::tfFindTCP(const tesseract_planning::ManipulatorInfo& manip_info)
{
  if (!manip_info.tcp.isString())
    throw std::runtime_error("tfFindTCP: TCP is not a string!");

  auto composite_mi_fwd_kin =
      environment_->getEnvironment()->getManipulatorManager()->getFwdKinematicSolver(manip_info.manipulator);
  if (composite_mi_fwd_kin == nullptr)
    throw std::runtime_error("tfFindTCP: Manipulator '" + manip_info.manipulator + "' does not exist!");

  const std::string& tip_link = composite_mi_fwd_kin->getTipLinkName();
  const std::string& tcp_name = manip_info.tcp.getString();

  auto tcp_msg = tf_buffer_->lookupTransform(tip_link, tcp_name, ros::Time(0), ros::Duration(2));
  return tf2::transformToEigen(tcp_msg.transform);
}

}  // namespace tesseract_planning_server
