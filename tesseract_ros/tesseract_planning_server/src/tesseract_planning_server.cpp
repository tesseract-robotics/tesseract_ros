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
TESSERACT_COMMON_IGNORE_WARNINGS_POP

#include <tesseract_planning_server/tesseract_planning_server.h>

#include <tesseract_motion_planners/simple/profile/simple_planner_default_lvs_plan_profile.h>
#include <tesseract_motion_planners/ompl/profile/ompl_default_plan_profile.h>
#include <tesseract_motion_planners/descartes/profile/descartes_default_plan_profile.h>
#include <tesseract_motion_planners/trajopt/profile/trajopt_default_composite_profile.h>
#include <tesseract_motion_planners/trajopt/profile/trajopt_default_plan_profile.h>

#include <tesseract_process_managers/taskflows/descartes_taskflow.h>
#include <tesseract_process_managers/taskflows/ompl_taskflow.h>
#include <tesseract_process_managers/taskflows/trajopt_taskflow.h>
#include <tesseract_process_managers/taskflows/cartesian_taskflow.h>
#include <tesseract_process_managers/taskflows/freespace_taskflow.h>

#include <tesseract_command_language/utils/utils.h>
#include <tesseract_command_language/deserialize.h>
#include <tesseract_command_language/serialize.h>

#include <tesseract_rosutils/utils.h>

using tesseract_rosutils::processMsg;

namespace tesseract_planning_server
{
const std::string TesseractPlanningServer::DEFAULT_GET_MOTION_PLAN_ACTION = "tesseract_get_motion_plan";

TesseractPlanningServer::TesseractPlanningServer(const std::string& robot_description,
                                                 std::string name,
                                                 std::string discrete_plugin,
                                                 std::string continuous_plugin)
  : nh_("~")
  , environment_(robot_description, name, discrete_plugin, continuous_plugin)
  , motion_plan_server_(nh_,
                        DEFAULT_GET_MOTION_PLAN_ACTION,
                        boost::bind(&TesseractPlanningServer::onMotionPlanningCallback, this, _1),
                        true)
  , tf_buffer_(std::make_shared<tf2_ros::Buffer>())
  , tf_listener_(*tf_buffer_)
{
  ctor();
}

TesseractPlanningServer::TesseractPlanningServer(std::shared_ptr<tesseract::Tesseract> tesseract,
                                                 std::string name,
                                                 std::string discrete_plugin,
                                                 std::string continuous_plugin)
  : nh_("~")
  , environment_(tesseract, name, discrete_plugin, continuous_plugin)
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
  loadDefaultPlannerProfiles();
  auto lock = environment_.lockEnvironmentWrite();
  environment_.getTesseract()->addFindTCPCallback(
      std::bind(&TesseractPlanningServer::tfFindTCP, this, std::placeholders::_1));
}

tesseract_monitoring::EnvironmentMonitor& TesseractPlanningServer::getEnvironmentMonitor() { return environment_; }
const tesseract_monitoring::EnvironmentMonitor& TesseractPlanningServer::getEnvironmentMonitor() const
{
  return environment_;
}

void TesseractPlanningServer::setCacheSize(long size)
{
  std::unique_lock<std::shared_mutex> lock(cache_mutex_);
  cache_size_ = static_cast<std::size_t>(size);
}

long TesseractPlanningServer::getCacheSize() const { return static_cast<long>(cache_size_); }

void TesseractPlanningServer::refreshCache()
{
  std::unique_lock<std::shared_mutex> lock(cache_mutex_);
  tesseract::Tesseract::Ptr thor;
  {
    auto lock = environment_.lockEnvironmentRead();
    int rev = environment_.getEnvironment()->getRevision();
    if (rev != cache_env_revision_ || cache_.empty())
    {
      thor = environment_.getTesseract()->clone();
      cache_env_revision_ = rev;
    }
  }

  if (thor != nullptr)
  {
    cache_.clear();
    for (std::size_t i = 0; i < cache_size_; ++i)
      cache_.push_back(thor->clone());
  }
  else if (cache_.size() <= 2)
  {
    for (std::size_t i = (cache_.size() - 1); i < cache_size_; ++i)
      cache_.push_back(thor->clone());
  }
}

tesseract::Tesseract::Ptr TesseractPlanningServer::getCachedTesseract()
{
  // This is to make sure the cached items are updated if needed
  refreshCache();

  tesseract_environment::EnvState current_state;
  {
    auto lock = environment_.lockEnvironmentRead();
    current_state = *(environment_.getEnvironment()->getCurrentState());
  }

  std::unique_lock<std::shared_mutex> lock(cache_mutex_);
  tesseract::Tesseract::Ptr t = cache_.back();

  // Update to the current joint values
  t->getEnvironment()->setState(current_state.joints);

  cache_.pop_back();

  return t;
}

void TesseractPlanningServer::onMotionPlanningCallback(const tesseract_msgs::GetMotionPlanGoalConstPtr& goal)
{
  ROS_INFO("Tesseract Planning Server Recieved Request!");
  tesseract_planning::Instruction program = tesseract_planning::fromXMLString(goal->request.instructions);
  const auto* composite_program = program.cast_const<tesseract_planning::CompositeInstruction>();

  tesseract_planning::Instruction seed = tesseract_planning::generateSkeletonSeed(*composite_program);
  if (!goal->request.seed.empty())
    seed = tesseract_planning::fromXMLString(goal->request.seed);

  tesseract_msgs::GetMotionPlanResult result;
  tesseract_planning::ProcessManager::Ptr pm;
  if (goal->request.name == goal->request.TRAJOPT_PLANNER_NAME)
    pm = createTrajOptProcessManager(goal);
  else if (goal->request.name == goal->request.OMPL_PLANNER_NAME)
    pm = createOMPLProcessManager(goal);
  else if (goal->request.name == goal->request.DESCARTES_PLANNER_NAME)
    pm = createDescartesProcessManager(goal);
  else if (goal->request.name == goal->request.CARTESIAN_PLANNER_NAME)
    pm = createCartesianProcessManager(goal);
  else if (goal->request.name == goal->request.FREESPACE_PLANNER_NAME)
    pm = createFreespaceProcessManager(goal);
  else if (goal->request.name == goal->request.RASTER_FT_PLANNER_NAME)
    pm = createRasterProcessManager(goal);
  else if (goal->request.name == goal->request.RASTER_O_FT_PLANNER_NAME)
    pm = createRasterOnlyProcessManager(goal);
  else if (goal->request.name == goal->request.RASTER_G_FT_PLANNER_NAME)
    pm = createRasterGlobalProcessManager(goal);
  else if (goal->request.name == goal->request.RASTER_FT_DT_PLANNER_NAME)
    pm = createRasterDTProcessManager(goal);
  else if (goal->request.name == goal->request.RASTER_FT_WAAD_PLANNER_NAME)
    pm = createRasterWAADProcessManager(goal);
  else if (goal->request.name == goal->request.RASTER_FT_WAAD_DT_PLANNER_NAME)
    pm = createRasterWAADDTProcessManager(goal);
  else if (goal->request.name == goal->request.RASTER_O_G_FT_PLANNER_NAME)
    pm = createRasterOnlyGlobalProcessManager(goal);
  else if (goal->request.name == goal->request.RASTER_CT_PLANNER_NAME)
    pm = createRasterDTProcessManager(goal);
  else if (goal->request.name == goal->request.RASTER_O_CT_PLANNER_NAME)
    pm = createRasterOnlyCTProcessManager(goal);
  else if (goal->request.name == goal->request.RASTER_CT_DT_PLANNER_NAME)
    pm = createRasterCTDTProcessManager(goal);
  else if (goal->request.name == goal->request.RASTER_CT_WAAD_PLANNER_NAME)
    pm = createRasterCTWAADProcessManager(goal);
  else if (goal->request.name == goal->request.RASTER_CT_WAAD_DT_PLANNER_NAME)
    pm = createRasterCTWAADDTProcessManager(goal);
  else if (goal->request.name == goal->request.RASTER_G_CT_PLANNER_NAME)
    pm = createRasterGlobalCTProcessManager(goal);
  else if (goal->request.name == goal->request.RASTER_O_G_CT_PLANNER_NAME)
    pm = createRasterOnlyGlobalCTProcessManager(goal);
  else
  {
    result.response.successful = false;
    ROS_ERROR("Requested motion planner is not supported!");
    motion_plan_server_.setSucceeded(result);
    return;
  }

  assert(pm != nullptr);
  tesseract::Tesseract::Ptr tc = getCachedTesseract();
  tesseract_planning::ManipulatorInfo mi = composite_program->getManipulatorInfo();
  if (!goal->request.commands.empty() && !processMsg(*(tc->getEnvironment()), goal->request.commands))
  {
    result.response.successful = false;
    result.response.status_string = "Failed to apply provided commands to the environment";
    ROS_INFO("Tesseract Planning Server Finished Request!");
    motion_plan_server_.setSucceeded(result);
  }
  pm->enableDebug(goal->request.debug);
  pm->enableProfile(goal->request.profile);
  pm->init(tesseract_planning::ProcessInput(tc, &program, mi, &seed, goal->request.debug));

  result.response.successful = pm->execute();
  result.response.results = tesseract_planning::toXMLString(seed);

  ROS_INFO("Tesseract Planning Server Finished Request!");
  motion_plan_server_.setSucceeded(result);
}

void TesseractPlanningServer::loadDefaultPlannerProfiles()
{
  trajopt_plan_profiles_[tesseract_planning::DEFAULT_PROFILE_KEY] =
      std::make_shared<tesseract_planning::TrajOptDefaultPlanProfile>();
  trajopt_composite_profiles_[tesseract_planning::DEFAULT_PROFILE_KEY] =
      std::make_shared<tesseract_planning::TrajOptDefaultCompositeProfile>();
  descartes_plan_profiles_[tesseract_planning::DEFAULT_PROFILE_KEY] =
      std::make_shared<tesseract_planning::DescartesDefaultPlanProfile<double>>();
  ompl_plan_profiles_[tesseract_planning::DEFAULT_PROFILE_KEY] =
      std::make_shared<tesseract_planning::OMPLDefaultPlanProfile>();
  simple_plan_profiles_[tesseract_planning::DEFAULT_PROFILE_KEY] =
      std::make_shared<tesseract_planning::SimplePlannerDefaultLVSPlanProfile>();
}

Eigen::Isometry3d TesseractPlanningServer::tfFindTCP(const tesseract_planning::ManipulatorInfo& manip_info)
{
  if (!manip_info.tcp.isString())
    throw std::runtime_error("tfFindTCP: TCP is not a string!");

  auto composite_mi_fwd_kin =
      environment_.getTesseract()->getEnvironment()->getManipulatorManager()->getFwdKinematicSolver(
          manip_info.manipulator);
  if (composite_mi_fwd_kin == nullptr)
    throw std::runtime_error("tfFindTCP: Manipulator '" + manip_info.manipulator + "' does not exist!");

  const std::string& tip_link = composite_mi_fwd_kin->getTipLinkName();
  const std::string& tcp_name = manip_info.tcp.getString();

  auto tcp_msg = tf_buffer_->lookupTransform(tip_link, tcp_name, ros::Time(0), ros::Duration(2));
  return tf2::transformToEigen(tcp_msg.transform);
}

tesseract_planning::SimpleProcessManager::Ptr
TesseractPlanningServer::createTrajOptProcessManager(const tesseract_msgs::GetMotionPlanGoalConstPtr& goal)
{
  tesseract_planning::TrajOptTaskflowParams params;
  params.enable_simple_planner = goal->request.seed.empty();
  params.simple_plan_profiles = simple_plan_profiles_;
  params.simple_composite_profiles = simple_composite_profiles_;
  params.trajopt_plan_profiles = trajopt_plan_profiles_;
  params.trajopt_composite_profiles = trajopt_composite_profiles_;
  tesseract_planning::GraphTaskflow::UPtr task = tesseract_planning::createTrajOptTaskflow(params);

  std::size_t nt = std::thread::hardware_concurrency();
  if (goal->request.num_threads != 0)
    nt = goal->request.num_threads;

  return std::make_shared<tesseract_planning::SimpleProcessManager>(std::move(task), nt);
}

tesseract_planning::SimpleProcessManager::Ptr
TesseractPlanningServer::createDescartesProcessManager(const tesseract_msgs::GetMotionPlanGoalConstPtr& goal)
{
  tesseract_planning::DescartesTaskflowParams params;
  params.enable_simple_planner = goal->request.seed.empty();
  params.simple_plan_profiles = simple_plan_profiles_;
  params.simple_composite_profiles = simple_composite_profiles_;
  params.descartes_plan_profiles = descartes_plan_profiles_;
  tesseract_planning::GraphTaskflow::UPtr task = tesseract_planning::createDescartesTaskflow(params);

  std::size_t nt = std::thread::hardware_concurrency();
  if (goal->request.num_threads != 0)
    nt = goal->request.num_threads;

  return std::make_shared<tesseract_planning::SimpleProcessManager>(std::move(task), nt);
}

tesseract_planning::SimpleProcessManager::Ptr
TesseractPlanningServer::createOMPLProcessManager(const tesseract_msgs::GetMotionPlanGoalConstPtr& goal)
{
  tesseract_planning::OMPLTaskflowParams params;
  params.enable_simple_planner = goal->request.seed.empty();
  params.simple_plan_profiles = simple_plan_profiles_;
  params.simple_composite_profiles = simple_composite_profiles_;
  params.ompl_plan_profiles = ompl_plan_profiles_;
  tesseract_planning::GraphTaskflow::UPtr task = tesseract_planning::createOMPLTaskflow(params);

  std::size_t nt = std::thread::hardware_concurrency();
  if (goal->request.num_threads != 0)
    nt = goal->request.num_threads;

  return std::make_shared<tesseract_planning::SimpleProcessManager>(std::move(task), nt);
}

tesseract_planning::SimpleProcessManager::Ptr
TesseractPlanningServer::createCartesianProcessManager(const tesseract_msgs::GetMotionPlanGoalConstPtr& goal)
{
  tesseract_planning::CartesianTaskflowParams params;
  params.enable_simple_planner = goal->request.seed.empty();
  params.simple_plan_profiles = simple_plan_profiles_;
  params.simple_composite_profiles = simple_composite_profiles_;
  params.descartes_plan_profiles = descartes_plan_profiles_;
  params.trajopt_plan_profiles = trajopt_plan_profiles_;
  params.trajopt_composite_profiles = trajopt_composite_profiles_;
  tesseract_planning::GraphTaskflow::UPtr task = tesseract_planning::createCartesianTaskflow(params);

  std::size_t nt = std::thread::hardware_concurrency();
  if (goal->request.num_threads != 0)
    nt = goal->request.num_threads;

  return std::make_shared<tesseract_planning::SimpleProcessManager>(std::move(task), nt);
}

tesseract_planning::SimpleProcessManager::Ptr
TesseractPlanningServer::createFreespaceProcessManager(const tesseract_msgs::GetMotionPlanGoalConstPtr& goal)
{
  tesseract_planning::FreespaceTaskflowParams params;
  params.enable_simple_planner = goal->request.seed.empty();
  params.simple_plan_profiles = simple_plan_profiles_;
  params.simple_composite_profiles = simple_composite_profiles_;
  params.ompl_plan_profiles = ompl_plan_profiles_;
  params.trajopt_plan_profiles = trajopt_plan_profiles_;
  params.trajopt_composite_profiles = trajopt_composite_profiles_;
  tesseract_planning::GraphTaskflow::UPtr task = tesseract_planning::createFreespaceTaskflow(params);

  std::size_t nt = std::thread::hardware_concurrency();
  if (goal->request.num_threads != 0)
    nt = goal->request.num_threads;

  return std::make_shared<tesseract_planning::SimpleProcessManager>(std::move(task), nt);
}

tesseract_planning::RasterProcessManager::Ptr
TesseractPlanningServer::createRasterProcessManager(const tesseract_msgs::GetMotionPlanGoalConstPtr& goal)
{
  // Create Freespace and Transition Taskflows
  tesseract_planning::FreespaceTaskflowParams fparams;
  fparams.enable_simple_planner = goal->request.seed.empty();
  fparams.simple_plan_profiles = simple_plan_profiles_;
  fparams.simple_composite_profiles = simple_composite_profiles_;
  fparams.ompl_plan_profiles = ompl_plan_profiles_;
  fparams.trajopt_plan_profiles = trajopt_plan_profiles_;
  fparams.trajopt_composite_profiles = trajopt_composite_profiles_;
  tesseract_planning::GraphTaskflow::UPtr freespace_task = tesseract_planning::createFreespaceTaskflow(fparams);
  tesseract_planning::GraphTaskflow::UPtr transition_task = tesseract_planning::createFreespaceTaskflow(fparams);

  // Create Raster Taskflow
  tesseract_planning::CartesianTaskflowParams cparams;
  cparams.enable_simple_planner = goal->request.seed.empty();
  cparams.simple_plan_profiles = simple_plan_profiles_;
  cparams.simple_composite_profiles = simple_composite_profiles_;
  cparams.descartes_plan_profiles = descartes_plan_profiles_;
  cparams.trajopt_plan_profiles = trajopt_plan_profiles_;
  cparams.trajopt_composite_profiles = trajopt_composite_profiles_;
  tesseract_planning::GraphTaskflow::UPtr raster_task = tesseract_planning::createCartesianTaskflow(cparams);

  std::size_t nt = std::thread::hardware_concurrency();
  if (goal->request.num_threads != 0)
    nt = goal->request.num_threads;

  return std::make_shared<tesseract_planning::RasterProcessManager>(
      std::move(freespace_task), std::move(transition_task), std::move(raster_task), nt);
}

tesseract_planning::RasterOnlyProcessManager::Ptr
TesseractPlanningServer::createRasterOnlyProcessManager(const tesseract_msgs::GetMotionPlanGoalConstPtr& goal)
{
  // Create Freespace and Transition Taskflows
  tesseract_planning::FreespaceTaskflowParams tparams;
  tparams.enable_simple_planner = goal->request.seed.empty();
  tparams.simple_plan_profiles = simple_plan_profiles_;
  tparams.simple_composite_profiles = simple_composite_profiles_;
  tparams.ompl_plan_profiles = ompl_plan_profiles_;
  tparams.trajopt_plan_profiles = trajopt_plan_profiles_;
  tparams.trajopt_composite_profiles = trajopt_composite_profiles_;
  tesseract_planning::GraphTaskflow::UPtr transition_task = tesseract_planning::createFreespaceTaskflow(tparams);

  // Create Raster Taskflow
  tesseract_planning::CartesianTaskflowParams cparams;
  cparams.enable_simple_planner = goal->request.seed.empty();
  cparams.simple_plan_profiles = simple_plan_profiles_;
  cparams.simple_composite_profiles = simple_composite_profiles_;
  cparams.descartes_plan_profiles = descartes_plan_profiles_;
  cparams.trajopt_plan_profiles = trajopt_plan_profiles_;
  cparams.trajopt_composite_profiles = trajopt_composite_profiles_;
  tesseract_planning::GraphTaskflow::UPtr raster_task = tesseract_planning::createCartesianTaskflow(cparams);

  std::size_t nt = std::thread::hardware_concurrency();
  if (goal->request.num_threads != 0)
    nt = goal->request.num_threads;

  return std::make_shared<tesseract_planning::RasterOnlyProcessManager>(
      std::move(transition_task), std::move(raster_task), nt);
}

tesseract_planning::RasterGlobalProcessManager::Ptr
TesseractPlanningServer::createRasterGlobalProcessManager(const tesseract_msgs::GetMotionPlanGoalConstPtr& goal)
{
  tesseract_planning::DescartesTaskflowParams global_params;
  global_params.enable_simple_planner = goal->request.seed.empty();
  global_params.enable_post_contact_discrete_check = false;
  global_params.enable_post_contact_continuous_check = false;
  global_params.enable_time_parameterization = false;
  global_params.simple_plan_profiles = simple_plan_profiles_;
  global_params.simple_composite_profiles = simple_composite_profiles_;
  global_params.descartes_plan_profiles = descartes_plan_profiles_;
  tesseract_planning::GraphTaskflow::UPtr global_task = tesseract_planning::createDescartesTaskflow(global_params);

  tesseract_planning::FreespaceTaskflowParams freespace_params;
  freespace_params.type = tesseract_planning::FreespaceTaskflowType::TRAJOPT_FIRST;
  freespace_params.enable_simple_planner = false;
  freespace_params.simple_plan_profiles = simple_plan_profiles_;
  freespace_params.simple_composite_profiles = simple_composite_profiles_;
  freespace_params.ompl_plan_profiles = ompl_plan_profiles_;
  freespace_params.trajopt_plan_profiles = trajopt_plan_profiles_;
  freespace_params.trajopt_composite_profiles = trajopt_composite_profiles_;
  tesseract_planning::GraphTaskflow::UPtr freespace_task =
      tesseract_planning::createFreespaceTaskflow(freespace_params);
  tesseract_planning::GraphTaskflow::UPtr transition_task =
      tesseract_planning::createFreespaceTaskflow(freespace_params);

  tesseract_planning::TrajOptTaskflowParams raster_params;
  raster_params.enable_simple_planner = false;
  raster_params.simple_plan_profiles = simple_plan_profiles_;
  raster_params.simple_composite_profiles = simple_composite_profiles_;
  raster_params.trajopt_plan_profiles = trajopt_plan_profiles_;
  raster_params.trajopt_composite_profiles = trajopt_composite_profiles_;
  tesseract_planning::GraphTaskflow::UPtr raster_task = tesseract_planning::createTrajOptTaskflow(raster_params);

  std::size_t nt = std::thread::hardware_concurrency();
  if (goal->request.num_threads != 0)
    nt = goal->request.num_threads;

  return std::make_shared<tesseract_planning::RasterGlobalProcessManager>(
      std::move(global_task), std::move(freespace_task), std::move(transition_task), std::move(raster_task), nt);
}

tesseract_planning::RasterDTProcessManager::Ptr
TesseractPlanningServer::createRasterDTProcessManager(const tesseract_msgs::GetMotionPlanGoalConstPtr& goal)
{
  // Create Freespace and Transition Taskflows
  tesseract_planning::FreespaceTaskflowParams fparams;
  fparams.enable_simple_planner = goal->request.seed.empty();
  fparams.simple_plan_profiles = simple_plan_profiles_;
  fparams.simple_composite_profiles = simple_composite_profiles_;
  fparams.ompl_plan_profiles = ompl_plan_profiles_;
  fparams.trajopt_plan_profiles = trajopt_plan_profiles_;
  fparams.trajopt_composite_profiles = trajopt_composite_profiles_;
  tesseract_planning::GraphTaskflow::UPtr freespace_task = tesseract_planning::createFreespaceTaskflow(fparams);
  tesseract_planning::GraphTaskflow::UPtr transition_task = tesseract_planning::createFreespaceTaskflow(fparams);

  // Create Raster Taskflow
  tesseract_planning::CartesianTaskflowParams cparams;
  cparams.enable_simple_planner = goal->request.seed.empty();
  cparams.simple_plan_profiles = simple_plan_profiles_;
  cparams.simple_composite_profiles = simple_composite_profiles_;
  cparams.descartes_plan_profiles = descartes_plan_profiles_;
  cparams.trajopt_plan_profiles = trajopt_plan_profiles_;
  cparams.trajopt_composite_profiles = trajopt_composite_profiles_;
  tesseract_planning::GraphTaskflow::UPtr raster_task = tesseract_planning::createCartesianTaskflow(cparams);

  std::size_t nt = std::thread::hardware_concurrency();
  if (goal->request.num_threads != 0)
    nt = goal->request.num_threads;

  return std::make_shared<tesseract_planning::RasterDTProcessManager>(
      std::move(freespace_task), std::move(transition_task), std::move(raster_task), nt);
}

tesseract_planning::RasterWAADProcessManager::Ptr
TesseractPlanningServer::createRasterWAADProcessManager(const tesseract_msgs::GetMotionPlanGoalConstPtr& goal)
{
  // Create Freespace and Transition Taskflows
  tesseract_planning::FreespaceTaskflowParams fparams;
  fparams.enable_simple_planner = goal->request.seed.empty();
  fparams.simple_plan_profiles = simple_plan_profiles_;
  fparams.simple_composite_profiles = simple_composite_profiles_;
  fparams.ompl_plan_profiles = ompl_plan_profiles_;
  fparams.trajopt_plan_profiles = trajopt_plan_profiles_;
  fparams.trajopt_composite_profiles = trajopt_composite_profiles_;
  tesseract_planning::GraphTaskflow::UPtr freespace_task = tesseract_planning::createFreespaceTaskflow(fparams);
  tesseract_planning::GraphTaskflow::UPtr transition_task = tesseract_planning::createFreespaceTaskflow(fparams);

  // Create Raster Taskflow
  tesseract_planning::CartesianTaskflowParams cparams;
  cparams.enable_simple_planner = goal->request.seed.empty();
  cparams.simple_plan_profiles = simple_plan_profiles_;
  cparams.simple_composite_profiles = simple_composite_profiles_;
  cparams.descartes_plan_profiles = descartes_plan_profiles_;
  cparams.trajopt_plan_profiles = trajopt_plan_profiles_;
  cparams.trajopt_composite_profiles = trajopt_composite_profiles_;
  tesseract_planning::GraphTaskflow::UPtr raster_task = tesseract_planning::createCartesianTaskflow(cparams);

  std::size_t nt = std::thread::hardware_concurrency();
  if (goal->request.num_threads != 0)
    nt = goal->request.num_threads;

  return std::make_shared<tesseract_planning::RasterWAADProcessManager>(
      std::move(freespace_task), std::move(transition_task), std::move(raster_task), nt);
}

tesseract_planning::RasterWAADDTProcessManager::Ptr
TesseractPlanningServer::createRasterWAADDTProcessManager(const tesseract_msgs::GetMotionPlanGoalConstPtr& goal)
{
  // Create Freespace and Transition Taskflows
  tesseract_planning::FreespaceTaskflowParams fparams;
  fparams.enable_simple_planner = goal->request.seed.empty();
  fparams.simple_plan_profiles = simple_plan_profiles_;
  fparams.simple_composite_profiles = simple_composite_profiles_;
  fparams.ompl_plan_profiles = ompl_plan_profiles_;
  fparams.trajopt_plan_profiles = trajopt_plan_profiles_;
  fparams.trajopt_composite_profiles = trajopt_composite_profiles_;
  tesseract_planning::GraphTaskflow::UPtr freespace_task = tesseract_planning::createFreespaceTaskflow(fparams);
  tesseract_planning::GraphTaskflow::UPtr transition_task = tesseract_planning::createFreespaceTaskflow(fparams);

  // Create Raster Taskflow
  tesseract_planning::CartesianTaskflowParams cparams;
  cparams.enable_simple_planner = goal->request.seed.empty();
  cparams.simple_plan_profiles = simple_plan_profiles_;
  cparams.simple_composite_profiles = simple_composite_profiles_;
  cparams.descartes_plan_profiles = descartes_plan_profiles_;
  cparams.trajopt_plan_profiles = trajopt_plan_profiles_;
  cparams.trajopt_composite_profiles = trajopt_composite_profiles_;
  tesseract_planning::GraphTaskflow::UPtr raster_task = tesseract_planning::createCartesianTaskflow(cparams);

  std::size_t nt = std::thread::hardware_concurrency();
  if (goal->request.num_threads != 0)
    nt = goal->request.num_threads;

  return std::make_shared<tesseract_planning::RasterWAADDTProcessManager>(
      std::move(freespace_task), std::move(transition_task), std::move(raster_task), nt);
}

tesseract_planning::RasterOnlyGlobalProcessManager::Ptr
TesseractPlanningServer::createRasterOnlyGlobalProcessManager(const tesseract_msgs::GetMotionPlanGoalConstPtr& goal)
{
  tesseract_planning::DescartesTaskflowParams global_params;
  global_params.enable_simple_planner = goal->request.seed.empty();
  global_params.enable_post_contact_discrete_check = false;
  global_params.enable_post_contact_continuous_check = false;
  global_params.enable_time_parameterization = false;
  global_params.simple_plan_profiles = simple_plan_profiles_;
  global_params.simple_composite_profiles = simple_composite_profiles_;
  global_params.descartes_plan_profiles = descartes_plan_profiles_;
  tesseract_planning::GraphTaskflow::UPtr global_task = tesseract_planning::createDescartesTaskflow(global_params);

  tesseract_planning::FreespaceTaskflowParams transition_params;
  transition_params.type = tesseract_planning::FreespaceTaskflowType::TRAJOPT_FIRST;
  transition_params.enable_simple_planner = false;
  transition_params.simple_plan_profiles = simple_plan_profiles_;
  transition_params.simple_composite_profiles = simple_composite_profiles_;
  transition_params.ompl_plan_profiles = ompl_plan_profiles_;
  transition_params.trajopt_plan_profiles = trajopt_plan_profiles_;
  transition_params.trajopt_composite_profiles = trajopt_composite_profiles_;
  tesseract_planning::GraphTaskflow::UPtr transition_task =
      tesseract_planning::createFreespaceTaskflow(transition_params);

  tesseract_planning::TrajOptTaskflowParams raster_params;
  raster_params.enable_simple_planner = false;
  raster_params.simple_plan_profiles = simple_plan_profiles_;
  raster_params.simple_composite_profiles = simple_composite_profiles_;
  raster_params.trajopt_plan_profiles = trajopt_plan_profiles_;
  raster_params.trajopt_composite_profiles = trajopt_composite_profiles_;
  tesseract_planning::GraphTaskflow::UPtr raster_task = tesseract_planning::createTrajOptTaskflow(raster_params);

  std::size_t nt = std::thread::hardware_concurrency();
  if (goal->request.num_threads != 0)
    nt = goal->request.num_threads;

  return std::make_shared<tesseract_planning::RasterOnlyGlobalProcessManager>(
      std::move(global_task), std::move(transition_task), std::move(raster_task), nt);
}

tesseract_planning::RasterProcessManager::Ptr
TesseractPlanningServer::createRasterCTProcessManager(const tesseract_msgs::GetMotionPlanGoalConstPtr& goal)
{
  // Create Freespace and Transition Taskflows
  tesseract_planning::FreespaceTaskflowParams freespace_params;
  freespace_params.enable_simple_planner = goal->request.seed.empty();
  freespace_params.simple_plan_profiles = simple_plan_profiles_;
  freespace_params.simple_composite_profiles = simple_composite_profiles_;
  freespace_params.ompl_plan_profiles = ompl_plan_profiles_;
  freespace_params.trajopt_plan_profiles = trajopt_plan_profiles_;
  freespace_params.trajopt_composite_profiles = trajopt_composite_profiles_;
  tesseract_planning::GraphTaskflow::UPtr freespace_task =
      tesseract_planning::createFreespaceTaskflow(freespace_params);

  // Create Raster Taskflow
  tesseract_planning::CartesianTaskflowParams cartesian_params;
  cartesian_params.enable_simple_planner = goal->request.seed.empty();
  cartesian_params.simple_plan_profiles = simple_plan_profiles_;
  cartesian_params.simple_composite_profiles = simple_composite_profiles_;
  cartesian_params.descartes_plan_profiles = descartes_plan_profiles_;
  cartesian_params.trajopt_plan_profiles = trajopt_plan_profiles_;
  cartesian_params.trajopt_composite_profiles = trajopt_composite_profiles_;
  tesseract_planning::GraphTaskflow::UPtr raster_task = tesseract_planning::createCartesianTaskflow(cartesian_params);
  tesseract_planning::GraphTaskflow::UPtr transition_task =
      tesseract_planning::createCartesianTaskflow(cartesian_params);

  std::size_t nt = std::thread::hardware_concurrency();
  if (goal->request.num_threads != 0)
    nt = goal->request.num_threads;

  return std::make_shared<tesseract_planning::RasterProcessManager>(
      std::move(freespace_task), std::move(transition_task), std::move(raster_task), nt);
}

tesseract_planning::RasterOnlyProcessManager::Ptr
TesseractPlanningServer::createRasterOnlyCTProcessManager(const tesseract_msgs::GetMotionPlanGoalConstPtr& goal)
{
  // Create Transition and Raster Taskflow
  tesseract_planning::CartesianTaskflowParams cartesian_params;
  cartesian_params.enable_simple_planner = goal->request.seed.empty();
  cartesian_params.simple_plan_profiles = simple_plan_profiles_;
  cartesian_params.simple_composite_profiles = simple_composite_profiles_;
  cartesian_params.descartes_plan_profiles = descartes_plan_profiles_;
  cartesian_params.trajopt_plan_profiles = trajopt_plan_profiles_;
  cartesian_params.trajopt_composite_profiles = trajopt_composite_profiles_;
  tesseract_planning::GraphTaskflow::UPtr raster_task = tesseract_planning::createCartesianTaskflow(cartesian_params);
  tesseract_planning::GraphTaskflow::UPtr transition_task =
      tesseract_planning::createCartesianTaskflow(cartesian_params);

  std::size_t nt = std::thread::hardware_concurrency();
  if (goal->request.num_threads != 0)
    nt = goal->request.num_threads;

  return std::make_shared<tesseract_planning::RasterOnlyProcessManager>(
      std::move(transition_task), std::move(raster_task), nt);
}

tesseract_planning::RasterDTProcessManager::Ptr
TesseractPlanningServer::createRasterCTDTProcessManager(const tesseract_msgs::GetMotionPlanGoalConstPtr& goal)
{
  // Create Freespace and Transition Taskflows
  tesseract_planning::FreespaceTaskflowParams freespace_params;
  freespace_params.enable_simple_planner = goal->request.seed.empty();
  freespace_params.simple_plan_profiles = simple_plan_profiles_;
  freespace_params.simple_composite_profiles = simple_composite_profiles_;
  freespace_params.ompl_plan_profiles = ompl_plan_profiles_;
  freespace_params.trajopt_plan_profiles = trajopt_plan_profiles_;
  freespace_params.trajopt_composite_profiles = trajopt_composite_profiles_;
  tesseract_planning::GraphTaskflow::UPtr freespace_task =
      tesseract_planning::createFreespaceTaskflow(freespace_params);

  // Create Raster Taskflow
  tesseract_planning::CartesianTaskflowParams cartesian_params;
  cartesian_params.enable_simple_planner = goal->request.seed.empty();
  cartesian_params.simple_plan_profiles = simple_plan_profiles_;
  cartesian_params.simple_composite_profiles = simple_composite_profiles_;
  cartesian_params.descartes_plan_profiles = descartes_plan_profiles_;
  cartesian_params.trajopt_plan_profiles = trajopt_plan_profiles_;
  cartesian_params.trajopt_composite_profiles = trajopt_composite_profiles_;
  tesseract_planning::GraphTaskflow::UPtr raster_task = tesseract_planning::createCartesianTaskflow(cartesian_params);
  tesseract_planning::GraphTaskflow::UPtr transition_task =
      tesseract_planning::createCartesianTaskflow(cartesian_params);

  std::size_t nt = std::thread::hardware_concurrency();
  if (goal->request.num_threads != 0)
    nt = goal->request.num_threads;

  return std::make_shared<tesseract_planning::RasterDTProcessManager>(
      std::move(freespace_task), std::move(transition_task), std::move(raster_task), nt);
}

tesseract_planning::RasterWAADProcessManager::Ptr
TesseractPlanningServer::createRasterCTWAADProcessManager(const tesseract_msgs::GetMotionPlanGoalConstPtr& goal)
{
  // Create Freespace and Transition Taskflows
  tesseract_planning::FreespaceTaskflowParams freespace_params;
  freespace_params.enable_simple_planner = goal->request.seed.empty();
  freespace_params.simple_plan_profiles = simple_plan_profiles_;
  freespace_params.simple_composite_profiles = simple_composite_profiles_;
  freespace_params.ompl_plan_profiles = ompl_plan_profiles_;
  freespace_params.trajopt_plan_profiles = trajopt_plan_profiles_;
  freespace_params.trajopt_composite_profiles = trajopt_composite_profiles_;
  tesseract_planning::GraphTaskflow::UPtr freespace_task =
      tesseract_planning::createFreespaceTaskflow(freespace_params);

  // Create Raster Taskflow
  tesseract_planning::CartesianTaskflowParams cartesian_params;
  cartesian_params.enable_simple_planner = goal->request.seed.empty();
  cartesian_params.simple_plan_profiles = simple_plan_profiles_;
  cartesian_params.simple_composite_profiles = simple_composite_profiles_;
  cartesian_params.descartes_plan_profiles = descartes_plan_profiles_;
  cartesian_params.trajopt_plan_profiles = trajopt_plan_profiles_;
  cartesian_params.trajopt_composite_profiles = trajopt_composite_profiles_;
  tesseract_planning::GraphTaskflow::UPtr raster_task = tesseract_planning::createCartesianTaskflow(cartesian_params);
  tesseract_planning::GraphTaskflow::UPtr transition_task =
      tesseract_planning::createCartesianTaskflow(cartesian_params);

  std::size_t nt = std::thread::hardware_concurrency();
  if (goal->request.num_threads != 0)
    nt = goal->request.num_threads;

  return std::make_shared<tesseract_planning::RasterWAADProcessManager>(
      std::move(freespace_task), std::move(transition_task), std::move(raster_task), nt);
}

tesseract_planning::RasterWAADDTProcessManager::Ptr
TesseractPlanningServer::createRasterCTWAADDTProcessManager(const tesseract_msgs::GetMotionPlanGoalConstPtr& goal)
{
  // Create Freespace and Transition Taskflows
  tesseract_planning::FreespaceTaskflowParams freespace_params;
  freespace_params.enable_simple_planner = goal->request.seed.empty();
  freespace_params.simple_plan_profiles = simple_plan_profiles_;
  freespace_params.simple_composite_profiles = simple_composite_profiles_;
  freespace_params.ompl_plan_profiles = ompl_plan_profiles_;
  freespace_params.trajopt_plan_profiles = trajopt_plan_profiles_;
  freespace_params.trajopt_composite_profiles = trajopt_composite_profiles_;
  tesseract_planning::GraphTaskflow::UPtr freespace_task =
      tesseract_planning::createFreespaceTaskflow(freespace_params);

  // Create Raster Taskflow
  tesseract_planning::CartesianTaskflowParams cartesian_params;
  cartesian_params.enable_simple_planner = goal->request.seed.empty();
  cartesian_params.simple_plan_profiles = simple_plan_profiles_;
  cartesian_params.simple_composite_profiles = simple_composite_profiles_;
  cartesian_params.descartes_plan_profiles = descartes_plan_profiles_;
  cartesian_params.trajopt_plan_profiles = trajopt_plan_profiles_;
  cartesian_params.trajopt_composite_profiles = trajopt_composite_profiles_;
  tesseract_planning::GraphTaskflow::UPtr raster_task = tesseract_planning::createCartesianTaskflow(cartesian_params);
  tesseract_planning::GraphTaskflow::UPtr transition_task =
      tesseract_planning::createCartesianTaskflow(cartesian_params);

  std::size_t nt = std::thread::hardware_concurrency();
  if (goal->request.num_threads != 0)
    nt = goal->request.num_threads;

  return std::make_shared<tesseract_planning::RasterWAADDTProcessManager>(
      std::move(freespace_task), std::move(transition_task), std::move(raster_task), nt);
}

tesseract_planning::RasterGlobalProcessManager::Ptr
TesseractPlanningServer::createRasterGlobalCTProcessManager(const tesseract_msgs::GetMotionPlanGoalConstPtr& goal)
{
  tesseract_planning::DescartesTaskflowParams global_params;
  global_params.enable_simple_planner = goal->request.seed.empty();
  global_params.enable_post_contact_discrete_check = false;
  global_params.enable_post_contact_continuous_check = false;
  global_params.enable_time_parameterization = false;
  global_params.simple_plan_profiles = simple_plan_profiles_;
  global_params.simple_composite_profiles = simple_composite_profiles_;
  global_params.descartes_plan_profiles = descartes_plan_profiles_;
  tesseract_planning::GraphTaskflow::UPtr global_task = tesseract_planning::createDescartesTaskflow(global_params);

  tesseract_planning::FreespaceTaskflowParams freespace_params;
  freespace_params.type = tesseract_planning::FreespaceTaskflowType::TRAJOPT_FIRST;
  freespace_params.enable_simple_planner = false;
  freespace_params.simple_plan_profiles = simple_plan_profiles_;
  freespace_params.simple_composite_profiles = simple_composite_profiles_;
  freespace_params.ompl_plan_profiles = ompl_plan_profiles_;
  freespace_params.trajopt_plan_profiles = trajopt_plan_profiles_;
  freespace_params.trajopt_composite_profiles = trajopt_composite_profiles_;
  tesseract_planning::GraphTaskflow::UPtr freespace_task =
      tesseract_planning::createFreespaceTaskflow(freespace_params);

  tesseract_planning::TrajOptTaskflowParams raster_params;
  raster_params.enable_simple_planner = false;
  raster_params.simple_plan_profiles = simple_plan_profiles_;
  raster_params.simple_composite_profiles = simple_composite_profiles_;
  raster_params.trajopt_plan_profiles = trajopt_plan_profiles_;
  raster_params.trajopt_composite_profiles = trajopt_composite_profiles_;
  tesseract_planning::GraphTaskflow::UPtr raster_task = tesseract_planning::createTrajOptTaskflow(raster_params);
  tesseract_planning::GraphTaskflow::UPtr transition_task = tesseract_planning::createTrajOptTaskflow(raster_params);

  std::size_t nt = std::thread::hardware_concurrency();
  if (goal->request.num_threads != 0)
    nt = goal->request.num_threads;

  return std::make_shared<tesseract_planning::RasterGlobalProcessManager>(
      std::move(global_task), std::move(freespace_task), std::move(transition_task), std::move(raster_task), nt);
}

tesseract_planning::RasterOnlyGlobalProcessManager::Ptr
TesseractPlanningServer::createRasterOnlyGlobalCTProcessManager(const tesseract_msgs::GetMotionPlanGoalConstPtr& goal)
{
  tesseract_planning::DescartesTaskflowParams global_params;
  global_params.enable_simple_planner = goal->request.seed.empty();
  global_params.enable_post_contact_discrete_check = false;
  global_params.enable_post_contact_continuous_check = false;
  global_params.enable_time_parameterization = false;
  global_params.simple_plan_profiles = simple_plan_profiles_;
  global_params.simple_composite_profiles = simple_composite_profiles_;
  global_params.descartes_plan_profiles = descartes_plan_profiles_;
  tesseract_planning::GraphTaskflow::UPtr global_task = tesseract_planning::createDescartesTaskflow(global_params);

  tesseract_planning::TrajOptTaskflowParams raster_params;
  raster_params.enable_simple_planner = false;
  raster_params.simple_plan_profiles = simple_plan_profiles_;
  raster_params.simple_composite_profiles = simple_composite_profiles_;
  raster_params.trajopt_plan_profiles = trajopt_plan_profiles_;
  raster_params.trajopt_composite_profiles = trajopt_composite_profiles_;
  tesseract_planning::GraphTaskflow::UPtr raster_task = tesseract_planning::createTrajOptTaskflow(raster_params);
  tesseract_planning::GraphTaskflow::UPtr transition_task = tesseract_planning::createTrajOptTaskflow(raster_params);

  std::size_t nt = std::thread::hardware_concurrency();
  if (goal->request.num_threads != 0)
    nt = goal->request.num_threads;

  return std::make_shared<tesseract_planning::RasterOnlyGlobalProcessManager>(
      std::move(global_task), std::move(transition_task), std::move(raster_task), nt);
}

}  // namespace tesseract_planning_server
