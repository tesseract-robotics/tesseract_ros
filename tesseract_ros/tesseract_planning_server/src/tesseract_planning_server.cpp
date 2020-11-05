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

#include <tesseract_process_managers/process_managers/raster_process_manager.h>
#include <tesseract_process_managers/process_managers/raster_global_process_manager.h>
#include <tesseract_process_managers/process_managers/raster_only_process_manager.h>
#include <tesseract_process_managers/process_managers/raster_only_global_process_manager.h>
#include <tesseract_process_managers/process_managers/raster_dt_process_manager.h>
#include <tesseract_process_managers/process_managers/raster_waad_process_manager.h>
#include <tesseract_process_managers/process_managers/raster_waad_dt_process_manager.h>

#include <tesseract_process_managers/process_managers/simple_process_manager.h>
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

  tesseract_planning::GraphTaskflow::UPtr task = nullptr;
  if (goal->request.name == goal->TRAJOPT_PLANNER_NAME)
  {
    tesseract_planning::TrajOptTaskflowParams params;
    params.enable_simple_planner = goal->request.seed.empty();
    params.simple_plan_profiles = simple_plan_profiles_;
    params.simple_composite_profiles = simple_composite_profiles_;
    params.trajopt_plan_profiles = trajopt_plan_profiles_;
    params.trajopt_composite_profiles = trajopt_composite_profiles_;
    task = tesseract_planning::createTrajOptTaskflow(params);
  }
  else if (goal->request.name == goal->OMPL_PLANNER_NAME)
  {
    tesseract_planning::OMPLTaskflowParams params;
    params.enable_simple_planner = goal->request.seed.empty();
    params.simple_plan_profiles = simple_plan_profiles_;
    params.simple_composite_profiles = simple_composite_profiles_;
    params.ompl_plan_profiles = ompl_plan_profiles_;
    task = tesseract_planning::createOMPLTaskflow(params);
  }
  else if (goal->request.name == goal->DESCARTES_PLANNER_NAME)
  {
    tesseract_planning::DescartesTaskflowParams params;
    params.enable_simple_planner = goal->request.seed.empty();
    params.simple_plan_profiles = simple_plan_profiles_;
    params.simple_composite_profiles = simple_composite_profiles_;
    params.descartes_plan_profiles = descartes_plan_profiles_;
    task = tesseract_planning::createDescartesTaskflow(params);
  }
  else if (goal->request.name == goal->CARTESIAN_PLANNER_NAME)
  {
    tesseract_planning::CartesianTaskflowParams params;
    params.enable_simple_planner = goal->request.seed.empty();
    params.simple_plan_profiles = simple_plan_profiles_;
    params.simple_composite_profiles = simple_composite_profiles_;
    params.descartes_plan_profiles = descartes_plan_profiles_;
    params.trajopt_plan_profiles = trajopt_plan_profiles_;
    params.trajopt_composite_profiles = trajopt_composite_profiles_;
    task = tesseract_planning::createCartesianTaskflow(params);
  }
  else if (goal->request.name == goal->FREESPACE_PLANNER_NAME)
  {
    tesseract_planning::FreespaceTaskflowParams params;
    params.enable_simple_planner = goal->request.seed.empty();
    params.simple_plan_profiles = simple_plan_profiles_;
    params.simple_composite_profiles = simple_composite_profiles_;
    params.ompl_plan_profiles = ompl_plan_profiles_;
    params.trajopt_plan_profiles = trajopt_plan_profiles_;
    params.trajopt_composite_profiles = trajopt_composite_profiles_;
    task = tesseract_planning::createFreespaceTaskflow(params);
  }

  tesseract_msgs::GetMotionPlanResult result;
  tesseract_planning::ManipulatorInfo mi = composite_program->getManipulatorInfo();
  std::size_t nt = std::thread::hardware_concurrency();
  if (goal->request.num_threads != 0)
    nt = goal->request.num_threads;

  tesseract_planning::ProcessManager::Ptr pm;
  if (task != nullptr)
  {
    pm = std::make_shared<tesseract_planning::SimpleProcessManager>(std::move(task), nt);
  }
  else
  {
    tesseract_planning::GraphTaskflow::UPtr gtask = nullptr;
    tesseract_planning::GraphTaskflow::UPtr gtaskf = nullptr;
    tesseract_planning::GraphTaskflow::UPtr gtaskc = nullptr;
    tesseract_planning::GraphTaskflow::UPtr gtasktf = nullptr;
    tesseract_planning::GraphTaskflow::UPtr gtasktc = nullptr;

    tesseract_planning::GraphTaskflow::UPtr ctask1 = nullptr;
    tesseract_planning::GraphTaskflow::UPtr ftask1 = nullptr;
    tesseract_planning::GraphTaskflow::UPtr task2 = nullptr;

    tesseract_planning::FreespaceTaskflowParams fparams;
    fparams.enable_simple_planner = goal->request.seed.empty();
    fparams.simple_plan_profiles = simple_plan_profiles_;
    fparams.simple_composite_profiles = simple_composite_profiles_;
    fparams.ompl_plan_profiles = ompl_plan_profiles_;
    fparams.trajopt_plan_profiles = trajopt_plan_profiles_;
    fparams.trajopt_composite_profiles = trajopt_composite_profiles_;

    tesseract_planning::CartesianTaskflowParams cparams;
    cparams.enable_simple_planner = goal->request.seed.empty();
    cparams.simple_plan_profiles = simple_plan_profiles_;
    cparams.simple_composite_profiles = simple_composite_profiles_;
    cparams.descartes_plan_profiles = descartes_plan_profiles_;
    cparams.trajopt_plan_profiles = trajopt_plan_profiles_;
    cparams.trajopt_composite_profiles = trajopt_composite_profiles_;

    task = tesseract_planning::createFreespaceTaskflow(fparams);
    ftask1 = tesseract_planning::createFreespaceTaskflow(fparams);
    ctask1 = tesseract_planning::createCartesianTaskflow(cparams);
    task2 = tesseract_planning::createCartesianTaskflow(cparams);

    tesseract_planning::DescartesTaskflowParams gdparams;
    gdparams.enable_simple_planner = goal->request.seed.empty();
    gdparams.enable_post_contact_discrete_check = false;
    gdparams.enable_post_contact_continuous_check = false;
    gdparams.enable_time_parameterization = false;
    gdparams.simple_plan_profiles = simple_plan_profiles_;
    gdparams.simple_composite_profiles = simple_composite_profiles_;
    gdparams.descartes_plan_profiles = descartes_plan_profiles_;
    gtask = tesseract_planning::createDescartesTaskflow(gdparams);

    tesseract_planning::FreespaceTaskflowParams gfparams;
    gfparams.type = tesseract_planning::FreespaceTaskflowType::TRAJOPT_FIRST;
    gfparams.enable_simple_planner = false;
    gfparams.simple_plan_profiles = simple_plan_profiles_;
    gfparams.simple_composite_profiles = simple_composite_profiles_;
    gfparams.ompl_plan_profiles = ompl_plan_profiles_;
    gfparams.trajopt_plan_profiles = trajopt_plan_profiles_;
    gfparams.trajopt_composite_profiles = trajopt_composite_profiles_;
    gtaskf = tesseract_planning::createFreespaceTaskflow(gfparams);
    gtasktf = tesseract_planning::createFreespaceTaskflow(gfparams);

    tesseract_planning::TrajOptTaskflowParams gcparams;
    gcparams.enable_simple_planner = false;
    gcparams.simple_plan_profiles = simple_plan_profiles_;
    gcparams.simple_composite_profiles = simple_composite_profiles_;
    gcparams.trajopt_plan_profiles = trajopt_plan_profiles_;
    gcparams.trajopt_composite_profiles = trajopt_composite_profiles_;
    gtaskc = tesseract_planning::createTrajOptTaskflow(gcparams);

    tesseract_planning::TrajOptTaskflowParams gtcparams;
    gtcparams.enable_simple_planner = false;
    gtcparams.simple_plan_profiles = simple_plan_profiles_;
    gtcparams.simple_composite_profiles = simple_composite_profiles_;
    gtcparams.trajopt_plan_profiles = trajopt_plan_profiles_;
    gtcparams.trajopt_composite_profiles = trajopt_composite_profiles_;
    gtasktc = tesseract_planning::createTrajOptTaskflow(gtcparams);

    if (goal->request.name == goal->RASTER_FT_PLANNER_NAME)
    {
      pm = std::make_shared<tesseract_planning::RasterProcessManager>(
          std::move(task), std::move(ftask1), std::move(task2), nt);
    }
    else if (goal->request.name == goal->RASTER_O_FT_PLANNER_NAME)
    {
      pm = std::make_shared<tesseract_planning::RasterOnlyProcessManager>(std::move(ftask1), std::move(task2), nt);
    }
    else if (goal->request.name == goal->RASTER_G_FT_PLANNER_NAME)
    {
      pm = std::make_shared<tesseract_planning::RasterGlobalProcessManager>(
          std::move(gtask), std::move(gtaskf), std::move(gtasktf), std::move(gtaskc), nt);
    }
    else if (goal->request.name == goal->RASTER_FT_DT_PLANNER_NAME)
    {
      pm = std::make_shared<tesseract_planning::RasterDTProcessManager>(
          std::move(task), std::move(ftask1), std::move(task2), nt);
    }
    else if (goal->request.name == goal->RASTER_FT_WAAD_PLANNER_NAME)
    {
      pm = std::make_shared<tesseract_planning::RasterWAADProcessManager>(
          std::move(task), std::move(ftask1), std::move(task2), nt);
    }
    else if (goal->request.name == goal->RASTER_FT_WAAD_DT_PLANNER_NAME)
    {
      pm = std::make_shared<tesseract_planning::RasterWAADDTProcessManager>(
          std::move(task), std::move(ftask1), std::move(task2), nt);
    }
    else if (goal->request.name == goal->RASTER_O_G_FT_PLANNER_NAME)
    {
      pm = std::make_shared<tesseract_planning::RasterOnlyGlobalProcessManager>(
          std::move(gtask), std::move(gtasktf), std::move(gtaskc), nt);
    }
    else if (goal->request.name == goal->RASTER_CT_PLANNER_NAME)
    {
      pm = std::make_shared<tesseract_planning::RasterProcessManager>(
          std::move(task), std::move(ctask1), std::move(task2), nt);
    }
    else if (goal->request.name == goal->RASTER_O_CT_PLANNER_NAME)
    {
      pm = std::make_shared<tesseract_planning::RasterOnlyProcessManager>(std::move(ctask1), std::move(task2), nt);
    }
    else if (goal->request.name == goal->RASTER_CT_DT_PLANNER_NAME)
    {
      pm = std::make_shared<tesseract_planning::RasterDTProcessManager>(
          std::move(task), std::move(ctask1), std::move(task2), nt);
    }
    else if (goal->request.name == goal->RASTER_CT_WAAD_PLANNER_NAME)
    {
      pm = std::make_shared<tesseract_planning::RasterWAADProcessManager>(
          std::move(task), std::move(ctask1), std::move(task2), nt);
    }
    else if (goal->request.name == goal->RASTER_CT_WAAD_DT_PLANNER_NAME)
    {
      pm = std::make_shared<tesseract_planning::RasterWAADDTProcessManager>(
          std::move(task), std::move(ctask1), std::move(task2), nt);
    }
    else if (goal->request.name == goal->RASTER_G_CT_PLANNER_NAME)
    {
      pm = std::make_shared<tesseract_planning::RasterGlobalProcessManager>(
          std::move(gtask), std::move(gtaskf), std::move(gtasktc), std::move(gtaskc), nt);
    }
    else if (goal->request.name == goal->RASTER_O_G_CT_PLANNER_NAME)
    {
      pm = std::make_shared<tesseract_planning::RasterOnlyGlobalProcessManager>(
          std::move(gtask), std::move(gtasktc), std::move(gtaskc), nt);
    }
    else
    {
      result.response.successful = false;
      ROS_ERROR("Requested motion planner is not supported!");
      motion_plan_server_.setSucceeded(result);
      return;
    }
  }

  assert(pm != nullptr);
  tesseract::Tesseract::Ptr tc = getCachedTesseract();
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
      environment_.getTesseract()->getManipulatorManager()->getFwdKinematicSolver(manip_info.manipulator);
  if (composite_mi_fwd_kin == nullptr)
    throw std::runtime_error("tfFindTCP: Manipulator '" + manip_info.manipulator + "' does not exist!");

  const std::string& tip_link = composite_mi_fwd_kin->getTipLinkName();
  const std::string& tcp_name = manip_info.tcp.getString();

  auto tcp_msg = tf_buffer_->lookupTransform(tip_link, tcp_name, ros::Time(0), ros::Duration(2));
  return tf2::transformToEigen(tcp_msg.transform);
}

}  // namespace tesseract_planning_server
