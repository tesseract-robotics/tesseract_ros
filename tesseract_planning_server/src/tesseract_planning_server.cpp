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
#include <tesseract_task_composer/planning/planning_task_composer_problem.h>

#ifdef TESSERACT_PLANNING_SERVER_HAS_DESCARTES
#include <tesseract_motion_planners/descartes/profile/descartes_profile.h>
#include <tesseract_motion_planners/descartes/profile/descartes_default_plan_profile.h>
#endif

#ifdef TESSERACT_PLANNING_SERVER_HAS_TRAJOPT
#include <tesseract_motion_planners/trajopt/profile/trajopt_profile.h>
#include <tesseract_motion_planners/trajopt/profile/trajopt_default_composite_profile.h>
#include <tesseract_motion_planners/trajopt/profile/trajopt_default_plan_profile.h>
#include <tesseract_motion_planners/trajopt/profile/trajopt_default_solver_profile.h>
#endif

#ifdef TESSERACT_PLANNING_SERVER_HAS_TRAJOPT_IFOPT
#include <tesseract_motion_planners/trajopt_ifopt/profile/trajopt_ifopt_profile.h>
#include <tesseract_motion_planners/trajopt_ifopt/profile/trajopt_ifopt_default_composite_profile.h>
#include <tesseract_motion_planners/trajopt_ifopt/profile/trajopt_ifopt_default_plan_profile.h>
#endif

#ifdef TESSERACT_PLANNING_SERVER_HAS_OMPL
#include <tesseract_motion_planners/ompl/profile/ompl_profile.h>
#include <tesseract_motion_planners/ompl/profile/ompl_default_plan_profile.h>
#endif

#include <tesseract_motion_planners/simple/profile/simple_planner_profile.h>
#include <tesseract_motion_planners/simple/profile/simple_planner_lvs_no_ik_plan_profile.h>

#include <tesseract_command_language/poly/instruction_poly.h>

#include <tesseract_monitoring/environment_monitor.h>
#include <tesseract_rosutils/utils.h>
#include <tesseract_common/serialization.h>
#include <tesseract_common/timer.h>

using tesseract_common::Serialization;
using tesseract_rosutils::processMsg;

static const std::string TRAJOPT_DEFAULT_NAMESPACE = "TrajOptMotionPlannerTask";
static const std::string TRAJOPT_IFOPT_DEFAULT_NAMESPACE = "TrajOptIfoptMotionPlannerTask";
static const std::string OMPL_DEFAULT_NAMESPACE = "OMPLMotionPlannerTask";
static const std::string DESCARTES_DEFAULT_NAMESPACE = "DescartesMotionPlannerTask";
static const std::string SIMPLE_DEFAULT_NAMESPACE = "SimpleMotionPlannerTask";

namespace tesseract_planning_server
{
const std::string TesseractPlanningServer::DEFAULT_GET_MOTION_PLAN_ACTION = "tesseract_get_motion_plan";

TesseractPlanningServer::TesseractPlanningServer(const std::string& robot_description,
                                                 std::string input_key,
                                                 std::string output_key,
                                                 std::string name)
  : nh_("~")
  , monitor_(std::make_shared<tesseract_monitoring::ROSEnvironmentMonitor>(robot_description, name))
  , environment_cache_(std::make_shared<tesseract_environment::DefaultEnvironmentCache>(monitor_->getEnvironment()))
  , profiles_(std::make_shared<tesseract_planning::ProfileDictionary>())
  , planning_server_(std::make_unique<tesseract_planning::TaskComposerServer>())
  , input_key_(std::move(input_key))
  , output_key_(std::move(output_key))
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
                                                 std::string input_key,
                                                 std::string output_key,
                                                 std::string name)
  : nh_("~")
  , monitor_(std::make_shared<tesseract_monitoring::ROSEnvironmentMonitor>(std::move(env), name))
  , environment_cache_(std::make_shared<tesseract_environment::DefaultEnvironmentCache>(monitor_->getEnvironment()))
  , profiles_(std::make_shared<tesseract_planning::ProfileDictionary>())
  , planning_server_(std::make_unique<tesseract_planning::TaskComposerServer>())
  , input_key_(std::move(input_key))
  , output_key_(std::move(output_key))
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
  monitor_->environment().addFindTCPOffsetCallback(
      std::bind(&TesseractPlanningServer::tfFindTCPOffset, this, std::placeholders::_1));
}

tesseract_environment::EnvironmentMonitor& TesseractPlanningServer::getEnvironmentMonitor() { return *monitor_; }
const tesseract_environment::EnvironmentMonitor& TesseractPlanningServer::getEnvironmentMonitor() const
{
  return *monitor_;
}

tesseract_planning::TaskComposerServer& TesseractPlanningServer::getTaskComposerServer() { return *planning_server_; }
const tesseract_planning::TaskComposerServer& TesseractPlanningServer::getTaskComposerServer() const
{
  return *planning_server_;
}

tesseract_environment::EnvironmentCache& TesseractPlanningServer::getEnvironmentCache() { return *environment_cache_; }
const tesseract_environment::EnvironmentCache& TesseractPlanningServer::getEnvironmentCache() const
{
  return *environment_cache_;
}

tesseract_planning::ProfileDictionary& TesseractPlanningServer::getProfileDictionary() { return *profiles_; }
const tesseract_planning::ProfileDictionary& TesseractPlanningServer::getProfileDictionary() const
{
  return *profiles_;
}

void TesseractPlanningServer::onMotionPlanningCallback(const tesseract_msgs::GetMotionPlanGoalConstPtr& goal)
{
  ROS_INFO("Tesseract Planning Server Received Request!");
  tesseract_msgs::GetMotionPlanResult result;

  // Check if process planner exist
  if (!planning_server_->hasTask(goal->request.name))
  {
    result.response.successful = false;
    std::ostringstream oss;
    oss << "Requested task '" << goal->request.name << "' is not supported!" << std::endl;
    oss << "   Available Tasks:" << std::endl;
    for (const auto& planner : planning_server_->getAvailableTasks())
      oss << "      - " << planner << std::endl;
    ROS_ERROR_STREAM(oss.str());
    motion_plan_server_.setSucceeded(result);
    return;
  }

  std::string executor_name = goal->request.executor;
  std::vector<std::string> available_executors = planning_server_->getAvailableExecutors();
  if (executor_name.empty() && !available_executors.empty())
    executor_name = planning_server_->getAvailableExecutors().front();

  // Check if executor exists
  if (!planning_server_->hasExecutor(executor_name))
  {
    result.response.successful = false;
    std::ostringstream oss;
    oss << "Requested executor '" << executor_name << "' is not supported!" << std::endl;
    oss << "   Available Executors:" << std::endl;
    for (const auto& executor : available_executors)
      oss << "      - " << executor << std::endl;
    ROS_ERROR_STREAM(oss.str());
    motion_plan_server_.setSucceeded(result);
    return;
  }

  auto problem = std::make_unique<tesseract_planning::PlanningTaskComposerProblem>(goal->request.name);
  auto data_storage = std::make_unique<tesseract_planning::TaskComposerDataStorage>();

  try
  {
    auto ci = Serialization::fromArchiveStringXML<tesseract_planning::InstructionPoly>(goal->request.instructions)
                  .as<tesseract_planning::CompositeInstruction>();
    data_storage->setData(input_key_, ci);
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

  tesseract_environment::Environment::Ptr env = environment_cache_->getCachedEnvironment();

  tesseract_scene_graph::SceneState env_state;
  tesseract_rosutils::fromMsg(env_state.joints, goal->request.environment_state.joint_state);

  env->applyCommands(tesseract_rosutils::fromMsg(goal->request.commands));
  env->setState(env_state.joints);

  problem->env = env;
  //  process_request.save_io = goal->request.save_io;
  problem->profiles = profiles_;
  problem->move_profile_remapping = tesseract_rosutils::fromMsg(goal->request.move_profile_remapping);
  problem->composite_profile_remapping = tesseract_rosutils::fromMsg(goal->request.composite_profile_remapping);
  problem->dotgraph = goal->request.dotgraph;

  // Store the initial state in the response for publishing trajectories
  tesseract_scene_graph::SceneState initial_state = env->getState();
  tesseract_rosutils::toMsg(result.response.initial_state, initial_state.joints);

  // Solve
  tesseract_common::Timer timer;
  timer.start();
  tesseract_planning::TaskComposerFuture::UPtr plan_future =
      planning_server_->run(std::move(problem), std::move(data_storage), executor_name);
  plan_future->wait();  // Wait for results
  timer.stop();

  // Generate DOT Graph if requested
  if (goal->request.dotgraph)
  {
    try
    {
      // Get Task
      const tesseract_planning::TaskComposerNode& task = planning_server_->getTask(plan_future->context->problem->name);

      // Save dot graph
      std::stringstream dotgraph;
      task.dump(dotgraph, nullptr, plan_future->context->task_infos.getInfoMap());
      result.response.dotgraph = dotgraph.str();
    }
    catch (const std::exception& e)
    {
      std::ostringstream oss;
      oss << "Failed to generated DOT Graph: '" << e.what() << "'!" << std::endl;
      ROS_ERROR_STREAM(oss.str());
    }
  }

  try
  {
    tesseract_common::AnyPoly results = plan_future->context->data_storage->getData(output_key_);
    result.response.results = Serialization::toArchiveStringXML<tesseract_planning::InstructionPoly>(
        results.as<tesseract_planning::CompositeInstruction>());
  }
  catch (const std::exception& e)
  {
    result.response.successful = false;
    std::ostringstream oss;
    oss << "Failed to get output results from task with error: '" << e.what() << "'!" << std::endl;
    ROS_ERROR_STREAM(oss.str());
    motion_plan_server_.setSucceeded(result);
    return;
  }

  result.response.successful = plan_future->context->isSuccessful();
  plan_future->clear();

  ROS_INFO("Tesseract Planning Server Finished Request in %f seconds!", timer.elapsedSeconds());
  motion_plan_server_.setSucceeded(result);
}

void TesseractPlanningServer::loadDefaultPlannerProfiles()
{
  // Add Simple Default Profiles
  profiles_->addProfile<tesseract_planning::SimplePlannerPlanProfile>(
      SIMPLE_DEFAULT_NAMESPACE,
      tesseract_planning::DEFAULT_PROFILE_KEY,
      std::make_shared<tesseract_planning::SimplePlannerLVSNoIKPlanProfile>());

  // Add TrajOpt Default Profiles
#ifdef TESSERACT_PLANNING_SERVER_HAS_TRAJOPT
  profiles_->addProfile<tesseract_planning::TrajOptPlanProfile>(
      TRAJOPT_DEFAULT_NAMESPACE,
      tesseract_planning::DEFAULT_PROFILE_KEY,
      std::make_shared<tesseract_planning::TrajOptDefaultPlanProfile>());
  profiles_->addProfile<tesseract_planning::TrajOptCompositeProfile>(
      TRAJOPT_DEFAULT_NAMESPACE,
      tesseract_planning::DEFAULT_PROFILE_KEY,
      std::make_shared<tesseract_planning::TrajOptDefaultCompositeProfile>());
  profiles_->addProfile<tesseract_planning::TrajOptSolverProfile>(
      TRAJOPT_DEFAULT_NAMESPACE,
      tesseract_planning::DEFAULT_PROFILE_KEY,
      std::make_shared<tesseract_planning::TrajOptDefaultSolverProfile>());
#endif

  // Add TrajOpt IFOPT Default Profiles
#ifdef TESSERACT_PLANNING_SERVER_HAS_TRAJOPT_IFOPT
  profiles_->addProfile<tesseract_planning::TrajOptIfoptPlanProfile>(
      TRAJOPT_IFOPT_DEFAULT_NAMESPACE,
      tesseract_planning::DEFAULT_PROFILE_KEY,
      std::make_shared<tesseract_planning::TrajOptIfoptDefaultPlanProfile>());
  profiles_->addProfile<tesseract_planning::TrajOptIfoptCompositeProfile>(
      TRAJOPT_IFOPT_DEFAULT_NAMESPACE,
      tesseract_planning::DEFAULT_PROFILE_KEY,
      std::make_shared<tesseract_planning::TrajOptIfoptDefaultCompositeProfile>());
#endif

  // Add Descartes Default Profiles
#ifdef TESSERACT_PLANNING_SERVER_HAS_DESCARTES
  profiles_->addProfile<tesseract_planning::DescartesPlanProfile<double>>(
      DESCARTES_DEFAULT_NAMESPACE,
      tesseract_planning::DEFAULT_PROFILE_KEY,
      std::make_shared<tesseract_planning::DescartesDefaultPlanProfile<double>>());
#endif

  // Add OMPL Default Profiles
#ifdef TESSERACT_PLANNING_SERVER_HAS_OMPL
  profiles_->addProfile<tesseract_planning::OMPLPlanProfile>(
      OMPL_DEFAULT_NAMESPACE,
      tesseract_planning::DEFAULT_PROFILE_KEY,
      std::make_shared<tesseract_planning::OMPLDefaultPlanProfile>());
#endif
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
