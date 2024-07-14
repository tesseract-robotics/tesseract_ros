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
#include <ros/node_handle.h>
#include <actionlib/server/simple_action_server.h>
#include <tf2_eigen/tf2_eigen.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <console_bridge/console.h>
TESSERACT_COMMON_IGNORE_WARNINGS_POP

#include <tesseract_planning_server/tesseract_planning_server.h>
#include <tesseract_task_composer/core/task_composer_server.h>
#include <tesseract_task_composer/core/task_composer_context.h>
#include <tesseract_task_composer/core/task_composer_future.h>
#include <tesseract_task_composer/core/task_composer_node.h>
#include <tesseract_task_composer/core/task_composer_data_storage.h>

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

#include <tesseract_rosutils/utils.h>
#include <tesseract_monitoring/environment_monitor.h>
#include <tesseract_msgs/GetMotionPlanAction.h>

#include <tesseract_common/serialization.h>
#include <tesseract_common/any_poly.h>
#include <tesseract_common/timer.h>
#include <tesseract_environment/environment.h>
#include <tesseract_environment/environment_cache.h>
#include <tesseract_environment/environment_monitor.h>
#include <tesseract_command_language/poly/instruction_poly.h>
#include <tesseract_command_language/composite_instruction.h>
#include <tesseract_command_language/profile_dictionary.h>
#include <tesseract_command_language/constants.h>

using tesseract_common::Serialization;
using tesseract_planning::InstructionPoly;
using tesseract_planning::TaskComposerDataStorage;
using tesseract_rosutils::processMsg;

static const std::string TRAJOPT_DEFAULT_NAMESPACE = "TrajOptMotionPlannerTask";
static const std::string TRAJOPT_IFOPT_DEFAULT_NAMESPACE = "TrajOptIfoptMotionPlannerTask";
static const std::string OMPL_DEFAULT_NAMESPACE = "OMPLMotionPlannerTask";
static const std::string DESCARTES_DEFAULT_NAMESPACE = "DescartesMotionPlannerTask";
static const std::string SIMPLE_DEFAULT_NAMESPACE = "SimpleMotionPlannerTask";

namespace tesseract_planning_server
{
const std::string TesseractPlanningServer::DEFAULT_GET_MOTION_PLAN_ACTION = "tesseract_get_motion_plan";

struct TesseractPlanningServer::Implementation
{
  ros::NodeHandle nh;

  /** @brief The environment monitor to keep the planning server updated with the latest */
  tesseract_environment::EnvironmentMonitor::Ptr monitor;

  /** @brief The environment cache being used by the process planning server */
  tesseract_environment::EnvironmentCache::Ptr environment_cache;

  /** @brief The task profiles */
  tesseract_planning::ProfileDictionary::Ptr profiles;

  /** @brief The task planning server */
  tesseract_planning::TaskComposerServer::UPtr planning_server;

  /** @brief The motion planning action server */
  actionlib::SimpleActionServer<tesseract_msgs::GetMotionPlanAction> motion_plan_server;

  /** @brief TF buffer to track TCP transforms */
  std::shared_ptr<tf2_ros::Buffer> tf_buffer;

  /** @brief TF listener to lookup TCP transforms */
  tf2_ros::TransformListener tf_listener;

  Implementation(const std::string& robot_description_, std::string name_)
    : nh("~")
    , monitor(std::make_shared<tesseract_monitoring::ROSEnvironmentMonitor>(robot_description_, name_))
    , environment_cache(std::make_shared<tesseract_environment::DefaultEnvironmentCache>(monitor->getEnvironment()))
    , profiles(std::make_shared<tesseract_planning::ProfileDictionary>())
    , planning_server(std::make_unique<tesseract_planning::TaskComposerServer>())
    , motion_plan_server(nh,
                         DEFAULT_GET_MOTION_PLAN_ACTION,
                         boost::bind(&TesseractPlanningServer::Implementation::onMotionPlanningCallback, this, _1),
                         true)
    , tf_buffer(std::make_shared<tf2_ros::Buffer>())
    , tf_listener(*tf_buffer)
  {
    ctor();
  }

  Implementation(std::unique_ptr<tesseract_environment::Environment> env_, std::string name_)
    : nh("~")
    , monitor(std::make_shared<tesseract_monitoring::ROSEnvironmentMonitor>(std::move(env_), name_))
    , environment_cache(std::make_shared<tesseract_environment::DefaultEnvironmentCache>(monitor->getEnvironment()))
    , profiles(std::make_shared<tesseract_planning::ProfileDictionary>())
    , planning_server(std::make_unique<tesseract_planning::TaskComposerServer>())
    , motion_plan_server(nh,
                         DEFAULT_GET_MOTION_PLAN_ACTION,
                         boost::bind(&TesseractPlanningServer::Implementation::onMotionPlanningCallback, this, _1),
                         true)
    , tf_buffer(std::make_shared<tf2_ros::Buffer>())
    , tf_listener(*tf_buffer)
  {
    ctor();
  }

  void ctor()
  {
    loadDefaultPlannerProfiles();
    monitor->environment().addFindTCPOffsetCallback(
        std::bind(&TesseractPlanningServer::Implementation::tfFindTCPOffset, this, std::placeholders::_1));
  }

  void loadDefaultPlannerProfiles()
  {
    // Add Simple Default Profiles
    profiles->addProfile<tesseract_planning::SimplePlannerPlanProfile>(
        SIMPLE_DEFAULT_NAMESPACE,
        tesseract_planning::DEFAULT_PROFILE_KEY,
        std::make_shared<tesseract_planning::SimplePlannerLVSNoIKPlanProfile>());

    // Add TrajOpt Default Profiles
#ifdef TESSERACT_PLANNING_SERVER_HAS_TRAJOPT
    profiles->addProfile<tesseract_planning::TrajOptPlanProfile>(
        TRAJOPT_DEFAULT_NAMESPACE,
        tesseract_planning::DEFAULT_PROFILE_KEY,
        std::make_shared<tesseract_planning::TrajOptDefaultPlanProfile>());
    profiles->addProfile<tesseract_planning::TrajOptCompositeProfile>(
        TRAJOPT_DEFAULT_NAMESPACE,
        tesseract_planning::DEFAULT_PROFILE_KEY,
        std::make_shared<tesseract_planning::TrajOptDefaultCompositeProfile>());
    profiles->addProfile<tesseract_planning::TrajOptSolverProfile>(
        TRAJOPT_DEFAULT_NAMESPACE,
        tesseract_planning::DEFAULT_PROFILE_KEY,
        std::make_shared<tesseract_planning::TrajOptDefaultSolverProfile>());
#endif

    // Add TrajOpt IFOPT Default Profiles
#ifdef TESSERACT_PLANNING_SERVER_HAS_TRAJOPT_IFOPT
    profiles->addProfile<tesseract_planning::TrajOptIfoptPlanProfile>(
        TRAJOPT_IFOPT_DEFAULT_NAMESPACE,
        tesseract_planning::DEFAULT_PROFILE_KEY,
        std::make_shared<tesseract_planning::TrajOptIfoptDefaultPlanProfile>());
    profiles->addProfile<tesseract_planning::TrajOptIfoptCompositeProfile>(
        TRAJOPT_IFOPT_DEFAULT_NAMESPACE,
        tesseract_planning::DEFAULT_PROFILE_KEY,
        std::make_shared<tesseract_planning::TrajOptIfoptDefaultCompositeProfile>());
#endif

    // Add Descartes Default Profiles
#ifdef TESSERACT_PLANNING_SERVER_HAS_DESCARTES
    profiles->addProfile<tesseract_planning::DescartesPlanProfile<double>>(
        DESCARTES_DEFAULT_NAMESPACE,
        tesseract_planning::DEFAULT_PROFILE_KEY,
        std::make_shared<tesseract_planning::DescartesDefaultPlanProfile<double>>());
#endif

    // Add OMPL Default Profiles
#ifdef TESSERACT_PLANNING_SERVER_HAS_OMPL
    profiles->addProfile<tesseract_planning::OMPLPlanProfile>(
        OMPL_DEFAULT_NAMESPACE,
        tesseract_planning::DEFAULT_PROFILE_KEY,
        std::make_shared<tesseract_planning::OMPLDefaultPlanProfile>());
#endif
  }

  Eigen::Isometry3d tfFindTCPOffset(const tesseract_common::ManipulatorInfo& manip_info)
  {
    if (manip_info.tcp_offset.index() == 1)
      throw std::runtime_error("tfFindTCPOffset: TCP offset is not a string!");

    if (manip_info.tcp_frame.empty())
      throw std::runtime_error("tfFindTCPOffset: TCP offset is empty!");

    const std::string& tcp_frame = manip_info.tcp_frame;
    const std::string& tcp_name = std::get<0>(manip_info.tcp_offset);

    auto tcp_msg = tf_buffer->lookupTransform(tcp_frame, tcp_name, ros::Time(0), ros::Duration(2));
    return tf2::transformToEigen(tcp_msg.transform);
  }

  void onMotionPlanningCallback(const tesseract_msgs::GetMotionPlanGoalConstPtr& goal)
  {
    ROS_INFO("Tesseract Planning Server Received Request!");
    tesseract_msgs::GetMotionPlanResult result;

    // Check if process planner exist
    if (!planning_server->hasTask(goal->request.name))
    {
      result.response.successful = false;
      std::ostringstream oss;
      oss << "Requested task '" << goal->request.name << "' is not supported!" << std::endl;
      oss << "   Available Tasks:" << std::endl;
      for (const auto& planner : planning_server->getAvailableTasks())
        oss << "      - " << planner << std::endl;
      ROS_ERROR_STREAM(oss.str());
      motion_plan_server.setSucceeded(result);
      return;
    }

    std::string executor_name = goal->request.executor;
    std::vector<std::string> available_executors = planning_server->getAvailableExecutors();
    if (executor_name.empty() && !available_executors.empty())
      executor_name = planning_server->getAvailableExecutors().front();

    // Check if executor exists
    if (!planning_server->hasExecutor(executor_name))
    {
      result.response.successful = false;
      std::ostringstream oss;
      oss << "Requested executor '" << executor_name << "' is not supported!" << std::endl;
      oss << "   Available Executors:" << std::endl;
      for (const auto& executor : available_executors)
        oss << "      - " << executor << std::endl;
      ROS_ERROR_STREAM(oss.str());
      motion_plan_server.setSucceeded(result);
      return;
    }

    tesseract_common::AnyPoly planning_input;
    try
    {
      planning_input = Serialization::fromArchiveStringXML<tesseract_common::AnyPoly>(goal->request.input);
    }
    catch (const std::exception& e)
    {
      result.response.successful = false;
      std::ostringstream oss;
      oss << "Failed to deserialize planning input instruction with error: '" << e.what() << "'!" << std::endl;
      oss << "   Make sure the program was serialized from an AnyPoly type." << std::endl;
      ROS_ERROR_STREAM(oss.str());
      motion_plan_server.setSucceeded(result);
      return;
    }

    tesseract_environment::Environment::Ptr env = environment_cache->getCachedEnvironment();

    tesseract_scene_graph::SceneState env_state;
    tesseract_rosutils::fromMsg(env_state.joints, goal->request.environment_state.joint_state);

    env->applyCommands(tesseract_rosutils::fromMsg(goal->request.commands));
    env->setState(env_state.joints);

    // Create solve data storage
    auto data = std::make_unique<TaskComposerDataStorage>();
    data->setData("planning_input", std::move(planning_input));
    data->setData("environment", std::move(env));
    data->setData("profiles", profiles);
    auto move_profile_remapping = tesseract_rosutils::fromMsg(goal->request.move_profile_remapping);
    if (!move_profile_remapping.empty())
      data->setData("move_profile_remapping", move_profile_remapping);

    auto composite_profile_remapping = tesseract_rosutils::fromMsg(goal->request.composite_profile_remapping);
    if (!composite_profile_remapping.empty())
      data->setData("composite_profile_remapping", composite_profile_remapping);

    // Store the initial state in the response for publishing trajectories
    tesseract_scene_graph::SceneState initial_state = env->getState();
    tesseract_rosutils::toMsg(result.response.initial_state, initial_state.joints);

    // Solve
    tesseract_common::Timer timer;
    timer.start();
    tesseract_planning::TaskComposerFuture::UPtr plan_future =
        planning_server->run(goal->request.name, std::move(data), goal->request.dotgraph, executor_name);
    plan_future->wait();  // Wait for results
    timer.stop();

    // Generate DOT Graph if requested
    if (goal->request.dotgraph)
    {
      try
      {
        // Save dot graph
        const tesseract_planning::TaskComposerNode& task = planning_server->getTask(plan_future->context->name);
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
      const tesseract_planning::TaskComposerNode& task = planning_server->getTask(plan_future->context->name);
      tesseract_common::AnyPoly results = plan_future->context->data_storage->getData(task.getOutputKeys().get("proble"
                                                                                                               "m"));
      result.response.results = Serialization::toArchiveStringXML<tesseract_planning::InstructionPoly>(
          results.as<tesseract_planning::CompositeInstruction>());
    }
    catch (const std::exception& e)
    {
      result.response.successful = false;
      std::ostringstream oss;
      oss << "Failed to get output results from task with error: '" << e.what() << "'!" << std::endl;
      ROS_ERROR_STREAM(oss.str());
      motion_plan_server.setSucceeded(result);
      return;
    }

    result.response.successful = plan_future->context->isSuccessful();
    plan_future->clear();

    ROS_INFO("Tesseract Planning Server Finished Request in %f seconds!", timer.elapsedSeconds());
    motion_plan_server.setSucceeded(result);
  }
};

TesseractPlanningServer::TesseractPlanningServer(const std::string& robot_description, std::string name)
  : impl_(std::make_unique<Implementation>(robot_description, std::move(name)))
{
}

TesseractPlanningServer::TesseractPlanningServer(std::unique_ptr<tesseract_environment::Environment> env,
                                                 std::string name)
  : impl_(std::make_unique<Implementation>(std::move(env), std::move(name)))
{
}

TesseractPlanningServer::~TesseractPlanningServer() = default;
tesseract_environment::EnvironmentMonitor& TesseractPlanningServer::getEnvironmentMonitor() { return *impl_->monitor; }
const tesseract_environment::EnvironmentMonitor& TesseractPlanningServer::getEnvironmentMonitor() const
{
  return *impl_->monitor;
}

tesseract_planning::TaskComposerServer& TesseractPlanningServer::getTaskComposerServer()
{
  return *impl_->planning_server;
}
const tesseract_planning::TaskComposerServer& TesseractPlanningServer::getTaskComposerServer() const
{
  return *impl_->planning_server;
}

tesseract_environment::EnvironmentCache& TesseractPlanningServer::getEnvironmentCache()
{
  return *impl_->environment_cache;
}
const tesseract_environment::EnvironmentCache& TesseractPlanningServer::getEnvironmentCache() const
{
  return *impl_->environment_cache;
}

tesseract_planning::ProfileDictionary& TesseractPlanningServer::getProfileDictionary() { return *impl_->profiles; }
const tesseract_planning::ProfileDictionary& TesseractPlanningServer::getProfileDictionary() const
{
  return *impl_->profiles;
}

}  // namespace tesseract_planning_server
