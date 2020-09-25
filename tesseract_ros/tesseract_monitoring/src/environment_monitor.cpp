/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2011, Willow Garage, Inc.
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of Willow Garage nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *********************************************************************/

/* Author: Ioan Sucan */

#include <tesseract_common/macros.h>
TESSERACT_COMMON_IGNORE_WARNINGS_PUSH
#include <ros/console.h>
#include <dynamic_reconfigure/server.h>
#include <tesseract_monitoring/EnvironmentMonitorDynamicReconfigureConfig.h>
TESSERACT_COMMON_IGNORE_WARNINGS_POP

#include <tesseract_environment/core/utils.h>
#include <tesseract_monitoring/environment_monitor.h>
#include <tesseract_scene_graph/utils.h>
#include <tesseract_rosutils/utils.h>

class DynamicReconfigureImpl
{
public:
  DynamicReconfigureImpl(tesseract_monitoring::EnvironmentMonitor* owner)
    : owner_(owner), dynamic_reconfigure_server_(ros::NodeHandle(decideNamespace(owner->getName())))
  {
    dynamic_reconfigure_server_.setCallback(
        boost::bind(&DynamicReconfigureImpl::dynamicReconfigureCallback, this, _1, _2));
  }

private:
  // make sure we do not advertise the same service multiple times, in case we
  // use multiple PlanningSceneMonitor
  // instances in a process
  static std::string decideNamespace(const std::string& name)
  {
    std::string ns = "~/" + name;
    std::replace(ns.begin(), ns.end(), ' ', '_');
    std::transform(ns.begin(), ns.end(), ns.begin(), ::tolower);
    if (ros::service::exists(ns + "/set_parameters", false))
    {
      unsigned int c = 1;
      while (ros::service::exists(ns + boost::lexical_cast<std::string>(c) + "/set_parameters", false))
        c++;
      ns += boost::lexical_cast<std::string>(c);
    }
    return ns;
  }

  void dynamicReconfigureCallback(tesseract_monitoring::EnvironmentMonitorDynamicReconfigureConfig& config,
                                  uint32_t /*level*/)
  {
    using namespace tesseract_monitoring;
    EnvironmentMonitor::EnvironmentUpdateType event = EnvironmentMonitor::UPDATE_NONE;
    if (config.publish_geometry_updates)
      event = static_cast<EnvironmentMonitor::EnvironmentUpdateType>(event | EnvironmentMonitor::UPDATE_GEOMETRY);
    if (config.publish_state_updates)
      event = static_cast<EnvironmentMonitor::EnvironmentUpdateType>(event | EnvironmentMonitor::UPDATE_STATE);
    if (config.publish_transforms_updates)
      event = static_cast<EnvironmentMonitor::EnvironmentUpdateType>(event | EnvironmentMonitor::UPDATE_TRANSFORMS);
    if (config.publish_environment)
    {
      owner_->setEnvironmentPublishingFrequency(config.publish_environment_hz);
      owner_->startPublishingEnvironment(event);
    }
    else
      owner_->stopPublishingEnvironment();
  }

  tesseract_monitoring::EnvironmentMonitor* owner_;
  dynamic_reconfigure::Server<tesseract_monitoring::EnvironmentMonitorDynamicReconfigureConfig>
      dynamic_reconfigure_server_;
};

namespace tesseract_monitoring
{
const std::string EnvironmentMonitor::DEFAULT_JOINT_STATES_TOPIC = "joint_states";
const std::string EnvironmentMonitor::DEFAULT_GET_ENVIRONMENT_CHANGES_SERVICE = R"(/get_tesseract_changes)";
const std::string EnvironmentMonitor::DEFAULT_GET_ENVIRONMENT_INFORMATION_SERVICE = R"(/get_tesseract_information)";
const std::string EnvironmentMonitor::DEFAULT_MODIFY_ENVIRONMENT_SERVICE = R"(/modify_tesseract)";
const std::string EnvironmentMonitor::DEFAULT_SAVE_SCENE_GRAPH_SERVICE = R"(/save_scene_graph)";
const std::string EnvironmentMonitor::DEFAULT_PUBLISH_ENVIRONMENT_TOPIC = R"(/tesseract_published_environment)";

EnvironmentMonitor::EnvironmentMonitor(const std::string& robot_description,
                                       std::string monitor_namespace,
                                       std::string discrete_plugin,
                                       std::string continuous_plugin)
  : monitor_namespace_(std::move(monitor_namespace))
  , discrete_plugin_name_(std::move(discrete_plugin))
  , continuous_plugin_name_(std::move(continuous_plugin))
  , nh_("~")
{
  // Initial setup
  std::string urdf_xml_string, srdf_xml_string;
  if (!root_nh_.hasParam(robot_description))
  {
    ROS_ERROR("Failed to find parameter: %s", robot_description.c_str());
    return;
  }

  if (!root_nh_.hasParam(robot_description + "_semantic"))
  {
    ROS_ERROR("Failed to find parameter: %s", (robot_description + "_semantic").c_str());
    return;
  }

  root_nh_.getParam(robot_description, urdf_xml_string);
  root_nh_.getParam(robot_description + "_semantic", srdf_xml_string);

  tesseract_ = std::make_shared<tesseract::Tesseract>();
  tesseract_scene_graph::ResourceLocator::Ptr locator = std::make_shared<tesseract_rosutils::ROSResourceLocator>();
  if (!tesseract_->init(urdf_xml_string, srdf_xml_string, locator))
    return;

  initialize();
}

EnvironmentMonitor::EnvironmentMonitor(tesseract::Tesseract::Ptr tesseract,
                                       std::string monitor_namespace,
                                       std::string discrete_plugin,
                                       std::string continuous_plugin)
  : monitor_namespace_(std::move(monitor_namespace))
  , discrete_plugin_name_(std::move(discrete_plugin))
  , continuous_plugin_name_(std::move(continuous_plugin))
  , tesseract_(std::move(tesseract))
  , nh_("~")
{
  initialize();
}

EnvironmentMonitor::~EnvironmentMonitor()
{
  stopPublishingEnvironment();
  stopStateMonitor();

  delete reconfigure_impl_;
  current_state_monitor_.reset();
  tesseract_.reset();
}

void EnvironmentMonitor::initialize()
{
  enforce_next_state_update_ = false;

  if (monitor_namespace_.empty())
    throw std::runtime_error("The monitor namespace cannot be empty!");

  if (!tesseract_->isInitialized())
  {
    ROS_FATAL_NAMED(monitor_namespace_, "Faild to initalize environment monitor");
    return;
  }

  try
  {
    discrete_manager_loader_.reset(
        new DiscreteContactManagerPluginLoader("tesseract_collision", "tesseract_collision::DiscreteContactManager"));
    for (auto plugin : discrete_manager_loader_->getDeclaredClasses())
    {
      auto fn = [&]() -> tesseract_collision::DiscreteContactManager::Ptr {
        return discrete_manager_loader_->createUniqueInstance(plugin);
      };
      tesseract_->getEnvironment()->registerDiscreteContactManager(discrete_manager_loader_->getClassType(plugin), fn);

      ROS_INFO("Discrete Contact Monitor Registered: %s", discrete_manager_loader_->getClassType(plugin).c_str());
    }

    // The tesseract sets a default so it is ok if one is not provided here.
    if (!discrete_plugin_name_.empty())
    {
      if (!discrete_manager_loader_->isClassAvailable(discrete_plugin_name_))
      {
        std::string msg = "\nFailed to set default discrete contact checker plugin: ";
        msg += discrete_plugin_name_ + '\n';
        msg += "  Available Plugins:\n";

        auto available_plugins = discrete_manager_loader_->getDeclaredClasses();
        for (const auto& plugin : available_plugins)
          msg += "    " + plugin + '\n';

        ROS_ERROR(msg.c_str());
      }
      else
      {
        tesseract_->getEnvironment()->setActiveDiscreteContactManager(discrete_plugin_name_);
      }
    }

    continuous_manager_loader_.reset(new ContinuousContactManagerPluginLoader("tesseract_collision",
                                                                              "tesseract_collision::"
                                                                              "ContinuousContactManager"));
    for (auto plugin : continuous_manager_loader_->getDeclaredClasses())
    {
      auto fn = [&]() -> tesseract_collision::ContinuousContactManager::Ptr {
        return continuous_manager_loader_->createUniqueInstance(plugin);
      };
      tesseract_->getEnvironment()->registerContinuousContactManager(continuous_manager_loader_->getClassType(plugin),
                                                                     fn);

      ROS_INFO("Continuous Contact Monitor Registered: %s", continuous_manager_loader_->getClassType(plugin).c_str());
    }

    if (!continuous_plugin_name_.empty())
    {
      if (!continuous_manager_loader_->isClassAvailable(continuous_plugin_name_))
      {
        std::string msg = "\nFailed to set default continuous contact checker plugin: ";
        msg += continuous_plugin_name_ + '\n';
        msg += "  Available Plugins:\n";

        auto available_plugins = continuous_manager_loader_->getDeclaredClasses();
        for (const auto& plugin : available_plugins)
          msg += "    " + plugin + '\n';

        ROS_ERROR(msg.c_str());
      }
      else
      {
        tesseract_->getEnvironment()->setActiveContinuousContactManager(continuous_plugin_name_);
      }
    }
  }
  catch (int& /*e*/)
  {
    ROS_ERROR_NAMED(monitor_namespace_, "Failed to load tesseract contact managers plugin");
    tesseract_.reset();
  }

  publish_environment_frequency_ = 2.0;
  new_environment_update_ = UPDATE_NONE;

  last_update_time_ = last_robot_motion_time_ = ros::Time::now();
  last_robot_state_update_wall_time_ = ros::WallTime::now();
  dt_state_update_ = ros::WallDuration(0.1);

  state_update_pending_ = false;
  state_update_timer_ = nh_.createWallTimer(dt_state_update_,
                                            &EnvironmentMonitor::updateJointStateTimerCallback,
                                            this,
                                            false,   // not a oneshot timer
                                            false);  // do not start the timer yet

  reconfigure_impl_ = new DynamicReconfigureImpl(this);

  std::string modify_environment_server = R"(/)" + monitor_namespace_ + DEFAULT_MODIFY_ENVIRONMENT_SERVICE;
  std::string get_environment_changes_server = R"(/)" + monitor_namespace_ + DEFAULT_GET_ENVIRONMENT_CHANGES_SERVICE;
  std::string get_environment_information_server =
      R"(/)" + monitor_namespace_ + DEFAULT_GET_ENVIRONMENT_INFORMATION_SERVICE;
  std::string save_scene_graph_server = R"(/)" + monitor_namespace_ + DEFAULT_SAVE_SCENE_GRAPH_SERVICE;

  modify_environment_server_ =
      root_nh_.advertiseService(modify_environment_server, &EnvironmentMonitor::modifyEnvironmentCallback, this);

  get_environment_changes_server_ = root_nh_.advertiseService(
      get_environment_changes_server, &EnvironmentMonitor::getEnvironmentChangesCallback, this);

  get_environment_information_server_ = root_nh_.advertiseService(
      get_environment_information_server, &EnvironmentMonitor::getEnvironmentInformationCallback, this);

  save_scene_graph_server_ =
      root_nh_.advertiseService(save_scene_graph_server, &EnvironmentMonitor::saveSceneGraphCallback, this);
}

const std::string& EnvironmentMonitor::getName() const { return monitor_namespace_; }

tesseract_scene_graph::SceneGraph::ConstPtr EnvironmentMonitor::getSceneGraph() const
{
  return tesseract_->getEnvironment()->getSceneGraph();
}

tesseract_scene_graph::SRDFModel::ConstPtr EnvironmentMonitor::getSRDF() const
{
  return tesseract_->getManipulatorManager()->getSRDFModel();
}

tesseract_environment::Environment::Ptr EnvironmentMonitor::getEnvironment() { return tesseract_->getEnvironment(); }

tesseract_environment::Environment::ConstPtr EnvironmentMonitor::getEnvironment() const
{
  return tesseract_->getEnvironment();
}

tesseract::Tesseract::Ptr EnvironmentMonitor::getTesseract() { return tesseract_; }

tesseract::Tesseract::ConstPtr EnvironmentMonitor::getTesseractConst() const { return tesseract_; }

void EnvironmentMonitor::stopPublishingEnvironment()
{
  if (publish_environment_)
  {
    std::unique_ptr<std::thread> copy;
    copy.swap(publish_environment_);
    new_environment_update_condition_.notify_all();
    copy->join();
    stopPublishingEnvironment();
    environment_publisher_.shutdown();
    ROS_INFO_NAMED(monitor_namespace_, "Stopped publishing maintained environment.");
  }
}

void EnvironmentMonitor::startPublishingEnvironment(EnvironmentUpdateType update_type)
{
  publish_update_types_ = update_type;
  if (!publish_environment_ && tesseract_->isInitialized())
  {
    std::string environment_topic = R"(/)" + monitor_namespace_ + DEFAULT_PUBLISH_ENVIRONMENT_TOPIC;
    environment_publisher_ = nh_.advertise<tesseract_msgs::TesseractState>(environment_topic, 100, false);
    ROS_INFO_NAMED(monitor_namespace_, "Publishing maintained environment on '%s'", environment_topic.c_str());
    publish_environment_ =
        std::make_unique<std::thread>(std::bind(&EnvironmentMonitor::environmentPublishingThread, this));
  }
}

double EnvironmentMonitor::getEnvironmentPublishingFrequency() const { return publish_environment_frequency_; }

void EnvironmentMonitor::environmentPublishingThread()
{
  ROS_DEBUG_NAMED(monitor_namespace_, "Started environment state publishing thread ...");

  // publish the full planning scene
  tesseract_msgs::TesseractState start_msg;
  tesseract_rosutils::toMsg(start_msg, *(tesseract_->getEnvironment()));

  environment_publisher_.publish(start_msg);
  ros::Duration(1.5).sleep();
  environment_publisher_.publish(start_msg);

  ROS_DEBUG_NAMED(monitor_namespace_, "Published the Tesseract Environment State for: '%s'", start_msg.id.c_str());

  do
  {
    tesseract_msgs::TesseractState msg;
    bool publish_msg = false;
    ros::Rate rate(publish_environment_frequency_);
    {
      std::unique_lock<std::shared_mutex> ulock(scene_update_mutex_);
      while (new_environment_update_ == UPDATE_NONE && publish_environment_)
        new_environment_update_condition_.wait(ulock);
      if (new_environment_update_ != UPDATE_NONE)
      {
        if ((publish_update_types_ & new_environment_update_) || new_environment_update_ == UPDATE_ENVIRONMENT)
        {
          tesseract_rosutils::toMsg(msg, *(tesseract_->getEnvironment()));

          // also publish timestamp of this robot_state
          msg.joint_state.header.stamp = last_robot_motion_time_;
          publish_msg = true;
        }
        new_environment_update_ = UPDATE_NONE;
      }
    }

    if (publish_msg)
    {
      rate.reset();
      environment_publisher_.publish(msg);
      rate.sleep();
    }
  } while (publish_environment_);
}

void EnvironmentMonitor::stopMonitoringEnvironment()
{
  monitored_environment_subscriber_.shutdown();
  ROS_INFO_NAMED(monitor_namespace_, "Stopped monitoring environment.");
}

CurrentStateMonitor::ConstPtr EnvironmentMonitor::getStateMonitor() const { return current_state_monitor_; }

CurrentStateMonitor::Ptr EnvironmentMonitor::getStateMonitor() { return current_state_monitor_; }

void EnvironmentMonitor::startMonitoringEnvironment(const std::string& monitored_namespace,
                                                    MonitoredEnvironmentMode mode)
{
  if (tesseract_->isInitialized())
  {
    monitored_environment_mode_ = mode;
    std::string monitored_environment_topic = R"(/)" + monitored_namespace + DEFAULT_PUBLISH_ENVIRONMENT_TOPIC;
    std::string monitored_environment_changes_service =
        R"(/)" + monitored_namespace + DEFAULT_GET_ENVIRONMENT_CHANGES_SERVICE;
    std::string monitored_environment_modify_service =
        R"(/)" + monitored_namespace + DEFAULT_MODIFY_ENVIRONMENT_SERVICE;

    get_monitored_environment_changes_client_ =
        nh_.serviceClient<tesseract_msgs::GetEnvironmentChanges>(monitored_environment_changes_service);
    modify_monitored_environment_client_ =
        nh_.serviceClient<tesseract_msgs::ModifyEnvironment>(monitored_environment_modify_service);

    monitored_environment_subscriber_ =
        nh_.subscribe(monitored_environment_topic, 1000, &EnvironmentMonitor::newTesseractStateCallback, this);
    ROS_INFO_NAMED(monitor_namespace_, "Monitoring external environment on '%s'", monitored_environment_topic.c_str());
  }
}

void EnvironmentMonitor::getStateMonitoredTopics(std::vector<std::string>& topics) const
{
  topics.clear();
  if (current_state_monitor_)
  {
    const std::string& t = current_state_monitor_->getMonitoredTopic();
    if (!t.empty())
      topics.push_back(t);
  }
}

double EnvironmentMonitor::getStateUpdateFrequency() const
{
  if (!dt_state_update_.isZero())
    return 1.0 / dt_state_update_.toSec();

  return 0.0;
}

void EnvironmentMonitor::triggerEnvironmentUpdateEvent(EnvironmentUpdateType update_type)
{
  // do not modify update functions while we are calling them
  std::scoped_lock<std::recursive_mutex> lock(update_lock_);

  for (auto& update_callback : update_callbacks_)
    update_callback(update_type);
  new_environment_update_ = static_cast<EnvironmentUpdateType>(new_environment_update_ | update_type);
  new_environment_update_condition_.notify_all();
}

void EnvironmentMonitor::newTesseractStateCallback(const tesseract_msgs::TesseractStateConstPtr& env)
{
  if (!tesseract_->getEnvironment())
    return;

  EnvironmentUpdateType upd = UPDATE_ENVIRONMENT;
  {
    std::unique_lock<std::shared_mutex> ulock(scene_update_mutex_);

    last_update_time_ = ros::Time::now();
    last_robot_motion_time_ = env->joint_state.header.stamp;
    ROS_DEBUG_STREAM_NAMED(monitor_namespace_,
                           "environment update " << fmod(last_update_time_.toSec(), 10.)
                                                 << " robot stamp: " << fmod(last_robot_motion_time_.toSec(), 10.));

    // If the monitored environment has changed then request the changes and apply
    auto environment = tesseract_->getEnvironment();
    if (static_cast<int>(env->revision) > environment->getRevision())
    {
      tesseract_msgs::GetEnvironmentChanges res;
      res.request.revision = static_cast<unsigned long>(environment->getRevision());
      if (get_monitored_environment_changes_client_.call(res))
      {
        if (!tesseract_rosutils::processMsg(*(tesseract_->getEnvironment()), res.response.commands))
        {
          ROS_ERROR_STREAM_NAMED(monitor_namespace_, "Failed to apply monitored environments changes.");
        }
      }
      else
      {
        ROS_ERROR_STREAM_NAMED(monitor_namespace_, "Failed to get monitored environments changes.");
      }
    }
    else if (static_cast<int>(env->revision) < environment->getRevision())
    {
      if (monitored_environment_mode_ == MonitoredEnvironmentMode::DEFAULT)
      {
        // If the monitored environment has a lower revision it is reset and additional changes are requested and
        // applied.
        if (tesseract_->reset())
        {
          if (static_cast<int>(env->revision) > environment->getRevision())
          {
            tesseract_msgs::GetEnvironmentChanges res;
            res.request.revision = static_cast<unsigned long>(environment->getRevision());
            if (get_monitored_environment_changes_client_.call(res))
            {
              if (!tesseract_rosutils::processMsg(*(tesseract_->getEnvironment()), res.response.commands))
              {
                ROS_ERROR_STREAM_NAMED(monitor_namespace_, "Failed to apply monitored environments changes.");
              }
            }
            else
            {
              ROS_ERROR_STREAM_NAMED(monitor_namespace_, "Failed to get monitored environments changes.");
            }
          }
        }
        else
        {
          ROS_ERROR_STREAM_NAMED(monitor_namespace_, "Failed to reset the tesseract object!");
        }
      }
      else if (monitored_environment_mode_ == MonitoredEnvironmentMode::SYNCHRONIZED)
      {
        // If this has been modified it will push the changes to the monitored environment to keep them in sync
        tesseract_msgs::ModifyEnvironment res;
        res.request.id = tesseract_->getEnvironment()->getName();
        res.request.revision = env->revision;
        if (tesseract_rosutils::toMsg(
                res.request.commands, tesseract_->getEnvironment()->getCommandHistory(), env->revision))
        {
          bool status = modify_monitored_environment_client_.call(res);
          if (!status || !res.response.success)
          {
            ROS_ERROR_STREAM_NAMED(monitor_namespace_, "Failed to update monitored environment!");
          }
        }
        else
        {
          ROS_ERROR_STREAM_NAMED(monitor_namespace_,
                                 "Failed to convert latest changes to message and update monitored environment!");
        }
      }
      else
      {
        ROS_ERROR_STREAM_NAMED(monitor_namespace_, "Unsupporte MonitoredEnvironmentMode!");
      }
    }
    else
    {
      upd = UPDATE_NONE;
    }
  }

  if (!tesseract_rosutils::isMsgEmpty(env->joint_state) || !tesseract_rosutils::isMsgEmpty(env->multi_dof_joint_state))
    upd = static_cast<EnvironmentUpdateType>(upd | UPDATE_STATE);

  triggerEnvironmentUpdateEvent(upd);
}

bool EnvironmentMonitor::applyEnvironmentCommandsMessage(
    const std::string& id,
    int revision,
    const std::vector<tesseract_msgs::EnvironmentCommand>& commands)
{
  if (!tesseract_->getEnvironment() || id != tesseract_->getEnvironment()->getName() ||
      revision != tesseract_->getEnvironment()->getRevision())
    return false;

  bool result;

  EnvironmentUpdateType upd = UPDATE_ENVIRONMENT;
  std::string old_scene_name;
  {
    std::unique_lock<std::shared_mutex> ulock(scene_update_mutex_);
    result = tesseract_rosutils::processMsg(*(tesseract_->getEnvironment()), commands);
  }

  triggerEnvironmentUpdateEvent(upd);
  return result;
}

bool EnvironmentMonitor::saveSceneGraphCallback(tesseract_msgs::SaveSceneGraphRequest& req,
                                                tesseract_msgs::SaveSceneGraphResponse& res)
{
  auto env = tesseract_->getEnvironment();
  res.success = !(env == nullptr);
  env->getSceneGraph()->saveDOT(req.filepath);
  res.id = env->getName();
  res.revision = static_cast<unsigned long>(env->getRevision());

  return true;
}

bool EnvironmentMonitor::waitForCurrentState(const ros::Time& t, double wait_time)
{
  if (t.isZero())
    return false;
  ros::WallTime start = ros::WallTime::now();
  ros::WallDuration timeout(wait_time);

  ROS_DEBUG_NAMED(monitor_namespace_, "sync robot state to: %.3fs", fmod(t.toSec(), 10.));

  if (current_state_monitor_)
  {
    // Wait for next robot update in state monitor. Those updates are only
    // passed to PSM when robot actually moved!
    enforce_next_state_update_ = true;  // enforce potential updates to be directly applied
    bool success = current_state_monitor_->waitForCurrentState(t, wait_time);
    enforce_next_state_update_ = false;  // back to normal throttling behavior,
                                         // not applying incoming updates
                                         // immediately

    /* If the robot doesn't move, we will never receive an update from CSM in
       planning scene.
       As we ensured that an update, if it is triggered by CSM, is directly
       passed to the scene,
       we can early return true here (if we successfully received a CSM update).
       Otherwise return false. */
    if (success)
      return true;

    ROS_WARN_NAMED(monitor_namespace_, "Failed to fetch current robot state.");
    return false;
  }

  // Sometimes there is no state monitor. In this case state updates are
  // received as part of scene updates only.
  // However, scene updates are only published if the robot actually moves.
  // Hence we need a timeout!
  // As publishing planning scene updates is throttled (2Hz by default), a 1s
  // timeout is a suitable default.
  std::shared_lock<std::shared_mutex> lock(scene_update_mutex_);
  ros::Time prev_robot_motion_time = last_robot_motion_time_;
  while (last_robot_motion_time_ < t &&  // Wait until the state update actually reaches the scene.
         timeout > ros::WallDuration())
  {
    ROS_DEBUG_STREAM_NAMED(monitor_namespace_,
                           "last robot motion: " << (t - last_robot_motion_time_).toSec() << " ago");
    new_environment_update_condition_.wait_for(lock, std::chrono::nanoseconds(timeout.toNSec()));
    timeout -= ros::WallTime::now() - start;  // compute remaining wait_time
  }
  bool success = last_robot_motion_time_ >= t;
  // suppress warning if we received an update at all
  if (!success && prev_robot_motion_time != last_robot_motion_time_)
    ROS_WARN_NAMED(monitor_namespace_,
                   "Maybe failed to update robot state, time diff: %.3fs",
                   (t - last_robot_motion_time_).toSec());

  ROS_DEBUG_STREAM_NAMED(monitor_namespace_,
                         "sync done: robot motion: " << (t - last_robot_motion_time_).toSec()
                                                     << " scene update: " << (t - last_update_time_).toSec());
  return success;
}

std::shared_lock<std::shared_mutex> EnvironmentMonitor::lockEnvironmentRead()
{
  return std::shared_lock(scene_update_mutex_);
}
std::unique_lock<std::shared_mutex> EnvironmentMonitor::lockEnvironmentWrite()
{
  return std::unique_lock(scene_update_mutex_);
}

void EnvironmentMonitor::startStateMonitor(const std::string& joint_states_topic, bool publish_tf)
{
  stopStateMonitor();
  if (tesseract_->getEnvironment())
  {
    if (!current_state_monitor_)
      current_state_monitor_.reset(
          new CurrentStateMonitor(tesseract_->getEnvironment(), tesseract_->getManipulatorManager(), root_nh_));

    current_state_monitor_->addUpdateCallback(boost::bind(&EnvironmentMonitor::onJointStateUpdate, this, _1));
    current_state_monitor_->startStateMonitor(joint_states_topic, publish_tf);

    {
      std::scoped_lock lock(state_pending_mutex_);
      if (!dt_state_update_.isZero())
        state_update_timer_.start();
    }
  }
  else
  {
    ROS_ERROR_NAMED(monitor_namespace_, "Cannot monitor robot state because planning scene is not configured");
  }
}

void EnvironmentMonitor::stopStateMonitor()
{
  if (current_state_monitor_)
    current_state_monitor_->stopStateMonitor();

  // stop must be called with state_pending_mutex_ unlocked to avoid deadlock
  state_update_timer_.stop();
  {
    std::scoped_lock lock(state_pending_mutex_);
    state_update_pending_ = false;
  }
}

void EnvironmentMonitor::onJointStateUpdate(const sensor_msgs::JointStateConstPtr& /* joint_state */)
{
  const ros::WallTime& n = ros::WallTime::now();
  ros::WallDuration dt = n - last_robot_state_update_wall_time_;

  bool update = enforce_next_state_update_;
  {
    std::scoped_lock lock(state_pending_mutex_);

    if (dt < dt_state_update_ && !update)
    {
      state_update_pending_ = true;
    }
    else
    {
      state_update_pending_ = false;
      last_robot_state_update_wall_time_ = n;
      update = true;
    }
  }
  // run the state update with state_pending_mutex_ unlocked
  if (update)
    updateEnvironmentWithCurrentState();
}

void EnvironmentMonitor::updateJointStateTimerCallback(const ros::WallTimerEvent& /*event*/)
{
  if (state_update_pending_)
  {
    bool update = false;

    const ros::WallTime& n = ros::WallTime::now();
    ros::WallDuration dt = n - last_robot_state_update_wall_time_;

    {
      // lock for access to dt_state_update_ and state_update_pending_
      std::scoped_lock lock(state_pending_mutex_);
      if (state_update_pending_ && dt >= dt_state_update_)
      {
        state_update_pending_ = false;
        last_robot_state_update_wall_time_ = ros::WallTime::now();
        update = true;
        ROS_DEBUG_STREAM_NAMED(monitor_namespace_,
                               "performPendingStateUpdate: " << fmod(last_robot_state_update_wall_time_.toSec(), 10));
      }
    }

    // run the state update with state_pending_mutex_ unlocked
    if (update)
    {
      updateEnvironmentWithCurrentState();
      ROS_DEBUG_NAMED(monitor_namespace_, "performPendingStateUpdate done");
    }
  }
}

void EnvironmentMonitor::setStateUpdateFrequency(double hz)
{
  bool update = false;
  if (hz > std::numeric_limits<double>::epsilon())
  {
    std::scoped_lock lock(state_pending_mutex_);
    dt_state_update_.fromSec(1.0 / hz);
    state_update_timer_.setPeriod(dt_state_update_);
    state_update_timer_.start();
  }
  else
  {
    // stop must be called with state_pending_mutex_ unlocked to avoid deadlock
    state_update_timer_.stop();
    std::scoped_lock lock(state_pending_mutex_);
    dt_state_update_ = ros::WallDuration(0, 0);
    if (state_update_pending_)
      update = true;
  }
  ROS_INFO_NAMED(
      monitor_namespace_, "Updating internal planning scene state at most every %lf seconds", dt_state_update_.toSec());

  if (update)
    updateEnvironmentWithCurrentState();
}

void EnvironmentMonitor::updateEnvironmentWithCurrentState()
{
  if (current_state_monitor_)
  {
    std::vector<std::string> missing;
    if (!current_state_monitor_->haveCompleteState(missing) &&
        (ros::Time::now() - current_state_monitor_->getMonitorStartTime()).toSec() > 1.0)
    {
      std::string missing_str = boost::algorithm::join(missing, ", ");
      ROS_WARN_THROTTLE_NAMED(
          1, monitor_namespace_, "The complete state of the robot is not yet known.  Missing %s", missing_str.c_str());
    }

    {
      std::unique_lock<std::shared_mutex> ulock(scene_update_mutex_);
      last_update_time_ = last_robot_motion_time_ = current_state_monitor_->getCurrentStateTime();
      ROS_DEBUG_STREAM_NAMED(monitor_namespace_, "robot state update " << fmod(last_robot_motion_time_.toSec(), 10.));

      tesseract_->getEnvironment()->setState(current_state_monitor_->getCurrentState()->joints);
    }
    triggerEnvironmentUpdateEvent(UPDATE_STATE);
  }
  else
    ROS_ERROR_THROTTLE_NAMED(
        1, monitor_namespace_, "State monitor is not active. Unable to set the planning scene state");
}

void EnvironmentMonitor::addUpdateCallback(const std::function<void(EnvironmentUpdateType)>& fn)
{
  std::scoped_lock lock(update_lock_);
  if (fn)
    update_callbacks_.push_back(fn);
}

void EnvironmentMonitor::clearUpdateCallbacks()
{
  std::scoped_lock lock(update_lock_);
  update_callbacks_.clear();
}

void EnvironmentMonitor::setEnvironmentPublishingFrequency(double hz)
{
  publish_environment_frequency_ = hz;
  ROS_DEBUG_NAMED(monitor_namespace_,
                  "Maximum frquency for publishing an environment is now %lf Hz",
                  publish_environment_frequency_);
}

bool EnvironmentMonitor::modifyEnvironmentCallback(tesseract_msgs::ModifyEnvironmentRequest& req,
                                                   tesseract_msgs::ModifyEnvironmentResponse& res)
{
  auto env = tesseract_->getEnvironment();
  if (req.append)
    res.success = applyEnvironmentCommandsMessage(req.id, env->getRevision(), req.commands);
  else
    res.success = applyEnvironmentCommandsMessage(req.id, static_cast<int>(req.revision), req.commands);

  res.revision = static_cast<unsigned long>(env->getRevision());
  return res.success;
}

bool EnvironmentMonitor::getEnvironmentChangesCallback(tesseract_msgs::GetEnvironmentChangesRequest& req,
                                                       tesseract_msgs::GetEnvironmentChangesResponse& res)
{
  if (static_cast<int>(req.revision) > tesseract_->getEnvironment()->getRevision())
  {
    res.success = false;
    return false;
  }

  res.id = tesseract_->getEnvironment()->getName();
  res.revision = static_cast<unsigned long>(tesseract_->getEnvironment()->getRevision());
  if (!tesseract_rosutils::toMsg(res.commands, tesseract_->getEnvironment()->getCommandHistory(), req.revision))
  {
    res.success = false;
    return false;
  }

  res.success = true;
  return res.success;
}

bool EnvironmentMonitor::getEnvironmentInformationCallback(tesseract_msgs::GetEnvironmentInformationRequest& req,
                                                           tesseract_msgs::GetEnvironmentInformationResponse& res)
{
  auto lock_read = lockEnvironmentRead();

  auto env = tesseract_->getEnvironment();
  res.id = env->getName();
  res.revision = static_cast<unsigned long>(env->getRevision());

  if (req.flags & tesseract_msgs::GetEnvironmentInformationRequest::COMMAND_HISTORY)
  {
    if (!tesseract_rosutils::toMsg(res.command_history, env->getCommandHistory(), 0))
    {
      res.success = false;
      return false;
    }
  }

  if (req.flags & tesseract_msgs::GetEnvironmentInformationRequest::LINK_LIST)
  {
    for (const auto& link : env->getSceneGraph()->getLinks())
    {
      tesseract_msgs::Link msg;
      if (!tesseract_rosutils::toMsg(msg, *link))
      {
        res.success = false;
        return false;
      }
      res.links.push_back(msg);
    }
  }

  if (req.flags & tesseract_msgs::GetEnvironmentInformationRequest::JOINT_LIST)
  {
    for (const auto& joint : env->getSceneGraph()->getJoints())
    {
      tesseract_msgs::Joint msg;
      if (!tesseract_rosutils::toMsg(msg, *joint))
      {
        res.success = false;
        return false;
      }
      res.joints.push_back(msg);
    }
  }

  if (req.flags & tesseract_msgs::GetEnvironmentInformationRequest::LINK_NAMES)
  {
    for (const auto& link : env->getLinkNames())
    {
      res.link_names.push_back(link);
    }
  }

  if (req.flags & tesseract_msgs::GetEnvironmentInformationRequest::JOINT_NAMES)
  {
    for (const auto& joint : env->getJointNames())
    {
      res.joint_names.push_back(joint);
    }
  }

  if (req.flags & tesseract_msgs::GetEnvironmentInformationRequest::ACTIVE_LINK_NAMES)
  {
    for (const auto& link : env->getActiveLinkNames())
    {
      res.active_link_names.push_back(link);
    }
  }

  if (req.flags & tesseract_msgs::GetEnvironmentInformationRequest::ACTIVE_JOINT_NAMES)
  {
    for (const auto& joint : env->getActiveJointNames())
    {
      res.active_joint_names.push_back(joint);
    }
  }

  if (req.flags & tesseract_msgs::GetEnvironmentInformationRequest::LINK_TRANSFORMS)
  {
    for (const auto& link_pair : env->getCurrentState()->link_transforms)
    {
      res.link_transforms.names.push_back(link_pair.first);
      geometry_msgs::Pose pose;
      tesseract_rosutils::toMsg(pose, link_pair.second);
      res.link_transforms.transforms.push_back(pose);
    }
  }

  if (req.flags & tesseract_msgs::GetEnvironmentInformationRequest::JOINT_TRANSFORMS)
  {
    for (const auto& joint_pair : env->getCurrentState()->joint_transforms)
    {
      res.joint_transforms.names.push_back(joint_pair.first);
      geometry_msgs::Pose pose;
      tesseract_rosutils::toMsg(pose, joint_pair.second);
      res.joint_transforms.transforms.push_back(pose);
    }
  }

  if (req.flags & tesseract_msgs::GetEnvironmentInformationRequest::ALLOWED_COLLISION_MATRIX)
  {
    if (!tesseract_rosutils::toMsg(res.allowed_collision_matrix, *env->getAllowedCollisionMatrix()))
    {
      res.success = false;
      return false;
    }
  }

  if (req.flags & tesseract_msgs::GetEnvironmentInformationRequest::KINEMATICS_INFORMATION)
  {
    auto manipulator_manager = tesseract_->getManipulatorManager();
    if (!tesseract_rosutils::toMsg(res.kinematics_information, *manipulator_manager))
    {
      res.success = false;
      return false;
    }
  }

  if (req.flags & tesseract_msgs::GetEnvironmentInformationRequest::JOINT_STATES)
  {
    if (!tesseract_rosutils::toMsg(res.joint_states, env->getCurrentState()->joints))
    {
      res.success = false;
      return false;
    }
  }

  res.success = true;
  return true;
}

}  // namespace tesseract_monitoring
