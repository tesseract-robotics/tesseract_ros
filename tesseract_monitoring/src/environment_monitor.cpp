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
TESSERACT_COMMON_IGNORE_WARNINGS_POP

#include <tesseract_monitoring/environment_monitor.h>
#include <tesseract_environment/utils.h>
#include <tesseract_scene_graph/utils.h>
#include <tesseract_rosutils/utils.h>

namespace tesseract_monitoring
{
EnvironmentMonitor::EnvironmentMonitor(const std::string& robot_description, std::string monitor_namespace)
  : monitor_namespace_(std::move(monitor_namespace)), nh_("~")
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

  env_ = std::make_shared<tesseract_environment::Environment>();
  auto locator = std::make_shared<tesseract_rosutils::ROSResourceLocator>();
  if (!env_->init(urdf_xml_string, srdf_xml_string, locator))
    return;

  if (!initialize())
  {
    ROS_DEBUG("EnvironmentMonitor Robot Description Constructor failed to initialize!");
  }
}

EnvironmentMonitor::EnvironmentMonitor(tesseract_environment::Environment::Ptr env, std::string monitor_namespace)
  : monitor_namespace_(std::move(monitor_namespace)), env_(std::move(env)), nh_("~")
{
  if (!initialize())
  {
    ROS_DEBUG("EnvironmentMonitor Tesseract Constructor failed to initialize!");
  }
}

EnvironmentMonitor::~EnvironmentMonitor()
{
  stopPublishingEnvironment();
  stopStateMonitor();

  current_state_monitor_.reset();
  env_ = nullptr;

  shutdown();
}

bool EnvironmentMonitor::waitForConnection(ros::Duration timeout) const
{
  const ros::WallTime start_time = ros::WallTime::now();
  const ros::WallDuration wall_timeout{ timeout.toSec() };
  bool is_connected{ false };
  while (ros::ok())
  {
    {
      auto lock = std::shared_lock(scene_update_mutex_);
      is_connected = env_->isInitialized();
    }
    if (is_connected)
      return true;

    if (wall_timeout >= ros::WallDuration(0))
    {
      const ros::WallTime current_time = ros::WallTime::now();
      if ((current_time - start_time) >= wall_timeout)
        return false;
    }

    ros::WallDuration(0.02).sleep();
    ros::spinOnce();
  }

  return false;
}

void EnvironmentMonitor::shutdown()
{
  monitored_environment_subscriber_.shutdown();
  get_monitored_environment_changes_client_.shutdown();
  get_monitored_environment_information_client_.shutdown();
  modify_monitored_environment_client_.shutdown();
  modify_environment_server_.shutdown();
  get_environment_changes_server_.shutdown();
  get_environment_information_server_.shutdown();
  save_scene_graph_server_.shutdown();
}

bool EnvironmentMonitor::initialize()
{
  enforce_next_state_update_ = false;

  if (monitor_namespace_.empty())
    throw std::runtime_error("The monitor namespace cannot be empty!");

  if (!env_->isInitialized())
  {
    ROS_DEBUG_NAMED(monitor_namespace_, "Failed to initialize environment monitor, the tesseract is uninitialized!");
    return false;
  }

  publish_environment_frequency_ = 30.0;

  last_update_time_ = last_robot_motion_time_ = ros::Time::now();
  last_robot_state_update_wall_time_ = ros::WallTime::now();
  dt_state_update_ = ros::WallDuration(0.1);

  state_update_pending_ = false;
  state_update_timer_ = nh_.createWallTimer(dt_state_update_,
                                            &EnvironmentMonitor::updateJointStateTimerCallback,
                                            this,
                                            false,   // not a oneshot timer
                                            false);  // do not start the timer yet

  // Shutdown current services
  modify_environment_server_.shutdown();
  get_environment_changes_server_.shutdown();
  get_environment_information_server_.shutdown();
  save_scene_graph_server_.shutdown();

  // Create new service
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

  return true;
}

const std::string& EnvironmentMonitor::getName() const { return monitor_namespace_; }

bool EnvironmentMonitor::applyCommand(const tesseract_environment::Command::ConstPtr& command)
{
  bool result = false;
  {
    auto lock = lockEnvironmentWrite();
    result = env_->applyCommand(command);
  }
  triggerEnvironmentUpdateEvent();
  return result;
}

bool EnvironmentMonitor::applyCommands(const tesseract_environment::Commands& commands)
{
  bool result = false;
  {
    auto lock = lockEnvironmentWrite();
    result = env_->applyCommands(commands);
  }
  triggerEnvironmentUpdateEvent();
  return result;
}

tesseract_scene_graph::SceneGraph::ConstPtr EnvironmentMonitor::getSceneGraph() const { return env_->getSceneGraph(); }

tesseract_srdf::KinematicsInformation EnvironmentMonitor::getKinematicsInformation() const
{
  return env_->getKinematicsInformation();
}

tesseract_environment::Environment& EnvironmentMonitor::getEnvironment() { return *env_; }

const tesseract_environment::Environment& EnvironmentMonitor::getEnvironment() const { return *env_; }

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

void EnvironmentMonitor::startPublishingEnvironment()
{
  if (!publish_environment_ && env_->isInitialized())
  {
    std::string environment_topic = R"(/)" + monitor_namespace_ + DEFAULT_PUBLISH_ENVIRONMENT_TOPIC;
    environment_publisher_ = nh_.advertise<tesseract_msgs::EnvironmentState>(environment_topic, 100, false);
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
  tesseract_msgs::EnvironmentState start_msg;
  tesseract_rosutils::toMsg(start_msg, *(env_));

  environment_publisher_.publish(start_msg);
  ros::Duration(1.5).sleep();
  environment_publisher_.publish(start_msg);

  ROS_DEBUG_NAMED(monitor_namespace_, "Published the Tesseract Environment State for: '%s'", start_msg.id.c_str());

  do
  {
    tesseract_msgs::EnvironmentState msg;
    bool publish_msg = false;
    ros::Rate rate(publish_environment_frequency_);
    {
      std::unique_lock<std::shared_mutex> ulock(scene_update_mutex_);
      tesseract_rosutils::toMsg(msg, *env_);

      // also publish timestamp of this robot_state
      msg.joint_state.header.stamp = last_robot_motion_time_;
      publish_msg = true;
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
  get_monitored_environment_changes_client_.shutdown();
  modify_monitored_environment_client_.shutdown();
  get_monitored_environment_information_client_.shutdown();
  monitored_environment_subscriber_.shutdown();
  ROS_INFO_NAMED(monitor_namespace_, "Stopped monitoring environment.");
}

const CurrentStateMonitor& EnvironmentMonitor::getStateMonitor() const { return *current_state_monitor_; }

CurrentStateMonitor& EnvironmentMonitor::getStateMonitor() { return *current_state_monitor_; }

void EnvironmentMonitor::startMonitoringEnvironment(const std::string& monitored_namespace,
                                                    MonitoredEnvironmentMode mode)
{
  monitored_environment_mode_ = mode;
  std::string monitored_environment_topic = R"(/)" + monitored_namespace + DEFAULT_PUBLISH_ENVIRONMENT_TOPIC;
  std::string monitored_environment_changes_service =
      R"(/)" + monitored_namespace + DEFAULT_GET_ENVIRONMENT_CHANGES_SERVICE;
  std::string monitored_environment_modify_service = R"(/)" + monitored_namespace + DEFAULT_MODIFY_ENVIRONMENT_SERVICE;
  std::string monitored_environment_information_service =
      R"(/)" + monitored_namespace + DEFAULT_GET_ENVIRONMENT_INFORMATION_SERVICE;

  stopMonitoringEnvironment();

  get_monitored_environment_changes_client_ =
      nh_.serviceClient<tesseract_msgs::GetEnvironmentChanges>(monitored_environment_changes_service);
  modify_monitored_environment_client_ =
      nh_.serviceClient<tesseract_msgs::ModifyEnvironment>(monitored_environment_modify_service);
  get_monitored_environment_information_client_ =
      nh_.serviceClient<tesseract_msgs::GetEnvironmentInformation>(monitored_environment_information_service);

  monitored_environment_subscriber_ =
      nh_.subscribe(monitored_environment_topic, 1000, &EnvironmentMonitor::newEnvironmentStateCallback, this);
  ROS_INFO_NAMED(monitor_namespace_, "Monitoring external environment on '%s'", monitored_environment_topic.c_str());
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

void EnvironmentMonitor::triggerEnvironmentUpdateEvent()
{
  // do not modify update functions while we are calling them
  std::scoped_lock<std::recursive_mutex> lock(update_lock_);

  for (auto& update_callback : update_callbacks_)
    update_callback();

  new_environment_update_condition_.notify_all();
}

void EnvironmentMonitor::newEnvironmentStateCallback(const tesseract_msgs::EnvironmentStateConstPtr& env)
{
  {
    std::unique_lock<std::shared_mutex> ulock(scene_update_mutex_);
    last_update_time_ = ros::Time::now();

    if (!env_->isInitialized())
    {
      tesseract_msgs::GetEnvironmentInformation res;
      res.request.flags = tesseract_msgs::GetEnvironmentInformationRequest::COMMAND_HISTORY |
                          tesseract_msgs::GetEnvironmentInformationRequest::KINEMATICS_INFORMATION;

      bool status = get_monitored_environment_information_client_.call(res);
      if (!status || !res.response.success)
      {
        ROS_ERROR_STREAM_NAMED(monitor_namespace_,
                               "newEnvironmentStateCallback: Failed to get monitor environment information!");
        return;
      }

      tesseract_environment::Commands commands;
      try
      {
        commands = tesseract_rosutils::fromMsg(res.response.command_history);
      }
      catch (const std::exception& e)
      {
        ROS_ERROR_NAMED(monitor_namespace_,
                        "newEnvironmentStateCallback: Failed to convert command history message, %s!",
                        e.what());
        return;
      }

      if (!env_->init(commands))
      {
        ROS_ERROR_STREAM_NAMED(monitor_namespace_, "newEnvironmentStateCallback: Failed to initialize environment!");
        return;
      }

      if (!initialize())
      {
        ROS_WARN("newEnvironmentStateCallback: EnvironmentMonitor Failed to initialize!");
      }
    }
    else
    {
      // If the monitored environment has changed then request the changes and apply
      if (static_cast<int>(env->revision) > env_->getRevision())
      {
        tesseract_msgs::GetEnvironmentChanges res;
        res.request.revision = static_cast<unsigned long>(env_->getRevision());
        if (get_monitored_environment_changes_client_.call(res))
        {
          if (!tesseract_rosutils::processMsg(*env_, res.response.commands))
          {
            ROS_ERROR_STREAM_NAMED(monitor_namespace_,
                                   "newEnvironmentStateCallback: Failed to apply monitored environments changes.");
          }
        }
        else
        {
          ROS_ERROR_STREAM_NAMED(monitor_namespace_,
                                 "newEnvironmentStateCallback: Failed to get monitored environments changes.");
        }
      }
      else if (static_cast<int>(env->revision) < env_->getRevision())
      {
        if (monitored_environment_mode_ == MonitoredEnvironmentMode::DEFAULT)
        {
          // If the monitored environment has a lower revision it is reset and additional changes are requested and
          // applied.
          if (env_->reset())
          {
            if (static_cast<int>(env->revision) > env_->getRevision())
            {
              tesseract_msgs::GetEnvironmentChanges res;
              res.request.revision = static_cast<unsigned long>(env_->getRevision());
              if (get_monitored_environment_changes_client_.call(res))
              {
                if (!tesseract_rosutils::processMsg(*env_, res.response.commands))
                {
                  ROS_ERROR_STREAM_NAMED(monitor_namespace_,
                                         "newEnvironmentStateCallback: Failed to apply monitored environments "
                                         "changes.");
                }
              }
              else
              {
                ROS_ERROR_STREAM_NAMED(monitor_namespace_,
                                       "newEnvironmentStateCallback: Failed to get monitored environments changes.");
              }
            }
          }
          else
          {
            ROS_ERROR_STREAM_NAMED(monitor_namespace_,
                                   "newEnvironmentStateCallback: Failed to reset the tesseract object!");
          }
        }
        else if (monitored_environment_mode_ == MonitoredEnvironmentMode::SYNCHRONIZED)
        {
          // If this has been modified it will push the changes to the monitored environment to keep them in sync
          tesseract_msgs::ModifyEnvironment res;
          res.request.id = env_->getName();
          res.request.revision = env->revision;
          if (tesseract_rosutils::toMsg(res.request.commands, env_->getCommandHistory(), env->revision))
          {
            bool status = modify_monitored_environment_client_.call(res);
            if (!status || !res.response.success)
            {
              ROS_ERROR_STREAM_NAMED(monitor_namespace_,
                                     "newEnvironmentStateCallback: Failed to update monitored environment!");
            }
          }
          else
          {
            ROS_ERROR_STREAM_NAMED(monitor_namespace_,
                                   "newEnvironmentStateCallback: Failed to convert latest changes to message and "
                                   "update "
                                   "monitored environment!");
          }
        }
        else
        {
          ROS_ERROR_STREAM_NAMED(monitor_namespace_,
                                 "newEnvironmentStateCallback: Unsupporte MonitoredEnvironmentMode!");
        }
      }
    }

    if (!tesseract_rosutils::isMsgEmpty(env->joint_state))
    {
      if (last_robot_motion_time_ != env->joint_state.header.stamp)
      {
        tesseract_rosutils::processMsg(*env_, env->joint_state);
        last_robot_motion_time_ = env->joint_state.header.stamp;
      }
    }

    ROS_DEBUG_STREAM_NAMED(monitor_namespace_,
                           "environment update " << fmod(last_update_time_.toSec(), 10.)
                                                 << " robot stamp: " << fmod(last_robot_motion_time_.toSec(), 10.));
  }

  triggerEnvironmentUpdateEvent();
}

bool EnvironmentMonitor::applyEnvironmentCommandsMessage(
    const std::string& id,
    int revision,
    const std::vector<tesseract_msgs::EnvironmentCommand>& commands)
{
  if (!env_ || id != env_->getName() || revision != env_->getRevision())
    return false;

  bool result = true;

  // Update joint state is not a tracked command so need to filter them out.
  std::vector<tesseract_msgs::EnvironmentCommand> filtered_commands;
  std::vector<tesseract_msgs::EnvironmentCommand> update_joint_state_commands;
  for (const auto& cmd : commands)
  {
    if (cmd.command == tesseract_msgs::EnvironmentCommand::UPDATE_JOINT_STATE)
      update_joint_state_commands.push_back(cmd);
    else
      filtered_commands.push_back(cmd);
  }

  std::string old_scene_name;
  {
    std::unique_lock<std::shared_mutex> ulock(scene_update_mutex_);
    if (!filtered_commands.empty())
      result = tesseract_rosutils::processMsg(*env_, filtered_commands);

    if (result)
    {
      for (const auto& cmd : update_joint_state_commands)
      {
        if (tesseract_rosutils::processMsg(*env_, cmd.joint_state))
        {
          last_robot_motion_time_ = ros::Time::now();
        }
        else
        {
          ROS_ERROR("Failed to apply UPDATE_JOINT_STATE command!");
          result = false;
        }
      }
    }
  }

  triggerEnvironmentUpdateEvent();
  return result;
}

bool EnvironmentMonitor::saveSceneGraphCallback(tesseract_msgs::SaveSceneGraphRequest& req,
                                                tesseract_msgs::SaveSceneGraphResponse& res)
{
  res.success = !(env_ == nullptr);
  env_->getSceneGraph()->saveDOT(req.filepath);
  res.id = env_->getName();
  res.revision = static_cast<unsigned long>(env_->getRevision());

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

std::shared_lock<std::shared_mutex> EnvironmentMonitor::lockEnvironmentRead() const
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
  if (env_)
  {
    if (!current_state_monitor_)
      current_state_monitor_.reset(new CurrentStateMonitor(env_, root_nh_));

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

      env_->setState(current_state_monitor_->getCurrentState().joints);
    }
    triggerEnvironmentUpdateEvent();
  }
  else
    ROS_ERROR_THROTTLE_NAMED(
        1, monitor_namespace_, "State monitor is not active. Unable to set the planning scene state");
}

void EnvironmentMonitor::addUpdateCallback(const std::function<void()>& fn)
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
  if (req.append)
    res.success = applyEnvironmentCommandsMessage(req.id, env_->getRevision(), req.commands);
  else
    res.success = applyEnvironmentCommandsMessage(req.id, static_cast<int>(req.revision), req.commands);

  res.revision = static_cast<unsigned long>(env_->getRevision());
  return res.success;
}

bool EnvironmentMonitor::getEnvironmentChangesCallback(tesseract_msgs::GetEnvironmentChangesRequest& req,
                                                       tesseract_msgs::GetEnvironmentChangesResponse& res)
{
  auto lock_read = lockEnvironmentRead();

  if (static_cast<int>(req.revision) > env_->getRevision())
  {
    res.success = false;
    return false;
  }

  res.id = env_->getName();
  res.revision = static_cast<unsigned long>(env_->getRevision());
  if (!tesseract_rosutils::toMsg(res.commands, env_->getCommandHistory(), req.revision))
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

  if (!env_->isInitialized())
  {
    res.success = false;
    return false;
  }

  res.id = env_->getName();
  res.revision = static_cast<unsigned long>(env_->getRevision());

  if (req.flags & tesseract_msgs::GetEnvironmentInformationRequest::COMMAND_HISTORY)
  {
    if (!tesseract_rosutils::toMsg(res.command_history, env_->getCommandHistory(), 0))
    {
      res.success = false;
      return false;
    }
  }

  if (req.flags & tesseract_msgs::GetEnvironmentInformationRequest::LINK_LIST)
  {
    for (const auto& link : env_->getSceneGraph()->getLinks())
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
    for (const auto& joint : env_->getSceneGraph()->getJoints())
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
    for (const auto& link : env_->getLinkNames())
    {
      res.link_names.push_back(link);
    }
  }

  if (req.flags & tesseract_msgs::GetEnvironmentInformationRequest::JOINT_NAMES)
  {
    for (const auto& joint : env_->getJointNames())
    {
      res.joint_names.push_back(joint);
    }
  }

  if (req.flags & tesseract_msgs::GetEnvironmentInformationRequest::ACTIVE_LINK_NAMES)
  {
    for (const auto& link : env_->getActiveLinkNames())
    {
      res.active_link_names.push_back(link);
    }
  }

  if (req.flags & tesseract_msgs::GetEnvironmentInformationRequest::ACTIVE_JOINT_NAMES)
  {
    for (const auto& joint : env_->getActiveJointNames())
    {
      res.active_joint_names.push_back(joint);
    }
  }

  tesseract_scene_graph::SceneState state = env_->getState();
  if (req.flags & tesseract_msgs::GetEnvironmentInformationRequest::LINK_TRANSFORMS)
  {
    for (const auto& link_pair : state.link_transforms)
    {
      res.link_transforms.names.push_back(link_pair.first);
      geometry_msgs::Pose pose;
      tesseract_rosutils::toMsg(pose, link_pair.second);
      res.link_transforms.transforms.push_back(pose);
    }
  }

  if (req.flags & tesseract_msgs::GetEnvironmentInformationRequest::JOINT_TRANSFORMS)
  {
    for (const auto& joint_pair : state.joint_transforms)
    {
      res.joint_transforms.names.push_back(joint_pair.first);
      geometry_msgs::Pose pose;
      tesseract_rosutils::toMsg(pose, joint_pair.second);
      res.joint_transforms.transforms.push_back(pose);
    }
  }

  if (req.flags & tesseract_msgs::GetEnvironmentInformationRequest::ALLOWED_COLLISION_MATRIX)
  {
    if (!tesseract_rosutils::toMsg(res.allowed_collision_matrix, *env_->getAllowedCollisionMatrix()))
    {
      res.success = false;
      return false;
    }
  }

  if (req.flags & tesseract_msgs::GetEnvironmentInformationRequest::KINEMATICS_INFORMATION)
  {
    if (!tesseract_rosutils::toMsg(res.kinematics_information, env_->getKinematicsInformation()))
    {
      res.success = false;
      return false;
    }
  }

  if (req.flags & tesseract_msgs::GetEnvironmentInformationRequest::JOINT_STATES)
  {
    if (!tesseract_rosutils::toMsg(res.joint_states, state.joints))
    {
      res.success = false;
      return false;
    }
  }

  res.success = true;
  return true;
}

}  // namespace tesseract_monitoring
