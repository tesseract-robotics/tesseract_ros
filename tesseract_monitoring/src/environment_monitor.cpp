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
#include <ros/node_handle.h>
#include <boost/algorithm/string.hpp>
#include <thread>
#include <utility>
TESSERACT_COMMON_IGNORE_WARNINGS_POP

#include <tesseract_monitoring/environment_monitor.h>
#include <tesseract_monitoring/current_state_monitor.h>

#include <tesseract_msgs/EnvironmentState.h>
#include <tesseract_msgs/EnvironmentCommand.h>
#include <tesseract_msgs/ModifyEnvironment.h>
#include <tesseract_msgs/GetEnvironmentChanges.h>
#include <tesseract_msgs/GetEnvironmentInformation.h>
#include <tesseract_msgs/SaveSceneGraph.h>

#include <tesseract_scene_graph/scene_state.h>
#include <tesseract_scene_graph/graph.h>
#include <tesseract_environment/environment.h>
#include <tesseract_environment/command.h>
#include <tesseract_environment/utils.h>
#include <tesseract_rosutils/utils.h>

namespace tesseract_monitoring
{
struct ROSEnvironmentMonitor::Implementation
{
  ROSEnvironmentMonitor* parent;

  ros::Time last_update_time;        /// Last time the state was updated
  ros::Time last_robot_motion_time;  /// Last time the robot has moved
  bool enforce_next_state_update;    /// flag to enforce immediate state update in onStateUpdate()

  ros::NodeHandle nh;
  ros::NodeHandle root_nh;
  std::string robot_description;

  // variables for planning scene publishing
  ros::Publisher environment_publisher;
  std::unique_ptr<std::thread> publish_environment;
  double publish_environment_frequency;

  // variables for monitored environment
  ros::Subscriber monitored_environment_subscriber;
  ros::ServiceClient get_monitored_environment_changes_client;
  ros::ServiceClient get_monitored_environment_information_client;
  ros::ServiceClient modify_monitored_environment_client;

  // host a service for modifying the environment
  ros::ServiceServer modify_environment_server;

  // host a service for getting the environment changes
  ros::ServiceServer get_environment_changes_server;

  // host a service for getting the environment information
  ros::ServiceServer get_environment_information_server;

  // host a service for saving the scene graph to a DOT file
  ros::ServiceServer save_scene_graph_server;

  // include a current state monitor
  CurrentStateMonitor::UPtr current_state_monitor;

  // Lock for state_update_pending_ and dt_state_update_
  std::mutex state_pending_mutex;

  /// True when we need to update the RobotState from current_state_monitor_
  // This field is protected by state_pending_mutex_
  volatile bool state_update_pending;

  /// the amount of time to wait in between updates to the robot state
  // This field is protected by state_pending_mutex_
  ros::WallDuration dt_state_update;

  /// timer for state updates.
  // Check if last_state_update_ is true and if so call updateSceneWithCurrentState()
  // Not safe to access from callback functions.
  ros::WallTimer state_update_timer;

  /// Last time the state was updated from current_state_monitor_
  // Only access this from callback functions (and constructor)
  ros::WallTime last_robot_state_update_wall_time;

  /// Constructor
  Implementation(ROSEnvironmentMonitor* parent_) : parent(parent_), nh("~")
  {
    if (!initialize())
    {
      ROS_DEBUG("EnvironmentMonitor Tesseract Constructor failed to initialize!");
    }
  }

  Implementation(ROSEnvironmentMonitor* parent_, std::string robot_description_)
    : parent(parent_), nh("~"), robot_description(std::move(robot_description_))
  {
    // Initial setup
    std::string urdf_xml_string, srdf_xml_string;
    if (!root_nh.hasParam(robot_description))
    {
      ROS_ERROR("Failed to find parameter: %s", robot_description.c_str());
      return;
    }

    if (!root_nh.hasParam(robot_description + "_semantic"))
    {
      ROS_ERROR("Failed to find parameter: %s", (robot_description + "_semantic").c_str());
      return;
    }

    root_nh.getParam(robot_description, urdf_xml_string);
    root_nh.getParam(robot_description + "_semantic", srdf_xml_string);

    parent->env_ = std::make_shared<tesseract_environment::Environment>();
    auto locator = std::make_shared<tesseract_rosutils::ROSResourceLocator>();
    if (!parent->env_->init(urdf_xml_string, srdf_xml_string, locator))
      return;

    if (!initialize())
    {
      ROS_DEBUG("EnvironmentMonitor Robot Description Constructor failed to initialize!");
    }
  }

  /** @brief Initialize the planning scene monitor
   *  @param scene The scene instance to fill with data (an instance is allocated if the one passed in is not allocated)
   */
  bool initialize()
  {
    enforce_next_state_update = false;

    const std::string& monitor_namespace = parent->monitor_namespace_;
    if (monitor_namespace.empty())
      throw std::runtime_error("The monitor namespace cannot be empty!");

    if (!parent->env_->isInitialized())
    {
      ROS_DEBUG_NAMED(monitor_namespace, "Failed to initialize environment monitor, the tesseract is uninitialized!");
      return false;
    }

    publish_environment_frequency = 30.0;

    last_update_time = last_robot_motion_time = ros::Time::now();
    last_robot_state_update_wall_time = ros::WallTime::now();
    dt_state_update = ros::WallDuration(0.1);

    state_update_pending = false;
    state_update_timer = nh.createWallTimer(dt_state_update,
                                            &ROSEnvironmentMonitor::Implementation::updateJointStateTimerCallback,
                                            this,
                                            false,   // not a oneshot timer
                                            false);  // do not start the timer yet

    // Shutdown current services
    modify_environment_server.shutdown();
    get_environment_changes_server.shutdown();
    get_environment_information_server.shutdown();
    save_scene_graph_server.shutdown();

    // Create new service
    std::string modify_environment_server_name = R"(/)" + monitor_namespace + DEFAULT_MODIFY_ENVIRONMENT_SERVICE;
    std::string get_environment_changes_server_name =
        R"(/)" + monitor_namespace + DEFAULT_GET_ENVIRONMENT_CHANGES_SERVICE;
    std::string get_environment_information_server_name =
        R"(/)" + monitor_namespace + DEFAULT_GET_ENVIRONMENT_INFORMATION_SERVICE;
    std::string save_scene_graph_server_name = R"(/)" + monitor_namespace + DEFAULT_SAVE_SCENE_GRAPH_SERVICE;

    modify_environment_server = root_nh.advertiseService(
        modify_environment_server_name, &ROSEnvironmentMonitor::Implementation::modifyEnvironmentCallback, this);

    get_environment_changes_server =
        root_nh.advertiseService(get_environment_changes_server_name,
                                 &ROSEnvironmentMonitor::Implementation::getEnvironmentChangesCallback,
                                 this);

    get_environment_information_server =
        root_nh.advertiseService(get_environment_information_server_name,
                                 &ROSEnvironmentMonitor::Implementation::getEnvironmentInformationCallback,
                                 this);

    save_scene_graph_server = root_nh.advertiseService(
        save_scene_graph_server_name, &ROSEnvironmentMonitor::Implementation::saveSceneGraphCallback, this);

    return true;
  }

  void shutdown()
  {
    monitored_environment_subscriber.shutdown();
    get_monitored_environment_changes_client.shutdown();
    get_monitored_environment_information_client.shutdown();
    modify_monitored_environment_client.shutdown();
    modify_environment_server.shutdown();
    get_environment_changes_server.shutdown();
    get_environment_information_server.shutdown();
    save_scene_graph_server.shutdown();
  }

  void stopPublishingEnvironment()
  {
    if (publish_environment)
    {
      std::unique_ptr<std::thread> copy;
      copy.swap(publish_environment);
      copy->join();
      stopPublishingEnvironment();
      environment_publisher.shutdown();
      ROS_INFO_NAMED(parent->monitor_namespace_, "Stopped publishing maintained environment.");
    }
  }

  void startPublishingEnvironment()
  {
    if (!publish_environment && parent->env_->isInitialized())
    {
      const std::string& monitor_namespace = parent->monitor_namespace_;
      std::string environment_topic = R"(/)" + monitor_namespace + DEFAULT_PUBLISH_ENVIRONMENT_TOPIC;
      environment_publisher = nh.advertise<tesseract_msgs::EnvironmentState>(environment_topic, 100, false);
      ROS_INFO_NAMED(monitor_namespace, "Publishing maintained environment on '%s'", environment_topic.c_str());
      publish_environment = std::make_unique<std::thread>(
          std::bind(&ROSEnvironmentMonitor::Implementation::environmentPublishingThread, this));
    }
  }

  void stopMonitoringEnvironment()
  {
    get_monitored_environment_changes_client.shutdown();
    modify_monitored_environment_client.shutdown();
    get_monitored_environment_information_client.shutdown();
    monitored_environment_subscriber.shutdown();
    ROS_INFO_NAMED(parent->monitor_namespace_, "Stopped monitoring environment.");
  }

  // publish environment update diffs (runs in its own thread)
  void environmentPublishingThread()
  {
    const std::string& monitor_namespace = parent->monitor_namespace_;
    ROS_DEBUG_NAMED(monitor_namespace, "Started environment state publishing thread ...");

    // publish the full planning scene
    tesseract_msgs::EnvironmentState start_msg;
    tesseract_rosutils::toMsg(start_msg, *parent->env_);

    environment_publisher.publish(start_msg);
    ros::Duration(1.5).sleep();
    environment_publisher.publish(start_msg);

    ROS_DEBUG_NAMED(monitor_namespace, "Published the Tesseract Environment State for: '%s'", start_msg.id.c_str());

    do
    {
      tesseract_msgs::EnvironmentState msg;
      bool publish_msg = false;
      ros::Rate rate(publish_environment_frequency);
      {
        tesseract_environment::Environment& env = *parent->env_;
        auto lock_read = env.lockRead();
        tesseract_rosutils::toMsg(msg, env);

        // also publish timestamp of this robot_state
        msg.joint_state.header.stamp = last_robot_motion_time;
        publish_msg = true;
      }

      if (publish_msg)
      {
        rate.reset();
        environment_publisher.publish(msg);
        rate.sleep();
      }
    } while (publish_environment);
  }

  void startMonitoringEnvironment(const std::string& monitored_namespace,
                                  tesseract_environment::MonitoredEnvironmentMode mode)
  {
    parent->mode_ = mode;
    std::string monitored_environment_topic = R"(/)" + monitored_namespace + DEFAULT_PUBLISH_ENVIRONMENT_TOPIC;
    std::string monitored_environment_changes_service =
        R"(/)" + monitored_namespace + DEFAULT_GET_ENVIRONMENT_CHANGES_SERVICE;
    std::string monitored_environment_modify_service =
        R"(/)" + monitored_namespace + DEFAULT_MODIFY_ENVIRONMENT_SERVICE;
    std::string monitored_environment_information_service =
        R"(/)" + monitored_namespace + DEFAULT_GET_ENVIRONMENT_INFORMATION_SERVICE;

    stopMonitoringEnvironment();

    get_monitored_environment_changes_client =
        nh.serviceClient<tesseract_msgs::GetEnvironmentChanges>(monitored_environment_changes_service);
    modify_monitored_environment_client =
        nh.serviceClient<tesseract_msgs::ModifyEnvironment>(monitored_environment_modify_service);
    get_monitored_environment_information_client =
        nh.serviceClient<tesseract_msgs::GetEnvironmentInformation>(monitored_environment_information_service);

    monitored_environment_subscriber = nh.subscribe(
        monitored_environment_topic, 1000, &ROSEnvironmentMonitor::Implementation::newEnvironmentStateCallback, this);
    ROS_INFO_NAMED(
        parent->monitor_namespace_, "Monitoring external environment on '%s'", monitored_environment_topic.c_str());
  }

  double getStateUpdateFrequency() const
  {
    if (!dt_state_update.isZero())
      return 1.0 / dt_state_update.toSec();

    return 0.0;
  }

  bool waitForCurrentState(std::chrono::duration<double> duration)
  {
    if (std::chrono::duration_cast<std::chrono::seconds>(duration).count() == 0)
      return false;

    ros::Time t = ros::Time::now();
    ros::WallTime start = ros::WallTime::now();
    ros::WallDuration timeout(duration.count());

    const std::string& monitor_namespace = parent->monitor_namespace_;
    ROS_DEBUG_NAMED(monitor_namespace, "sync robot state to: %.3fs", fmod(duration.count(), 10.));

    if (current_state_monitor)
    {
      // Wait for next robot update in state monitor. Those updates are only
      // passed to PSM when robot actually moved!
      enforce_next_state_update = true;  // enforce potential updates to be directly applied
      bool success = current_state_monitor->waitForCurrentState(t, duration.count());
      enforce_next_state_update = false;  // back to normal throttling behavior,
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

      ROS_WARN_NAMED(monitor_namespace, "Failed to fetch current robot state.");
      return false;
    }

    // Sometimes there is no state monitor. In this case state updates are
    // received as part of scene updates only.
    // However, scene updates are only published if the robot actually moves.
    // Hence we need a timeout!
    // As publishing planning scene updates is throttled (2Hz by default), a 1s
    // timeout is a suitable default.
    ros::Time prev_robot_motion_time = last_robot_motion_time;
    while (last_robot_motion_time < t &&  // Wait until the state update actually reaches the scene.
           timeout > ros::WallDuration())
    {
      ROS_DEBUG_STREAM_NAMED(monitor_namespace,
                             "last robot motion: " << (t - last_robot_motion_time).toSec() << " ago");
      timeout -= ros::WallTime::now() - start;  // compute remaining wait_time
    }
    bool success = last_robot_motion_time >= t;
    // suppress warning if we received an update at all
    if (!success && prev_robot_motion_time != last_robot_motion_time)
      ROS_WARN_NAMED(monitor_namespace,
                     "Maybe failed to update robot state, time diff: %.3fs",
                     (t - last_robot_motion_time).toSec());

    ROS_DEBUG_STREAM_NAMED(monitor_namespace,
                           "sync done: robot motion: " << (t - last_robot_motion_time).toSec()
                                                       << " scene update: " << (t - last_update_time).toSec());
    return success;
  }

  void startStateMonitor(const std::string& joint_states_topic, bool publish_tf)
  {
    stopStateMonitor();
    if (parent->env_)
    {
      if (!current_state_monitor)
        current_state_monitor.reset(new CurrentStateMonitor(parent->env_, root_nh));

      current_state_monitor->addUpdateCallback(
          boost::bind(&ROSEnvironmentMonitor::Implementation::onJointStateUpdate, this, _1));
      current_state_monitor->startStateMonitor(joint_states_topic, publish_tf);

      {
        std::scoped_lock lock(state_pending_mutex);
        if (!dt_state_update.isZero())
          state_update_timer.start();
      }
    }
    else
    {
      ROS_ERROR_NAMED(parent->monitor_namespace_,
                      "Cannot monitor robot state because planning scene is not configured");
    }
  }

  void stopStateMonitor()
  {
    if (current_state_monitor)
      current_state_monitor->stopStateMonitor();

    // stop must be called with state_pending_mutex_ unlocked to avoid deadlock
    state_update_timer.stop();
    {
      std::scoped_lock lock(state_pending_mutex);
      state_update_pending = false;
    }
  }

  void setStateUpdateFrequency(double hz)
  {
    bool update = false;
    if (hz > std::numeric_limits<double>::epsilon())
    {
      std::scoped_lock lock(state_pending_mutex);
      dt_state_update.fromSec(1.0 / hz);
      state_update_timer.setPeriod(dt_state_update);
      state_update_timer.start();
    }
    else
    {
      // stop must be called with state_pending_mutex_ unlocked to avoid deadlock
      state_update_timer.stop();
      std::scoped_lock lock(state_pending_mutex);
      dt_state_update = ros::WallDuration(0, 0);
      if (state_update_pending)
        update = true;
    }
    ROS_INFO_NAMED(parent->monitor_namespace_,
                   "Updating internal planning scene state at most every %lf seconds",
                   dt_state_update.toSec());

    if (update)
      updateEnvironmentWithCurrentState();
  }

  void updateEnvironmentWithCurrentState()
  {
    const std::string& monitor_namespace = parent->monitor_namespace_;
    if (current_state_monitor)
    {
      std::vector<std::string> missing;
      if (!current_state_monitor->haveCompleteState(missing) &&
          (ros::Time::now() - current_state_monitor->getMonitorStartTime()).toSec() > 1.0)
      {
        std::string missing_str = boost::algorithm::join(missing, ", ");
        ROS_WARN_THROTTLE_NAMED(
            1, monitor_namespace, "The complete state of the robot is not yet known.  Missing %s", missing_str.c_str());
      }

      last_update_time = last_robot_motion_time = current_state_monitor->getCurrentStateTime();
      ROS_DEBUG_STREAM_NAMED(monitor_namespace, "robot state update " << fmod(last_robot_motion_time.toSec(), 10.));

      parent->env_->setState(current_state_monitor->getCurrentState().joints);
    }
    else
      ROS_ERROR_THROTTLE_NAMED(
          1, monitor_namespace, "State monitor is not active. Unable to set the planning scene state");
  }

  void setEnvironmentPublishingFrequency(double hz)
  {
    publish_environment_frequency = hz;
    ROS_DEBUG_NAMED(parent->monitor_namespace_,
                    "Maximum frquency for publishing an environment is now %lf Hz",
                    publish_environment_frequency);
  }

  // called by current_state_monitor_ when robot state (as monitored on joint state topic) changes
  void onJointStateUpdate(const sensor_msgs::JointStateConstPtr& /*joint_state*/)
  {
    const ros::WallTime& n = ros::WallTime::now();
    ros::WallDuration dt = n - last_robot_state_update_wall_time;

    bool update = enforce_next_state_update;
    {
      std::scoped_lock lock(state_pending_mutex);

      if (dt < dt_state_update && !update)
      {
        state_update_pending = true;
      }
      else
      {
        state_update_pending = false;
        last_robot_state_update_wall_time = n;
        update = true;
      }
    }
    // run the state update with state_pending_mutex_ unlocked
    if (update)
      updateEnvironmentWithCurrentState();
  }

  // called by state_update_timer_ when a state update it pending
  void updateJointStateTimerCallback(const ros::WallTimerEvent& event)
  {
    if (state_update_pending)
    {
      bool update = false;

      const ros::WallTime& n = ros::WallTime::now();
      ros::WallDuration dt = n - last_robot_state_update_wall_time;

      {
        // lock for access to dt_state_update_ and state_update_pending_
        std::scoped_lock lock(state_pending_mutex);
        if (state_update_pending && dt >= dt_state_update)
        {
          state_update_pending = false;
          last_robot_state_update_wall_time = ros::WallTime::now();
          update = true;
          ROS_DEBUG_STREAM_NAMED(parent->monitor_namespace_,
                                 "performPendingStateUpdate: " << fmod(last_robot_state_update_wall_time.toSec(), 10));
        }
      }

      // run the state update with state_pending_mutex_ unlocked
      if (update)
      {
        updateEnvironmentWithCurrentState();
        ROS_DEBUG_NAMED(parent->monitor_namespace_, "performPendingStateUpdate done");
      }
    }
  }

  // Callback for a new state msg
  void newEnvironmentStateCallback(const tesseract_msgs::EnvironmentStateConstPtr& env_msg)
  {
    last_update_time = ros::Time::now();
    const auto& monitor_namespace = parent->monitor_namespace_;
    if (!parent->env_->isInitialized())
    {
      tesseract_msgs::GetEnvironmentInformation res;
      res.request.flags = tesseract_msgs::GetEnvironmentInformationRequest::COMMAND_HISTORY |
                          tesseract_msgs::GetEnvironmentInformationRequest::KINEMATICS_INFORMATION;

      bool status = get_monitored_environment_information_client.call(res);
      if (!status || !res.response.success)
      {
        ROS_ERROR_STREAM_NAMED(monitor_namespace,
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
        ROS_ERROR_NAMED(
            monitor_namespace, "newEnvironmentStateCallback: Failed to convert command history message, %s!", e.what());
        return;
      }

      if (!parent->env_->init(commands))
      {
        ROS_ERROR_STREAM_NAMED(monitor_namespace, "newEnvironmentStateCallback: Failed to initialize environment!");
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
      if (static_cast<int>(env_msg->revision) > parent->env_->getRevision())
      {
        tesseract_msgs::GetEnvironmentChanges res;
        res.request.revision = static_cast<unsigned long>(parent->env_->getRevision());
        if (get_monitored_environment_changes_client.call(res))
        {
          if (!tesseract_rosutils::processMsg(*parent->env_, res.response.commands))
          {
            ROS_ERROR_STREAM_NAMED(monitor_namespace,
                                   "newEnvironmentStateCallback: Failed to apply monitored environments changes.");
          }
        }
        else
        {
          ROS_ERROR_STREAM_NAMED(monitor_namespace,
                                 "newEnvironmentStateCallback: Failed to get monitored environments changes.");
        }
      }
      else if (static_cast<int>(env_msg->revision) < parent->env_->getRevision())
      {
        if (parent->mode_ == tesseract_environment::MonitoredEnvironmentMode::DEFAULT)
        {
          // If the monitored environment has a lower revision it is reset and additional changes are requested and
          // applied.
          if (parent->env_->reset())
          {
            if (static_cast<int>(env_msg->revision) > parent->env_->getRevision())
            {
              tesseract_msgs::GetEnvironmentChanges res;
              res.request.revision = static_cast<unsigned long>(parent->env_->getRevision());
              if (get_monitored_environment_changes_client.call(res))
              {
                if (!tesseract_rosutils::processMsg(*parent->env_, res.response.commands))
                {
                  ROS_ERROR_STREAM_NAMED(monitor_namespace,
                                         "newEnvironmentStateCallback: Failed to apply monitored environments "
                                         "changes.");
                }
              }
              else
              {
                ROS_ERROR_STREAM_NAMED(monitor_namespace,
                                       "newEnvironmentStateCallback: Failed to get monitored environments changes.");
              }
            }
          }
          else
          {
            ROS_ERROR_STREAM_NAMED(monitor_namespace,
                                   "newEnvironmentStateCallback: Failed to reset the tesseract object!");
          }
        }
        else if (parent->mode_ == tesseract_environment::MonitoredEnvironmentMode::SYNCHRONIZED)
        {
          // If this has been modified it will push the changes to the monitored environment to keep them in sync
          tesseract_msgs::ModifyEnvironment res;
          res.request.id = parent->env_->getName();
          res.request.revision = env_msg->revision;
          if (tesseract_rosutils::toMsg(res.request.commands, parent->env_->getCommandHistory(), env_msg->revision))
          {
            bool status = modify_monitored_environment_client.call(res);
            if (!status || !res.response.success)
            {
              ROS_ERROR_STREAM_NAMED(monitor_namespace,
                                     "newEnvironmentStateCallback: Failed to update monitored environment!");
            }
          }
          else
          {
            ROS_ERROR_STREAM_NAMED(monitor_namespace,
                                   "newEnvironmentStateCallback: Failed to convert latest changes to message and "
                                   "update "
                                   "monitored environment!");
          }
        }
        else
        {
          ROS_ERROR_STREAM_NAMED(monitor_namespace,
                                 "newEnvironmentStateCallback: Unsupporte MonitoredEnvironmentMode!");
        }
      }
    }

    if (!tesseract_rosutils::isMsgEmpty(env_msg->joint_state) ||
        !tesseract_rosutils::isMsgEmpty(env_msg->floating_joint_states))
    {
      if (last_robot_motion_time != env_msg->joint_state.header.stamp)
      {
        tesseract_rosutils::processMsg(*parent->env_, env_msg->joint_state, env_msg->floating_joint_states);
        last_robot_motion_time = env_msg->joint_state.header.stamp;
      }
    }

    ROS_DEBUG_STREAM_NAMED(monitor_namespace,
                           "environment update " << fmod(last_update_time.toSec(), 10.)
                                                 << " robot stamp: " << fmod(last_robot_motion_time.toSec(), 10.));
  }

  /** @brief Callback for modifying the environment via service request */
  bool modifyEnvironmentCallback(tesseract_msgs::ModifyEnvironmentRequest& req,
                                 tesseract_msgs::ModifyEnvironmentResponse& res)
  {
    if (req.append)
      res.success = applyEnvironmentCommandsMessage(req.id, parent->env_->getRevision(), req.commands);
    else
      res.success = applyEnvironmentCommandsMessage(req.id, static_cast<int>(req.revision), req.commands);

    res.revision = static_cast<unsigned long>(parent->env_->getRevision());
    return res.success;
  }

  /** @brief Callback for get the environment changes via service request */
  bool getEnvironmentChangesCallback(tesseract_msgs::GetEnvironmentChangesRequest& req,
                                     tesseract_msgs::GetEnvironmentChangesResponse& res)
  {
    tesseract_environment::Environment& env = *parent->env_;
    auto lock_read = env.lockRead();

    if (static_cast<int>(req.revision) > env.getRevision())
    {
      res.success = false;
      return false;
    }

    res.id = env.getName();
    res.revision = static_cast<unsigned long>(env.getRevision());
    if (!tesseract_rosutils::toMsg(res.commands, env.getCommandHistory(), req.revision))
    {
      res.success = false;
      return false;
    }

    res.success = true;
    return res.success;
  }

  /** @brief Callback for get the environment information via service request */
  bool getEnvironmentInformationCallback(tesseract_msgs::GetEnvironmentInformationRequest& req,
                                         tesseract_msgs::GetEnvironmentInformationResponse& res)
  {
    tesseract_environment::Environment& env = *parent->env_;
    auto lock_read = env.lockRead();

    if (!env.isInitialized())
    {
      res.success = false;
      return false;
    }

    res.id = env.getName();
    res.revision = static_cast<unsigned long>(env.getRevision());

    if (req.flags & tesseract_msgs::GetEnvironmentInformationRequest::COMMAND_HISTORY)
    {
      if (!tesseract_rosutils::toMsg(res.command_history, env.getCommandHistory(), 0))
      {
        res.success = false;
        return false;
      }
    }

    if (req.flags & tesseract_msgs::GetEnvironmentInformationRequest::LINK_LIST)
    {
      for (const auto& link : env.getSceneGraph()->getLinks())
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
      for (const auto& joint : env.getSceneGraph()->getJoints())
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
      for (const auto& link : env.getLinkNames())
      {
        res.link_names.push_back(link);
      }
    }

    if (req.flags & tesseract_msgs::GetEnvironmentInformationRequest::JOINT_NAMES)
    {
      for (const auto& joint : env.getJointNames())
      {
        res.joint_names.push_back(joint);
      }
    }

    if (req.flags & tesseract_msgs::GetEnvironmentInformationRequest::ACTIVE_LINK_NAMES)
    {
      for (const auto& link : env.getActiveLinkNames())
      {
        res.active_link_names.push_back(link);
      }
    }

    if (req.flags & tesseract_msgs::GetEnvironmentInformationRequest::ACTIVE_JOINT_NAMES)
    {
      for (const auto& joint : env.getActiveJointNames())
      {
        res.active_joint_names.push_back(joint);
      }
    }

    tesseract_scene_graph::SceneState state = env.getState();
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
      if (!tesseract_rosutils::toMsg(res.allowed_collision_matrix, *env.getAllowedCollisionMatrix()))
      {
        res.success = false;
        return false;
      }
    }

    if (req.flags & tesseract_msgs::GetEnvironmentInformationRequest::KINEMATICS_INFORMATION)
    {
      if (!tesseract_rosutils::toMsg(res.kinematics_information, env.getKinematicsInformation()))
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

      if (!tesseract_rosutils::toMsg(res.floating_joint_states, state.floating_joints))
      {
        res.success = false;
        return false;
      }
    }

    res.success = true;
    return true;
  }

  /** @brief Callback to save the scene graph to a DOT via a service request */
  bool saveSceneGraphCallback(tesseract_msgs::SaveSceneGraphRequest& req, tesseract_msgs::SaveSceneGraphResponse& res)
  {
    res.success = false;
    if (parent->env_)
    {
      tesseract_environment::Environment& env = *parent->env_;
      auto lock = env.lockRead();
      res.success = true;
      env.getSceneGraph()->saveDOT(req.filepath);
      res.id = env.getName();
      res.revision = static_cast<unsigned long>(env.getRevision());
    }
    return true;
  }

  // Called when new service request is called to modify the environment.
  bool applyEnvironmentCommandsMessage(const std::string& id,
                                       int revision,
                                       const std::vector<tesseract_msgs::EnvironmentCommand>& commands)
  {
    if (parent->env_ == nullptr)
      return false;

    tesseract_environment::Environment& env = *parent->env_;
    if (id != env.getName() || revision != env.getRevision())
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

    if (!filtered_commands.empty())
      result = tesseract_rosutils::processMsg(env, filtered_commands);

    if (result)
    {
      for (const auto& cmd : update_joint_state_commands)
      {
        if (tesseract_rosutils::processMsg(env, cmd.joint_state, cmd.floating_joint_states))
        {
          last_robot_motion_time = ros::Time::now();
        }
        else
        {
          ROS_ERROR("Failed to apply UPDATE_JOINT_STATE command!");
          result = false;
        }
      }
    }

    return result;
  }
};

ROSEnvironmentMonitor::ROSEnvironmentMonitor(std::string robot_description, std::string monitor_namespace)
  : EnvironmentMonitor(std::move(monitor_namespace))
  , impl_(std::make_unique<Implementation>(this, std::move(robot_description)))
{
}

ROSEnvironmentMonitor::ROSEnvironmentMonitor(std::shared_ptr<tesseract_environment::Environment> env,
                                             std::string monitor_namespace)
  : EnvironmentMonitor(std::move(env), std::move(monitor_namespace)), impl_(std::make_unique<Implementation>(this))
{
}

ROSEnvironmentMonitor::~ROSEnvironmentMonitor()
{
  stopPublishingEnvironment();
  stopStateMonitor();

  impl_->current_state_monitor.reset();
  env_ = nullptr;

  shutdown();
}

const std::string& ROSEnvironmentMonitor::getURDFDescription() const { return impl_->robot_description; }

bool ROSEnvironmentMonitor::waitForConnection(std::chrono::duration<double> duration) const
{
  const ros::WallTime start_time = ros::WallTime::now();
  ros::WallDuration wall_timeout;
  if (std::chrono::duration_cast<std::chrono::seconds>(duration).count() == 0)
    wall_timeout = ros::WallDuration(-1);
  else
    wall_timeout = ros::WallDuration(duration.count());

  bool is_connected{ false };
  while (ros::ok())
  {
    is_connected = env_->isInitialized();
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

void ROSEnvironmentMonitor::shutdown() { impl_->shutdown(); }

void ROSEnvironmentMonitor::stopPublishingEnvironment() { impl_->stopPublishingEnvironment(); }

void ROSEnvironmentMonitor::startPublishingEnvironment() { impl_->startPublishingEnvironment(); }

double ROSEnvironmentMonitor::getEnvironmentPublishingFrequency() const { return impl_->publish_environment_frequency; }

void ROSEnvironmentMonitor::stopMonitoringEnvironment() { return impl_->stopMonitoringEnvironment(); }

const CurrentStateMonitor& ROSEnvironmentMonitor::getStateMonitor() const { return *impl_->current_state_monitor; }

CurrentStateMonitor& ROSEnvironmentMonitor::getStateMonitor() { return *impl_->current_state_monitor; }

void ROSEnvironmentMonitor::startMonitoringEnvironment(const std::string& monitored_namespace,
                                                       tesseract_environment::MonitoredEnvironmentMode mode)
{
  impl_->startMonitoringEnvironment(monitored_namespace, mode);
}

double ROSEnvironmentMonitor::getStateUpdateFrequency() const { return impl_->getStateUpdateFrequency(); }

bool ROSEnvironmentMonitor::waitForCurrentState(std::chrono::duration<double> duration)
{
  return impl_->waitForCurrentState(duration);
}

void ROSEnvironmentMonitor::startStateMonitor(const std::string& joint_states_topic, bool publish_tf)
{
  impl_->startStateMonitor(joint_states_topic, publish_tf);
}

void ROSEnvironmentMonitor::stopStateMonitor() { impl_->stopStateMonitor(); }

void ROSEnvironmentMonitor::setStateUpdateFrequency(double hz) { impl_->setStateUpdateFrequency(hz); }

void ROSEnvironmentMonitor::updateEnvironmentWithCurrentState() { impl_->updateEnvironmentWithCurrentState(); }

void ROSEnvironmentMonitor::setEnvironmentPublishingFrequency(double hz)
{
  impl_->setEnvironmentPublishingFrequency(hz);
}

}  // namespace tesseract_monitoring
