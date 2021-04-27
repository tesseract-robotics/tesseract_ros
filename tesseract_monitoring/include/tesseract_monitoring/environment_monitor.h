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

#ifndef TESSERACT_MONITORING_ENVIRONMENT_H
#define TESSERACT_MONITORING_ENVIRONMENT_H

#include <tesseract_common/macros.h>
TESSERACT_COMMON_IGNORE_WARNINGS_PUSH
#include <pluginlib/class_loader.hpp>
#include <ros/ros.h>
#include <ros/service_client.h>
#include <message_filters/subscriber.h>
#include <memory>
#include <shared_mutex>
#include <mutex>
#include <condition_variable>
#include <thread>
#include <functional>
#include <tesseract_msgs/EnvironmentState.h>
#include <tesseract_msgs/ModifyEnvironment.h>
#include <tesseract_msgs/GetEnvironmentChanges.h>
#include <tesseract_msgs/GetEnvironmentInformation.h>
#include <tesseract_msgs/SaveSceneGraph.h>
TESSERACT_COMMON_IGNORE_WARNINGS_POP

#include <tesseract_collision/core/discrete_contact_manager.h>
#include <tesseract_collision/core/continuous_contact_manager.h>
#include <tesseract_environment/core/environment.h>
#include <tesseract_monitoring/current_state_monitor.h>
#include <tesseract_monitoring/environment_monitor.h>
#include <tesseract_monitoring/constants.h>
#include <tesseract_rosutils/utils.h>
#include <tesseract_scene_graph/graph.h>
#include <tesseract_urdf/urdf_parser.h>
#include <tesseract_kinematics/core/forward_kinematics.h>

namespace tesseract_monitoring
{
using DiscreteContactManagerPluginLoader = pluginlib::ClassLoader<tesseract_collision::DiscreteContactManager>;
using DiscreteContactManagerPluginLoaderPtr = std::shared_ptr<DiscreteContactManagerPluginLoader>;

using ContinuousContactManagerPluginLoader = pluginlib::ClassLoader<tesseract_collision::ContinuousContactManager>;
using ContinuousContactManagerPluginLoaderPtr = std::shared_ptr<ContinuousContactManagerPluginLoader>;

enum class MonitoredEnvironmentMode : int
{
  /**
   * @brief The default behavior when monitoring another environment is the following.
   *
   * Case1: If the revision is greater than, it will call a service of the monitored environment to get the changes and
   *        apply them.
   * Case2: If the revision is less than, it will reinitialize the environment and request the remaining changes and
   *        apply them.
   */
  DEFAULT = 0,

  /**
   * @brief The synchronized behavior when monitoring another environment is the following.
   *
   * @warning Currently this works best if there is only one monitor which is sychronized becuase currently if two
   *          environment monitors are synchronized, it currently does not have a way to reason about which ones
   *          should get applied. Need to research approach for this type of system.
   *
   * Case1: If the revision is greater than, it will call a service of the monitored environment to get the changes and
   *        apply them.
   * Case2: If the revision is less than, it will call a service of the monitored environment to apply the new changes.
   */
  SYNCHRONIZED = 1
};

/**
 * @brief TesseractMonitor
 * Subscribes to the topic \e tesseract_environment */
class EnvironmentMonitor
{
public:
  using Ptr = std::shared_ptr<EnvironmentMonitor>;
  using ConstPtr = std::shared_ptr<const EnvironmentMonitor>;

  /** @brief Constructor
   *  @param robot_description The name of the ROS parameter that contains the URDF (in string format)
   *  @param monitor_namespace A name identifying this monitor, must be unique
   */
  EnvironmentMonitor(const std::string& robot_description,
                     std::string monitor_namespace,
                     std::string discrete_plugin = "",
                     std::string continuous_plugin = "");

  /** @brief Constructor
   *  @param monitor_namespace A name identifying this monitor, must be unique
   */
  EnvironmentMonitor(tesseract_environment::Environment::Ptr env,
                     std::string monitor_namespace,
                     std::string discrete_plugin = "",
                     std::string continuous_plugin = "");

  ~EnvironmentMonitor();
  EnvironmentMonitor(const EnvironmentMonitor&) = delete;
  EnvironmentMonitor& operator=(const EnvironmentMonitor&) = delete;
  EnvironmentMonitor(EnvironmentMonitor&&) = delete;
  EnvironmentMonitor& operator=(EnvironmentMonitor&&) = delete;

  /** \brief Get the name of this monitor */
  const std::string& getName() const;

  /**
   * @brief Wait for connection to upstream environment
   * @param timeout The duration to wait before returning, if not timeout is provided it waits indefinitely
   * @return True if it has connected to upstream environment, otherwise false
   */
  bool waitForConnection(ros::Duration timeout = ros::Duration(-1)) const;

  /**
   * @brief Apply provided command to the environment owned by this monitor
   *
   * @details This should only be used if this monitor is the master. If this is monitoring another environment the
   * local changes will get removed on the next update cycle. Recommend using the TesseractMonitorInterface to apply
   * commands to the monitored environment until the todo below is implemented.
   *
   * @todo Add similar commands applyCommandUpstream
   *
   * @param command The command to apply
   * @return True if successful, otherwise false
   */
  bool applyCommand(const tesseract_environment::Command::ConstPtr& command);
  bool applyCommands(const tesseract_environment::Commands& commands);

  /**
   * @brief Get the scene graph
   * @return Const SceneGraph
   */
  tesseract_scene_graph::SceneGraph::ConstPtr getSceneGraph() const;

  /**
   * @brief Get the kinematics information
   * @return Const KinematicsInformatiion
   */
  const tesseract_srdf::KinematicsInformation& getKinematicsInformation() const;

  /**
   * @brief Returns an @b unsafe pointer to the current environment.
   * @warning TesseractMonitor has a background thread which repeatedly updates and clobbers various contents of its
   *          internal tesseract instance.  This function just returns a pointer to that dynamic internal object.
   *          The correct thing is to call lockEnvironmentRead or lockEnvironmentWrite before accessing the contents.
   * @see lockEnvironmentRead
   * @see lockEnvironmentWrite
   * @return A pointer to the current environment.*/
  tesseract_environment::Environment::Ptr getEnvironment();

  /**
   * @brief Returns an @b unsafe const pointer to the current environment.
   * @warning TesseractMonitor has a background thread which repeatedly updates and clobbers various contents of its
   *          internal tesseract instance.  This function just returns a pointer to that dynamic internal object.
   *          The correct thing is to call lockEnvironmentRead before accessing the contents.
   * @see lockEnvironmentRead
   * @return A pointer to the current environment.*/
  tesseract_environment::Environment::ConstPtr getEnvironment() const;

  /** @brief Return true if the scene \e scene can be updated directly
      or indirectly by this monitor. This function will return true if
      the pointer of the scene is the same as the one maintained,
      or if a parent of the scene is the one maintained. */
  bool updatesEnvironment(const tesseract_environment::Environment::ConstPtr& env) const;

  /** @brief Return true if the scene \e scene can be updated directly
      or indirectly by this monitor. This function will return true if
      the pointer of the scene is the same as the one maintained,
      or if a parent of the scene is the one maintained. */
  bool updatesEnvironment(const tesseract_environment::Environment::Ptr& env) const;

  /** @brief Get the stored robot description
   *  @return An instance of the stored robot description*/
  const std::string& getURDFDescription() const { return robot_description_; }

  /** \brief Start publishing the maintained environment.*/
  void startPublishingEnvironment();

  /** \brief Stop publishing the maintained environment. */
  void stopPublishingEnvironment();

  /** \brief Set the maximum frequency at which environment are being published */
  void setEnvironmentPublishingFrequency(double hz);

  /** \brief Get the maximum frequency at which environment are published (Hz) */
  double getEnvironmentPublishingFrequency() const;

  /** @brief Get the stored instance of the stored current state monitor
   *  @return An instance of the stored current state monitor*/
  CurrentStateMonitor::ConstPtr getStateMonitor() const;
  CurrentStateMonitor::Ptr getStateMonitor();

  /** @brief Start the current state monitor
      @param joint_states_topic the topic to listen to for joint states
      @param publish_tf If true, TFs will be published for each joint (similar to robot description publisher). Default:
     true  */
  void startStateMonitor(const std::string& joint_states_topic = DEFAULT_JOINT_STATES_TOPIC, bool publish_tf = true);

  /** @brief Stop the state monitor*/
  void stopStateMonitor();

  /** @brief Update the scene using the monitored state. This function is automatically called when an update to the
     current state is received (if startStateMonitor() has been called).
      The updates are throttled to a maximum update frequency however, which is set by setStateUpdateFrequency(). */
  void updateEnvironmentWithCurrentState();

  /** @brief Update the scene using the monitored state at a specified frequency, in Hz. This function has an effect
     only when updates from the CurrentStateMonitor are received at a higher frequency.
      In that case, the updates are throttled down, so that they do not exceed a maximum update frequency specified
     here.
      @param hz the update frequency. By default this is 10Hz. */
  void setStateUpdateFrequency(double hz);

  /** @brief Get the maximum frequency (Hz) at which the current state of the planning scene is updated.*/
  double getStateUpdateFrequency() const;

  /**
   * @brief Start the monitoring of an environment topic
   * @param monitored_namespace The namespace of the environment to monitor
   */
  void startMonitoringEnvironment(const std::string& monitored_namespace,
                                  MonitoredEnvironmentMode mode = MonitoredEnvironmentMode::DEFAULT);

  /** \brief Stop monitoring the external environment. */
  void stopMonitoringEnvironment();

  /** @brief Add a function to be called when an update to the scene is received */
  void addUpdateCallback(const std::function<void()>& fn);

  /** @brief Clear the functions to be called when an update to the scene is received */
  void clearUpdateCallbacks();

  /** \brief Return the time when the last update was made to the planning scene (by \e any monitor) */
  const ros::Time& getLastUpdateTime() const { return last_update_time_; }

  /** @brief This function is called every time there is a change to the planning scene */
  void triggerEnvironmentUpdateEvent();

  /** \brief Wait for robot state to become more recent than time t.
   *
   * If there is no state monitor active, there will be no scene updates.
   * Hence, you can specify a timeout to wait for those updates. Default is 1s.
   */
  bool waitForCurrentState(const ros::Time& t, double wait_time = 1.);

  /** \brief Lock the scene for reading (multiple threads can lock for reading at the same time) */
  std::shared_lock<std::shared_mutex> lockEnvironmentRead();

  /** \brief Lock the scene for writing (only one thread can lock for writing and no other thread can lock for reading)
   */
  std::unique_lock<std::shared_mutex> lockEnvironmentWrite();

  void clearOctomap();

  void getStateMonitoredTopics(std::vector<std::string>& topics) const;

  /** @brief Shutdown advertised services */
  void shutdown();

protected:
  /** @brief Initialize the planning scene monitor
   *  @param scene The scene instance to fill with data (an instance is allocated if the one passed in is not allocated)
   */
  bool initialize();

  /// The name of this scene monitor
  std::string monitor_namespace_;
  std::string discrete_plugin_name_;
  std::string continuous_plugin_name_;

  DiscreteContactManagerPluginLoaderPtr discrete_manager_loader_;
  ContinuousContactManagerPluginLoaderPtr continuous_manager_loader_;

  tesseract_environment::Environment::Ptr env_;
  mutable std::shared_mutex scene_update_mutex_;  /// mutex for stored scene
  ros::Time last_update_time_;                    /// Last time the state was updated
  ros::Time last_robot_motion_time_;              /// Last time the robot has moved
  bool enforce_next_state_update_;                /// flag to enforce immediate state update in onStateUpdate()

  ros::NodeHandle nh_;
  ros::NodeHandle root_nh_;
  std::string robot_description_;

  // variables for planning scene publishing
  ros::Publisher environment_publisher_;
  std::unique_ptr<std::thread> publish_environment_;
  double publish_environment_frequency_;
  std::condition_variable_any new_environment_update_condition_;

  // variables for monitored environment
  MonitoredEnvironmentMode monitored_environment_mode_;
  ros::Subscriber monitored_environment_subscriber_;
  ros::ServiceClient get_monitored_environment_changes_client_;
  ros::ServiceClient get_monitored_environment_information_client_;
  ros::ServiceClient modify_monitored_environment_client_;

  // host a service for modifying the environment
  ros::ServiceServer modify_environment_server_;

  // host a service for getting the environment changes
  ros::ServiceServer get_environment_changes_server_;

  // host a service for getting the environment information
  ros::ServiceServer get_environment_information_server_;

  // host a service for saving the scene graph to a DOT file
  ros::ServiceServer save_scene_graph_server_;

  // include a current state monitor
  CurrentStateMonitor::Ptr current_state_monitor_;

  /// lock access to update_callbacks_
  std::recursive_mutex update_lock_;

  /// List of callbacks to trigger when updates are received
  std::vector<std::function<void()> > update_callbacks_;

private:
  void getUpdatedFrameTransforms(std::vector<geometry_msgs::TransformStamped>& transforms);

  // publish environment update diffs (runs in its own thread)
  void environmentPublishingThread();

  // called by current_state_monitor_ when robot state (as monitored on joint state topic) changes
  void onJointStateUpdate(const sensor_msgs::JointStateConstPtr& joint_state);

  // called by state_update_timer_ when a state update it pending
  void updateJointStateTimerCallback(const ros::WallTimerEvent& event);

  // Callback for a new state msg
  void newEnvironmentStateCallback(const tesseract_msgs::EnvironmentStateConstPtr& env);

  /** @brief Callback for modifying the environment via service request */
  bool modifyEnvironmentCallback(tesseract_msgs::ModifyEnvironmentRequest& req,
                                 tesseract_msgs::ModifyEnvironmentResponse& res);

  /** @brief Callback for get the environment changes via service request */
  bool getEnvironmentChangesCallback(tesseract_msgs::GetEnvironmentChangesRequest& req,
                                     tesseract_msgs::GetEnvironmentChangesResponse& res);

  /** @brief Callback for get the environment information via service request */
  bool getEnvironmentInformationCallback(tesseract_msgs::GetEnvironmentInformationRequest& req,
                                         tesseract_msgs::GetEnvironmentInformationResponse& res);

  /** @brief Callback to save the scene graph to a DOT via a service request */
  bool saveSceneGraphCallback(tesseract_msgs::SaveSceneGraphRequest& req, tesseract_msgs::SaveSceneGraphResponse& res);

  // Called when new service request is called to modify the environment.
  bool applyEnvironmentCommandsMessage(const std::string& id,
                                       int revision,
                                       const std::vector<tesseract_msgs::EnvironmentCommand>& commands);

  // Lock for state_update_pending_ and dt_state_update_
  std::mutex state_pending_mutex_;

  /// True when we need to update the RobotState from current_state_monitor_
  // This field is protected by state_pending_mutex_
  volatile bool state_update_pending_;

  /// the amount of time to wait in between updates to the robot state
  // This field is protected by state_pending_mutex_
  ros::WallDuration dt_state_update_;

  /// the amount of time to wait when looking up transforms
  // Setting this to a non-zero value resolves issues when the sensor data is
  // arriving so fast that it is preceding the transform state.
  ros::Duration shape_transform_cache_lookup_wait_time_;

  /// timer for state updates.
  // Check if last_state_update_ is true and if so call updateSceneWithCurrentState()
  // Not safe to access from callback functions.
  ros::WallTimer state_update_timer_;

  /// Last time the state was updated from current_state_monitor_
  // Only access this from callback functions (and constructor)
  ros::WallTime last_robot_state_update_wall_time_;
};

}  // namespace tesseract_monitoring

#endif
