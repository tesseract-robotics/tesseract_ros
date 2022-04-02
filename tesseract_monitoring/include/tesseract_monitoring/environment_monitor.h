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
#include <tesseract_environment/environment.h>
#include <tesseract_environment/environment_monitor.h>
#include <tesseract_monitoring/current_state_monitor.h>
#include <tesseract_monitoring/environment_monitor.h>
#include <tesseract_monitoring/constants.h>
#include <tesseract_rosutils/utils.h>
#include <tesseract_scene_graph/graph.h>
#include <tesseract_urdf/urdf_parser.h>
#include <tesseract_kinematics/core/forward_kinematics.h>

namespace tesseract_monitoring
{
/**
 * @brief TesseractMonitor
 * Subscribes to the topic \e tesseract_environment */
class ROSEnvironmentMonitor : public tesseract_environment::EnvironmentMonitor
{
public:
  using Ptr = std::shared_ptr<ROSEnvironmentMonitor>;
  using ConstPtr = std::shared_ptr<const ROSEnvironmentMonitor>;
  using UPtr = std::unique_ptr<ROSEnvironmentMonitor>;
  using ConstUPtr = std::unique_ptr<const ROSEnvironmentMonitor>;

  /**
   * @brief Constructor
   * @param robot_description The name of the ROS parameter that contains the URDF (in string format)
   * @param monitor_namespace A name identifying this monitor, must be unique
   */
  ROSEnvironmentMonitor(std::string robot_description, std::string monitor_namespace);

  /**
   * @brief Constructor
   * @param env The environment
   * @param monitor_namespace A name identifying this monitor, must be unique
   */
  ROSEnvironmentMonitor(tesseract_environment::Environment::Ptr env, std::string monitor_namespace);

  ~ROSEnvironmentMonitor() override;
  ROSEnvironmentMonitor(const ROSEnvironmentMonitor&) = delete;
  ROSEnvironmentMonitor& operator=(const ROSEnvironmentMonitor&) = delete;
  ROSEnvironmentMonitor(ROSEnvironmentMonitor&&) = delete;
  ROSEnvironmentMonitor& operator=(ROSEnvironmentMonitor&&) = delete;

  /** @brief Get the stored robot description
   *  @return An instance of the stored robot description*/
  const std::string& getURDFDescription() const;

  bool waitForConnection(std::chrono::duration<double> duration = std::chrono::seconds(0)) const override final;

  void startPublishingEnvironment() override final;

  void stopPublishingEnvironment() override final;

  void setEnvironmentPublishingFrequency(double hz) override final;

  double getEnvironmentPublishingFrequency() const override final;

  void startStateMonitor(const std::string& joint_states_topic = DEFAULT_JOINT_STATES_TOPIC,
                         bool publish_tf = true) override final;

  void stopStateMonitor() override final;

  void setStateUpdateFrequency(double hz = 10) override final;

  double getStateUpdateFrequency() const override final;

  void updateEnvironmentWithCurrentState() override final;

  void startMonitoringEnvironment(const std::string& monitored_namespace,
                                  tesseract_environment::MonitoredEnvironmentMode mode =
                                      tesseract_environment::MonitoredEnvironmentMode::DEFAULT) override final;

  void stopMonitoringEnvironment() override final;

  bool waitForCurrentState(std::chrono::duration<double> duration = std::chrono::seconds(1)) override final;

  void shutdown() override final;

  /** @brief Get the stored instance of the stored current state monitor
   *  @return An instance of the stored current state monitor*/
  const CurrentStateMonitor& getStateMonitor() const;
  CurrentStateMonitor& getStateMonitor();

protected:
  /** @brief Initialize the planning scene monitor
   *  @param scene The scene instance to fill with data (an instance is allocated if the one passed in is not allocated)
   */
  bool initialize();

  ros::Time last_update_time_;        /// Last time the state was updated
  ros::Time last_robot_motion_time_;  /// Last time the robot has moved
  bool enforce_next_state_update_;    /// flag to enforce immediate state update in onStateUpdate()

  ros::NodeHandle nh_;
  ros::NodeHandle root_nh_;
  std::string robot_description_;

  // variables for planning scene publishing
  ros::Publisher environment_publisher_;
  std::unique_ptr<std::thread> publish_environment_;
  double publish_environment_frequency_;

  // variables for monitored environment
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
  CurrentStateMonitor::UPtr current_state_monitor_;

private:
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
