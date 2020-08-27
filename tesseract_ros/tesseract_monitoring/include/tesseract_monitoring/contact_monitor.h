/**
 * @file contact_monitor.h
 * @brief definition of the contact_monitor library.  It publishes
 * info about which links are (almost) in collision, and how far from/in
 * collision they are.
 *
 * @author David Merz, Jr.
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

#ifndef CONTACT_MONITOR_H
#define CONTACT_MONITOR_H

#include <tesseract_common/macros.h>
TESSERACT_COMMON_IGNORE_WARNINGS_PUSH
#include <ros/ros.h>
#include <sensor_msgs/JointState.h>
#include <tesseract_msgs/ComputeContactResultVector.h>
#include <tesseract_msgs/ModifyEnvironment.h>
#include <tesseract_msgs/TesseractState.h>
TESSERACT_COMMON_IGNORE_WARNINGS_POP

#include <boost/thread/mutex.hpp>
#include <boost/thread/thread.hpp>  // boost::thread and boost::condition_variable

#include <tesseract_collision/core/discrete_contact_manager.h>
#include <tesseract/tesseract.h>

namespace contact_monitor
{
class ContactMonitor
{
public:
  /// The name of the topic used by default for receiving joint states
  static const std::string DEFAULT_JOINT_STATES_TOPIC;  // "/joint_states"

  /// The name of the topic used by default for publishing the monitored tesseract environment (this is without "/" in
  /// the name, so the topic is prefixed by the node name)
  static const std::string DEFAULT_PUBLISH_ENVIRONMENT_TOPIC;  // "/tesseract_published_environment"

  // The name of the topic this should listen to for updates
  static const std::string DEFAULT_MONITOR_ENVIRONMENT_TOPIC;  // "/tesseract_monitored_environment

  /// The name of the service used by default for setting the full tesseract environment state
  static const std::string DEFAULT_MODIFY_ENVIRONMENT_SERVICE;  // "/modify_tesseract"

  /// The name of the topic used by default for publishing the monitored contact results (this is without "/" in
  /// the name, so the topic is prefixed by the node name)
  static const std::string DEFAULT_PUBLISH_CONTACT_RESULTS_TOPIC;  // "/contact_results"

  /// The name of the topic used by default for publishing the monitored contact results markers (this is without "/" in
  /// the name, so the topic is prefixed by the node name)
  static const std::string DEFAULT_PUBLISH_CONTACT_MARKER_TOPIC;  // "/contact_results_markers"

  /// The name of the service used by default for computing the contact results
  static const std::string DEFAULT_COMPUTE_CONTACT_RESULTS_SERVICE;  // "/compute_contact_results"

  ContactMonitor(const tesseract::Tesseract::Ptr& tess,
                 ros::NodeHandle& nh,
                 ros::NodeHandle& pnh,
                 const std::vector<std::string>& monitored_link_names,
                 const tesseract_collision::ContactTestType& type,
                 double contact_distance = 0.1,
                 const std::string& joint_state_topic = DEFAULT_JOINT_STATES_TOPIC);
  ~ContactMonitor();
  /**
   * @brief Custom copy constructor, copy assignment, move constructor, and move
   * assignment.  Because the condition vairable current_joint_states_evt_
   * requires a 'cleanup' function call, a custom destructor is required.
   * When a custom destructor is required, it is best to explicitly create
   * these functions.  Here, we do so by assigning them to default values.
   */
  ContactMonitor(const ContactMonitor&) = delete;
  ContactMonitor& operator=(const ContactMonitor&) = delete;
  ContactMonitor(ContactMonitor&&) = delete;
  ContactMonitor& operator=(ContactMonitor&&) = delete;

  /**
   * @brief Start publishing the contact monitors environment
   * @param topic The topic to publish the contact monitor environment on
   */
  void startPublishingEnvironment(const std::string& topic = DEFAULT_PUBLISH_ENVIRONMENT_TOPIC);

  /**
   * @brief Start monitoring an environment for applying changes to this environment
   * @param topic The topic to monitor for environment changes
   */
  void startMonitoringEnvironment(const std::string& topic = DEFAULT_MONITOR_ENVIRONMENT_TOPIC);

  /**
   * @brief Start publishing the contact markers
   * @param topic The topic topic to publish the contact results markers on
   */
  void startPublishingMarkers(const std::string& topic = DEFAULT_PUBLISH_CONTACT_MARKER_TOPIC);

  /**
   * @brief Compute collision results and publish results.
   *
   * This also publishes environment and contact markers if correct flags are enabled for visualization and debuging.
   */
  void computeCollisionReportThread();

  void callbackJointState(boost::shared_ptr<sensor_msgs::JointState> msg);

  bool callbackModifyTesseractEnv(tesseract_msgs::ModifyEnvironment::Request& request,
                                  tesseract_msgs::ModifyEnvironment::Response& response);

  bool callbackComputeContactResultVector(tesseract_msgs::ComputeContactResultVector::Request& request,
                                          tesseract_msgs::ComputeContactResultVector::Response& response);

  void callbackTesseractEnvDiff(const tesseract_msgs::TesseractStatePtr& state);

private:
  tesseract::Tesseract::Ptr tess_;
  ros::NodeHandle& nh_;
  ros::NodeHandle& pnh_;
  std::vector<std::string> monitored_link_names_;
  tesseract_collision::ContactTestType type_;
  double contact_distance_;
  tesseract_collision::DiscreteContactManager::Ptr manager_;
  bool publish_environment_{ false };
  bool publish_contact_markers_{ false };
  ros::Subscriber joint_states_sub_;
  ros::Publisher contact_results_pub_;
  ros::Publisher environment_pub_;
  ros::Publisher contact_marker_pub_;
  ros::Subscriber environment_diff_sub_;
  ros::ServiceServer modify_env_service_;
  ros::ServiceServer compute_contact_results_;
  boost::mutex modify_mutex_;
  boost::shared_ptr<sensor_msgs::JointState> current_joint_states_;
  boost::condition_variable current_joint_states_evt_;
};

}  // end namespace contact_monitor

#endif  // CONTACT_MONITOR_H
