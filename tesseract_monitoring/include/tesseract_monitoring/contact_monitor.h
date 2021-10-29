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

#ifndef TESSERACT_MONITORING_CONTACT_MONITOR_H
#define TESSERACT_MONITORING_CONTACT_MONITOR_H

#include <tesseract_common/macros.h>
TESSERACT_COMMON_IGNORE_WARNINGS_PUSH
#include <ros/ros.h>
#include <sensor_msgs/JointState.h>
#include <tesseract_msgs/ComputeContactResultVector.h>
#include <tesseract_msgs/ModifyEnvironment.h>
#include <tesseract_msgs/EnvironmentState.h>
#include <mutex>
#include <condition_variable>
TESSERACT_COMMON_IGNORE_WARNINGS_POP

#include <tesseract_collision/core/discrete_contact_manager.h>
#include <tesseract_environment/environment.h>
#include <tesseract_monitoring/environment_monitor.h>

namespace tesseract_monitoring
{
class ContactMonitor
{
public:
  ContactMonitor(std::string monitor_namespace,
                 tesseract_environment::Environment::UPtr env,
                 ros::NodeHandle& nh,
                 ros::NodeHandle& pnh,
                 const std::vector<std::string>& monitored_link_names,
                 const tesseract_collision::ContactTestType& type,
                 double contact_distance = 0.1,
                 const std::string& joint_state_topic = DEFAULT_JOINT_STATES_TOPIC);
  ~ContactMonitor();
  /**
   * @brief Custom copy constructor, copy assignment, move constructor, and move
   * assignment.  Because the condition variable current_joint_states_evt_
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
  void startPublishingEnvironment();

  /**
   * @brief Start monitoring an environment for applying changes to this environment
   * @param topic The topic to monitor for environment changes
   */
  void startMonitoringEnvironment(const std::string& monitored_namepsace);

  /**
   * @brief Start publishing the contact markers
   * @param topic The topic topic to publish the contact results markers on
   */
  void startPublishingMarkers();

  /**
   * @brief Compute collision results and publish results.
   *
   * This also publishes environment and contact markers if correct flags are enabled for visualization and debugging.
   */
  void computeCollisionReportThread();

  void callbackJointState(boost::shared_ptr<sensor_msgs::JointState> msg);

  bool callbackModifyTesseractEnv(tesseract_msgs::ModifyEnvironment::Request& request,
                                  tesseract_msgs::ModifyEnvironment::Response& response);

  bool callbackComputeContactResultVector(tesseract_msgs::ComputeContactResultVector::Request& request,
                                          tesseract_msgs::ComputeContactResultVector::Response& response);

  void callbackTesseractEnvDiff(const tesseract_msgs::EnvironmentStatePtr& state);

private:
  std::string monitor_namespace_;
  std::string monitored_namespace_;
  int env_revision_{ 0 };
  tesseract_monitoring::EnvironmentMonitor::UPtr monitor_;
  ros::NodeHandle& nh_;
  ros::NodeHandle& pnh_;
  std::vector<std::string> monitored_link_names_;
  tesseract_collision::ContactTestType type_;
  double contact_distance_;
  tesseract_collision::DiscreteContactManager::UPtr manager_;
  bool publish_contact_markers_{ false };
  ros::Subscriber joint_states_sub_;
  ros::Publisher contact_results_pub_;
  ros::Publisher contact_marker_pub_;
  ros::ServiceServer compute_contact_results_;
  std::mutex modify_mutex_;
  boost::shared_ptr<sensor_msgs::JointState> current_joint_states_;
  std::condition_variable current_joint_states_evt_;
};

}  // namespace tesseract_monitoring

#endif  // TESSERACT_MONITORING_
