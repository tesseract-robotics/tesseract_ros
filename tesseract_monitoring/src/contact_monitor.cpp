/**
 * @file contact_monitor.cpp
 * @brief implementation of the contact_monitor library.  It publishes
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

#include <tesseract_monitoring/contact_monitor.h>
#include <tesseract_monitoring/environment_monitor.h>

#include <tesseract_common/macros.h>
TESSERACT_COMMON_IGNORE_WARNINGS_PUSH
#include <ros/node_handle.h>
#include <sensor_msgs/JointState.h>
#include <tesseract_msgs/ComputeContactResultVector.h>
#include <tesseract_msgs/ModifyEnvironment.h>
#include <tesseract_msgs/EnvironmentState.h>
#include <visualization_msgs/MarkerArray.h>
#include <mutex>
#include <condition_variable>
TESSERACT_COMMON_IGNORE_WARNINGS_POP

#include <tesseract_rosutils/utils.h>
#include <tesseract_rosutils/plotting.h>
#include <tesseract_monitoring/constants.h>

#include <tesseract_common/contact_allowed_validator.h>
#include <tesseract_collision/core/discrete_contact_manager.h>
#include <tesseract_scene_graph/scene_state.h>
#include <tesseract_environment/environment.h>
#include <tesseract_environment/environment_monitor.h>
#include <tesseract_visualization/markers/contact_results_marker.h>

namespace tesseract_monitoring
{
struct ContactMonitor::Implementation
{
  std::string monitor_namespace;
  std::string monitored_namespace;
  int env_revision{ 0 };
  tesseract_environment::EnvironmentMonitor::UPtr monitor;
  ros::NodeHandle& nh;
  ros::NodeHandle& pnh;
  std::vector<std::string> monitored_link_names;
  std::vector<std::string> disabled_link_names;
  tesseract_collision::ContactTestType type;
  double contact_distance;
  tesseract_collision::DiscreteContactManager::UPtr manager;
  bool publish_contact_markers{ false };
  ros::Subscriber joint_states_sub;
  ros::Publisher contact_results_pub;
  ros::Publisher contact_marker_pub;
  ros::ServiceServer compute_contact_results;
  std::mutex modify_mutex;
  boost::shared_ptr<sensor_msgs::JointState> current_joint_states;
  std::condition_variable current_joint_states_evt;

  Implementation(std::string monitor_namespace_,
                 tesseract_environment::Environment::UPtr env_,
                 ros::NodeHandle& nh_,
                 ros::NodeHandle& pnh_,
                 std::vector<std::string> monitored_link_names_,
                 std::vector<std::string> disabled_link_names_,
                 tesseract_collision::ContactTestType type_,
                 double contact_distance_,
                 const std::string& joint_state_topic)
    : monitor_namespace(std::move(monitor_namespace_))
    , nh(nh_)
    , pnh(pnh_)
    , monitored_link_names(std::move(monitored_link_names_))
    , disabled_link_names(std::move(disabled_link_names_))
    , type(type_)
    , contact_distance(contact_distance_)
  {
    if (env_ == nullptr)
      throw std::runtime_error("Null pointer passed for environment object to contact monitor.");

    // Create Environment Monitor
    monitor = std::make_unique<tesseract_monitoring::ROSEnvironmentMonitor>(std::move(env_), monitor_namespace);
    manager = monitor->environment().getDiscreteContactManager();

    if (manager == nullptr)
      throw std::runtime_error("Contact monitor failed to get discrete contact manager from environment!");

    manager->setActiveCollisionObjects(monitored_link_names);
    manager->setDefaultCollisionMarginData(contact_distance);
    for (const auto& disabled_link_name : disabled_link_names)
      manager->disableCollisionObject(disabled_link_name);

    std::cout << ((disabled_link_names.empty()) ? "Empty" : "Not Empty") << std::endl;

    joint_states_sub = nh.subscribe(joint_state_topic, 1, &ContactMonitor::Implementation::callbackJointState, this);
    std::string contact_results_topic = R"(/)" + monitor_namespace + DEFAULT_PUBLISH_CONTACT_RESULTS_TOPIC;
    std::string compute_contact_results_topic = R"(/)" + monitor_namespace + DEFAULT_COMPUTE_CONTACT_RESULTS_SERVICE;

    contact_results_pub = pnh.advertise<tesseract_msgs::ContactResultVector>(contact_results_topic, 1, true);
    compute_contact_results = pnh.advertiseService(
        compute_contact_results_topic, &ContactMonitor::Implementation::callbackComputeContactResultVector, this);
  }

  /**
   * @brief Compute collision results and publish results.
   *
   * This also publishes environment and contact markers if correct flags are enabled for visualization and debuging.
   */
  void computeCollisionReportThread()
  {
    while (!ros::isShuttingDown())
    {
      boost::shared_ptr<sensor_msgs::JointState> msg = nullptr;
      tesseract_collision::ContactResultMap contacts;
      tesseract_collision::ContactResultVector contacts_vector;
      tesseract_msgs::ContactResultVector contacts_msg;
      std::string root_link;
      // Limit the lock
      {
        std::unique_lock lock(modify_mutex);
        root_link = monitor->environment().getRootLinkName();
        if (env_revision != monitor->environment().getRevision())
        {
          // Create a new manager
          std::vector<std::string> active;
          tesseract_collision::CollisionMarginData contact_margin_data;
          tesseract_common::ContactAllowedValidator::ConstPtr fn;

          {
            auto lock_read = monitor->environment().lockRead();

            env_revision = monitor->environment().getRevision();
            active = manager->getActiveCollisionObjects();
            contact_margin_data = manager->getCollisionMarginData();
            fn = manager->getContactAllowedValidator();
            manager = monitor->environment().getDiscreteContactManager();
          }

          manager->setActiveCollisionObjects(active);
          manager->setCollisionMarginData(contact_margin_data);
          manager->setContactAllowedValidator(fn);
          for (const auto& disabled_link_name : disabled_link_names)
            manager->disableCollisionObject(disabled_link_name);
        }

        if (!current_joint_states)
        {
          current_joint_states_evt.wait(lock);
        }

        if (!current_joint_states)
          continue;

        msg = current_joint_states;
        current_joint_states.reset();

        contacts.clear();
        contacts_vector.clear();
        contacts_msg.contacts.clear();

        monitor->environment().setState(msg->name,
                                        Eigen::Map<Eigen::VectorXd>(msg->position.data(), msg->position.size()));
        tesseract_scene_graph::SceneState state = monitor->environment().getState();

        manager->setCollisionObjectsTransform(state.link_transforms);
        manager->contactTest(contacts, type);
      }

      if (!contacts.empty())
      {
        contacts.flattenCopyResults(contacts_vector);
        contacts_msg.contacts.reserve(contacts_vector.size());
        for (std::size_t i = 0; i < contacts_vector.size(); ++i)
        {
          tesseract_msgs::ContactResult contact_msg;
          tesseract_rosutils::toMsg(contact_msg, contacts_vector[i], msg->header.stamp);
          contacts_msg.contacts.push_back(contact_msg);
        }

        contact_results_pub.publish(contacts_msg);

        if (publish_contact_markers)
        {
          int id_counter = 0;
          tesseract_visualization::ContactResultsMarker marker(
              monitored_link_names, contacts_vector, manager->getCollisionMarginData());
          visualization_msgs::MarkerArray marker_msg = tesseract_rosutils::getContactResultsMarkerArrayMsg(
              id_counter, root_link, "contact_monitor", msg->header.stamp, marker);
          contact_marker_pub.publish(marker_msg);
        }
      }
    }
  }

  void callbackJointState(boost::shared_ptr<sensor_msgs::JointState> msg)
  {
    std::scoped_lock lock(modify_mutex);
    current_joint_states = std::move(msg);
    current_joint_states_evt.notify_all();
  }

  bool callbackModifyTesseractEnv(tesseract_msgs::ModifyEnvironmentRequest& request,
                                  tesseract_msgs::ModifyEnvironmentResponse& response)
  {
    std::scoped_lock lock(modify_mutex);

    int revision = static_cast<int>(request.revision);
    {
      auto lock_read = monitor->environment().lockRead();
      if (request.append)
        revision = monitor->environment().getRevision();

      if (!monitor->environment().isInitialized() || request.id != monitor->environment().getName() ||
          revision != monitor->environment().getRevision())
        return false;
    }

    response.success = tesseract_rosutils::processMsg(monitor->environment(), request.commands);
    response.revision = static_cast<unsigned long>(monitor->environment().getRevision());

    // Create a new manager
    std::vector<std::string> active;
    tesseract_collision::CollisionMarginData contact_margin_data;
    tesseract_common::ContactAllowedValidator::ConstPtr fn;

    {
      auto lock_read = monitor->environment().lockRead();
      active = manager->getActiveCollisionObjects();
      contact_margin_data = manager->getCollisionMarginData();
      fn = manager->getContactAllowedValidator();
      manager = monitor->environment().getDiscreteContactManager();
    }

    manager->setActiveCollisionObjects(active);
    manager->setCollisionMarginData(contact_margin_data);
    manager->setContactAllowedValidator(fn);
    for (const auto& disabled_link_name : disabled_link_names)
      manager->disableCollisionObject(disabled_link_name);

    return true;
  }

  bool callbackComputeContactResultVector(tesseract_msgs::ComputeContactResultVectorRequest& request,
                                          tesseract_msgs::ComputeContactResultVectorResponse& response)
  {
    thread_local tesseract_collision::ContactResultMap contact_results;
    thread_local tesseract_collision::ContactResultVector contacts_vector;
    contact_results.clear();
    contacts_vector.clear();

    monitor->environment().setState(
        request.joint_states.name,
        Eigen::Map<Eigen::VectorXd>(request.joint_states.position.data(), request.joint_states.position.size()));
    tesseract_scene_graph::SceneState state = monitor->environment().getState();

    // Limit the lock
    {
      std::scoped_lock lock(modify_mutex);
      manager->setCollisionObjectsTransform(state.link_transforms);
      manager->contactTest(contact_results, type);
    }

    contact_results.flattenCopyResults(contacts_vector);
    response.collision_result.contacts.reserve(contacts_vector.size());
    for (const auto& contact : contacts_vector)
    {
      tesseract_msgs::ContactResult contact_msg;
      tesseract_rosutils::toMsg(contact_msg, contact, request.joint_states.header.stamp);
      response.collision_result.contacts.push_back(contact_msg);
    }
    response.success = true;

    return true;
  }
};

ContactMonitor::ContactMonitor(std::string monitor_namespace,
                               std::unique_ptr<tesseract_environment::Environment> env,
                               ros::NodeHandle& nh,
                               ros::NodeHandle& pnh,
                               std::vector<std::string> monitored_link_names,
                               std::vector<std::string> disabled_link_names,
                               tesseract_collision::ContactTestType type,
                               double contact_distance,
                               const std::string& joint_state_topic)
  : impl_(std::make_unique<Implementation>(std::move(monitor_namespace),
                                           std::move(env),
                                           nh,
                                           pnh,
                                           std::move(monitored_link_names),
                                           std::move(disabled_link_names),
                                           type,
                                           contact_distance,
                                           joint_state_topic))
{
}

ContactMonitor::~ContactMonitor() { impl_->current_joint_states_evt.notify_all(); }

void ContactMonitor::startPublishingEnvironment() { impl_->monitor->startPublishingEnvironment(); }

void ContactMonitor::startMonitoringEnvironment(const std::string& monitored_namepsace)
{
  impl_->monitor->startMonitoringEnvironment(monitored_namepsace);
}

void ContactMonitor::startPublishingMarkers()
{
  impl_->publish_contact_markers = true;
  std::string contact_marker_topic = R"(/)" + impl_->monitor_namespace + DEFAULT_PUBLISH_CONTACT_MARKER_TOPIC;
  impl_->contact_marker_pub = impl_->pnh.advertise<visualization_msgs::MarkerArray>(contact_marker_topic, 1, true);
}

void ContactMonitor::computeCollisionReportThread() { impl_->computeCollisionReportThread(); }
}  // namespace tesseract_monitoring
