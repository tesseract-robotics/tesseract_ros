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
#include <tesseract_msgs/ContactResultVector.h>
#include <visualization_msgs/MarkerArray.h>
TESSERACT_COMMON_IGNORE_WARNINGS_POP

#include <tesseract_rosutils/utils.h>
#include <tesseract_rosutils/plotting.h>
#include <tesseract_monitoring/constants.h>

namespace tesseract_monitoring
{
ContactMonitor::ContactMonitor(std::string monitor_namespace,
                               tesseract_environment::Environment::UPtr env,
                               ros::NodeHandle& nh,
                               ros::NodeHandle& pnh,
                               std::vector<std::string> monitored_link_names,
                               std::vector<std::string> disabled_link_names,
                               tesseract_collision::ContactTestType type,
                               double contact_distance,
                               const std::string& joint_state_topic)
  : monitor_namespace_(std::move(monitor_namespace))
  , nh_(nh)
  , pnh_(pnh)
  , monitored_link_names_(std::move(monitored_link_names))
  , disabled_link_names_(std::move(disabled_link_names))
  , type_(type)
  , contact_distance_(contact_distance)
{
  if (env == nullptr)
    throw std::runtime_error("Null pointer passed for environment object to contact monitor.");

  // Create Environment Monitor
  monitor_ = std::make_unique<tesseract_monitoring::ROSEnvironmentMonitor>(std::move(env), monitor_namespace_);
  manager_ = monitor_->environment().getDiscreteContactManager();

  if (manager_ == nullptr)
    throw std::runtime_error("Contact monitor failed to get discrete contact manager from environment!");

  manager_->setActiveCollisionObjects(monitored_link_names_);
  manager_->setDefaultCollisionMarginData(contact_distance_);
  for (const auto& disabled_link_name : disabled_link_names_)
    manager_->disableCollisionObject(disabled_link_name);

  std::cout << ((disabled_link_names_.empty()) ? "Empty" : "Not Empty") << std::endl;

  joint_states_sub_ = nh_.subscribe(joint_state_topic, 1, &ContactMonitor::callbackJointState, this);
  std::string contact_results_topic = R"(/)" + monitor_namespace_ + DEFAULT_PUBLISH_CONTACT_RESULTS_TOPIC;
  std::string compute_contact_results = R"(/)" + monitor_namespace_ + DEFAULT_COMPUTE_CONTACT_RESULTS_SERVICE;

  contact_results_pub_ = pnh_.advertise<tesseract_msgs::ContactResultVector>(contact_results_topic, 1, true);
  compute_contact_results_ =
      pnh_.advertiseService(compute_contact_results, &ContactMonitor::callbackComputeContactResultVector, this);
}

ContactMonitor::~ContactMonitor() { current_joint_states_evt_.notify_all(); }

void ContactMonitor::startPublishingEnvironment() { monitor_->startPublishingEnvironment(); }

void ContactMonitor::startMonitoringEnvironment(const std::string& monitored_namepsace)
{
  monitor_->startMonitoringEnvironment(monitored_namepsace);
}

void ContactMonitor::startPublishingMarkers()
{
  publish_contact_markers_ = true;
  std::string contact_marker_topic = R"(/)" + monitor_namespace_ + DEFAULT_PUBLISH_CONTACT_MARKER_TOPIC;
  contact_marker_pub_ = pnh_.advertise<visualization_msgs::MarkerArray>(contact_marker_topic, 1, true);
}

/**
 * @brief Compute collision results and publish results.
 *
 * This also publishes environment and contact markers if correct flags are enabled for visualization and debuging.
 */
void ContactMonitor::computeCollisionReportThread()
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
      std::unique_lock lock(modify_mutex_);
      root_link = monitor_->environment().getRootLinkName();
      if (env_revision_ != monitor_->environment().getRevision())
      {
        // Create a new manager
        std::vector<std::string> active;
        tesseract_collision::CollisionMarginData contact_margin_data;
        tesseract_collision::IsContactAllowedFn fn;

        {
          auto lock_read = monitor_->environment().lockRead();

          env_revision_ = monitor_->environment().getRevision();
          active = manager_->getActiveCollisionObjects();
          contact_margin_data = manager_->getCollisionMarginData();
          fn = manager_->getIsContactAllowedFn();
          manager_ = monitor_->environment().getDiscreteContactManager();
        }

        manager_->setActiveCollisionObjects(active);
        manager_->setCollisionMarginData(contact_margin_data);
        manager_->setIsContactAllowedFn(fn);
        for (const auto& disabled_link_name : disabled_link_names_)
          manager_->disableCollisionObject(disabled_link_name);
      }

      if (!current_joint_states_)
      {
        current_joint_states_evt_.wait(lock);
      }

      if (!current_joint_states_)
        continue;

      msg = current_joint_states_;
      current_joint_states_.reset();

      contacts.clear();
      contacts_vector.clear();
      contacts_msg.contacts.clear();

      monitor_->environment().setState(msg->name,
                                       Eigen::Map<Eigen::VectorXd>(msg->position.data(), msg->position.size()));
      tesseract_scene_graph::SceneState state = monitor_->environment().getState();

      manager_->setCollisionObjectsTransform(state.link_transforms);
      manager_->contactTest(contacts, type_);
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

      contact_results_pub_.publish(contacts_msg);

      if (publish_contact_markers_)
      {
        int id_counter = 0;
        tesseract_visualization::ContactResultsMarker marker(
            monitored_link_names_, contacts_vector, manager_->getCollisionMarginData());
        visualization_msgs::MarkerArray marker_msg = tesseract_rosutils::ROSPlotting::getContactResultsMarkerArrayMsg(
            id_counter, root_link, "contact_monitor", msg->header.stamp, marker);
        contact_marker_pub_.publish(marker_msg);
      }
    }
  }
}

void ContactMonitor::callbackJointState(boost::shared_ptr<sensor_msgs::JointState> msg)
{
  std::scoped_lock lock(modify_mutex_);
  current_joint_states_ = std::move(msg);
  current_joint_states_evt_.notify_all();
}

bool ContactMonitor::callbackModifyTesseractEnv(tesseract_msgs::ModifyEnvironmentRequest& request,
                                                tesseract_msgs::ModifyEnvironmentResponse& response)
{
  std::scoped_lock lock(modify_mutex_);

  int revision = static_cast<int>(request.revision);
  {
    auto lock_read = monitor_->environment().lockRead();
    if (request.append)
      revision = monitor_->environment().getRevision();

    if (!monitor_->environment().isInitialized() || request.id != monitor_->environment().getName() ||
        revision != monitor_->environment().getRevision())
      return false;
  }

  response.success = tesseract_rosutils::processMsg(monitor_->environment(), request.commands);
  response.revision = static_cast<unsigned long>(monitor_->environment().getRevision());

  // Create a new manager
  std::vector<std::string> active;
  tesseract_collision::CollisionMarginData contact_margin_data;
  tesseract_collision::IsContactAllowedFn fn;

  {
    auto lock_read = monitor_->environment().lockRead();
    active = manager_->getActiveCollisionObjects();
    contact_margin_data = manager_->getCollisionMarginData();
    fn = manager_->getIsContactAllowedFn();
    manager_ = monitor_->environment().getDiscreteContactManager();
  }

  manager_->setActiveCollisionObjects(active);
  manager_->setCollisionMarginData(contact_margin_data);
  manager_->setIsContactAllowedFn(fn);
  for (const auto& disabled_link_name : disabled_link_names_)
    manager_->disableCollisionObject(disabled_link_name);

  return true;
}

bool ContactMonitor::callbackComputeContactResultVector(tesseract_msgs::ComputeContactResultVectorRequest& request,
                                                        tesseract_msgs::ComputeContactResultVectorResponse& response)
{
  thread_local tesseract_collision::ContactResultMap contact_results;
  thread_local tesseract_collision::ContactResultVector contacts_vector;
  contact_results.clear();
  contacts_vector.clear();

  monitor_->environment().setState(
      request.joint_states.name,
      Eigen::Map<Eigen::VectorXd>(request.joint_states.position.data(), request.joint_states.position.size()));
  tesseract_scene_graph::SceneState state = monitor_->environment().getState();

  // Limit the lock
  {
    std::scoped_lock lock(modify_mutex_);
    manager_->setCollisionObjectsTransform(state.link_transforms);
    manager_->contactTest(contact_results, type_);
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

}  // namespace tesseract_monitoring
