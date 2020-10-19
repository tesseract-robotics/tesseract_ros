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

#include <tesseract_common/macros.h>
TESSERACT_COMMON_IGNORE_WARNINGS_PUSH
#include <tesseract_msgs/ContactResultVector.h>
#include <visualization_msgs/MarkerArray.h>
TESSERACT_COMMON_IGNORE_WARNINGS_POP

#include <tesseract_rosutils/utils.h>
#include <tesseract_rosutils/plotting.h>

namespace tesseract_monitoring
{
const std::string ContactMonitor::DEFAULT_JOINT_STATES_TOPIC = "joint_states";
const std::string ContactMonitor::DEFAULT_PUBLISH_CONTACT_RESULTS_TOPIC = R"(/contact_results)";
const std::string ContactMonitor::DEFAULT_PUBLISH_CONTACT_MARKER_TOPIC = R"(/contact_results_markers)";
const std::string ContactMonitor::DEFAULT_COMPUTE_CONTACT_RESULTS_SERVICE = R"(/compute_contact_results)";

ContactMonitor::ContactMonitor(std::string monitor_namespace,
                               const tesseract::Tesseract::Ptr& tesseract,
                               ros::NodeHandle& nh,
                               ros::NodeHandle& pnh,
                               const std::vector<std::string>& monitored_link_names,
                               const tesseract_collision::ContactTestType& type,
                               double contact_distance,
                               const std::string& joint_state_topic)
  : monitor_namespace_(std::move(monitor_namespace))
  , tesseract_(tesseract)
  , nh_(nh)
  , pnh_(pnh)
  , monitored_link_names_(monitored_link_names)
  , type_(type)
  , contact_distance_(contact_distance)
{
  if (tesseract_ == nullptr)
  {
    ROS_ERROR("Null pointer passed for tesseract object.  Not setting up contact monitor.");
    return;
  }

  // Create Environment Monitor
  monitor_ = std::make_shared<tesseract_monitoring::EnvironmentMonitor>(tesseract_, monitor_namespace_);

  manager_ = tesseract->getEnvironment()->getDiscreteContactManager();
  if (manager_ == nullptr)
  {
    ROS_ERROR("Discrete contact manager is a null pointer.  Not setting up contact monitor.");
    return;
  }
  manager_->setActiveCollisionObjects(monitored_link_names);
  manager_->setContactDistanceThreshold(contact_distance);

  joint_states_sub_ = nh_.subscribe(joint_state_topic, 1, &ContactMonitor::callbackJointState, this);
  std::string contact_results_topic = R"(/)" + monitor_namespace_ + DEFAULT_PUBLISH_CONTACT_RESULTS_TOPIC;
  std::string compute_contact_results = R"(/)" + monitor_namespace_ + DEFAULT_COMPUTE_CONTACT_RESULTS_SERVICE;

  contact_results_pub_ = pnh_.advertise<tesseract_msgs::ContactResultVector>(contact_results_topic, 1, true);
  compute_contact_results_ =
      pnh_.advertiseService(compute_contact_results, &ContactMonitor::callbackComputeContactResultVector, this);
}

ContactMonitor::~ContactMonitor() { current_joint_states_evt_.notify_all(); }

void ContactMonitor::startPublishingEnvironment()
{
  monitor_->startPublishingEnvironment(tesseract_monitoring::EnvironmentMonitor::UPDATE_ENVIRONMENT);
}

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
    tesseract_msgs::ContactResultVector contacts_msg;
    // Limit the lock
    {
      std::unique_lock lock(modify_mutex_);
      if (env_revision_ != tesseract_->getEnvironment()->getRevision())
      {
        env_revision_ = tesseract_->getEnvironment()->getRevision();

        // Create a new manager
        std::vector<std::string> active = manager_->getActiveCollisionObjects();
        double contact_distance = manager_->getContactDistanceThreshold();
        tesseract_collision::IsContactAllowedFn fn = manager_->getIsContactAllowedFn();

        manager_ = tesseract_->getEnvironment()->getDiscreteContactManager();
        manager_->setActiveCollisionObjects(active);
        manager_->setContactDistanceThreshold(contact_distance);
        manager_->setIsContactAllowedFn(fn);
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
      contacts_msg.contacts.clear();

      tesseract_->getEnvironment()->setState(msg->name, msg->position);
      tesseract_environment::EnvState::ConstPtr state = tesseract_->getEnvironment()->getCurrentState();

      manager_->setCollisionObjectsTransform(state->link_transforms);
      manager_->contactTest(contacts, type_);
    }

    tesseract_collision::ContactResultVector contacts_vector;
    tesseract_collision::flattenResults(std::move(contacts), contacts_vector);
    Eigen::VectorXd safety_distance(contacts_vector.size());
    contacts_msg.contacts.reserve(contacts_vector.size());
    for (std::size_t i = 0; i < contacts_vector.size(); ++i)
    {
      safety_distance[static_cast<long>(i)] = contact_distance_;
      tesseract_msgs::ContactResult contact_msg;
      tesseract_rosutils::toMsg(contact_msg, contacts_vector[i], msg->header.stamp);
      contacts_msg.contacts.push_back(contact_msg);
    }
    contact_results_pub_.publish(contacts_msg);

    if (publish_contact_markers_)
    {
      int id_counter = 0;
      visualization_msgs::MarkerArray marker_msg = tesseract_rosutils::ROSPlotting::getContactResultsMarkerArrayMsg(
          id_counter,
          tesseract_->getEnvironment()->getSceneGraph()->getRoot(),
          "contact_monitor",
          msg->header.stamp,
          monitored_link_names_,
          contacts_vector,
          safety_distance);
      contact_marker_pub_.publish(marker_msg);
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

  auto env = tesseract_->getEnvironment();
  int revision = static_cast<int>(request.revision);
  if (request.append)
    revision = env->getRevision();

  if (!tesseract_->getEnvironment() || request.id != tesseract_->getEnvironment()->getName() ||
      revision != tesseract_->getEnvironment()->getRevision())
    return false;

  response.success = tesseract_rosutils::processMsg(*(tesseract_->getEnvironment()), request.commands);
  response.revision = static_cast<unsigned long>(env->getRevision());

  // Create a new manager
  std::vector<std::string> active = manager_->getActiveCollisionObjects();
  double contact_distance = manager_->getContactDistanceThreshold();
  tesseract_collision::IsContactAllowedFn fn = manager_->getIsContactAllowedFn();

  manager_ = tesseract_->getEnvironment()->getDiscreteContactManager();
  manager_->setActiveCollisionObjects(active);
  manager_->setContactDistanceThreshold(contact_distance);
  manager_->setIsContactAllowedFn(fn);

  return true;
}

bool ContactMonitor::callbackComputeContactResultVector(tesseract_msgs::ComputeContactResultVectorRequest& request,
                                                        tesseract_msgs::ComputeContactResultVectorResponse& response)
{
  tesseract_collision::ContactResultMap contact_results;

  // Limit the lock
  {
    std::scoped_lock lock(modify_mutex_);

    tesseract_->getEnvironment()->setState(request.joint_states.name, request.joint_states.position);
    tesseract_environment::EnvState::ConstPtr state = tesseract_->getEnvironment()->getCurrentState();

    manager_->setCollisionObjectsTransform(state->link_transforms);
    manager_->contactTest(contact_results, type_);
  }

  tesseract_collision::ContactResultVector contacts_vector;
  tesseract_collision::flattenResults(std::move(contact_results), contacts_vector);
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
