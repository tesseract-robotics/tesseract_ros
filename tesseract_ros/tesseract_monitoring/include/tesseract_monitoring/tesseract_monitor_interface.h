/**
 * @file tesseract_monitor_interface.h
 * @brief This is a utility class for applying changes to multiple tesseract monitors
 *
 * @author Levi Armstrong
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
#ifndef TESSERACT_MONITORING_TESSERACT_MONITOR_INTERFACE_H
#define TESSERACT_MONITORING_TESSERACT_MONITOR_INTERFACE_H

#include <tesseract_common/macros.h>
TESSERACT_COMMON_IGNORE_WARNINGS_PUSH
#include <ros/ros.h>
#include <ros/service.h>
#include <vector>
#include <tesseract_msgs/GetEnvironmentInformation.h>
TESSERACT_COMMON_IGNORE_WARNINGS_POP

#include <tesseract_environment/core/commands.h>
#include <tesseract_environment/core/environment.h>
#include <tesseract/tesseract.h>
#include <tesseract/manipulator_manager.h>
#include <tesseract_rosutils/utils.h>

namespace tesseract_monitoring
{
class TesseractMonitorInterface
{
public:
  /**
   * @brief The name of the service used by default for requesting tesseract environment information
   *
   * The full name is "/monitor_namespace/get_tesseract_information"
   */
  static const std::string DEFAULT_GET_ENVIRONMENT_INFORMATION_SERVICE;

  /**
   * @brief The name of the service used by default for setting the full tesseract environment state
   *
   * The full name is "/monitor_namespace/modify_tesseract"
   */
  static const std::string DEFAULT_MODIFY_ENVIRONMENT_SERVICE;

  TesseractMonitorInterface() = default;
  virtual ~TesseractMonitorInterface() = default;
  TesseractMonitorInterface(const TesseractMonitorInterface&) = default;
  TesseractMonitorInterface& operator=(const TesseractMonitorInterface&) = default;
  TesseractMonitorInterface(TesseractMonitorInterface&&) = default;
  TesseractMonitorInterface& operator=(TesseractMonitorInterface&&) = default;

  /**
   * @brief Add monitor namespace to interface
   * @param monitor_namespace
   */
  void addNamespace(std::string monitor_namespace);

  /**
   * @brief Remove monitor namespace from interface
   * @param monitor_namespace
   */
  void removeNamespace(const std::string& monitor_namespace);

  /**
   * @brief Apply provided command to all monitor namespaces
   * @param command The command to apply
   * @return A vector of failed namespace, if empty all namespace were updated successfully.
   */
  std::vector<std::string> applyCommand(const tesseract_environment::Command& command) const;
  std::vector<std::string> applyCommands(const tesseract_environment::Commands& commands) const;
  std::vector<std::string> applyCommands(const std::vector<tesseract_environment::Command>& commands) const;

  /**
   * @brief Apply provided command to only the provided namespace. The namespace does not have to be one that is
   * currently stored in this class.
   * @param command The command to apply
   * @return True if successful, otherwise false
   */
  bool applyCommand(const std::string& monitor_namespace, const tesseract_environment::Command& command) const;
  bool applyCommands(const std::string& monitor_namespace, const tesseract_environment::Commands& commands) const;
  bool applyCommands(const std::string& monitor_namespace,
                     const std::vector<tesseract_environment::Command>& commands) const;

  /**
   * @brief Pull information from the environment in the provided namespace and create a Environment Object
   * @param monitor_namespace The namespace to extract the environment from.
   * @return Environment Shared Pointer, if nullptr it failed
   */
  template <typename S>
  tesseract_environment::Environment::Ptr getEnvironment(const std::string& monitor_namespace) const
  {
    tesseract_msgs::GetEnvironmentInformation res;
    res.request.flags = tesseract_msgs::GetEnvironmentInformationRequest::COMMAND_HISTORY;

    bool status = ros::service::call(R"(/)" + monitor_namespace + DEFAULT_GET_ENVIRONMENT_INFORMATION_SERVICE, res);
    if (!status || !res.response.success)
    {
      ROS_ERROR_STREAM_NAMED(monitor_namespace, "Failed to get monitor environment information!");
      return nullptr;
    }

    tesseract_environment::Commands commands;
    try
    {
      commands = tesseract_rosutils::fromMsg(res.response.command_history);
    }
    catch (...)
    {
      ROS_ERROR_STREAM_NAMED(monitor_namespace, "Failed to convert command history message!");
      return nullptr;
    }

    auto env = std::make_shared<tesseract_environment::Environment>();
    env->init<S>(commands);

    return env;
  }

  /**
   * @brief Pull information from the environment in the provided namespace and create a Environment Object
   * @param monitor_namespace The namespace to extract the environment from.
   * @return Environment Shared Pointer, if nullptr it failed
   */
  template <typename S>
  tesseract::Tesseract::Ptr getTesseract(const std::string& monitor_namespace) const
  {
    tesseract_msgs::GetEnvironmentInformation res;
    res.request.flags = tesseract_msgs::GetEnvironmentInformationRequest::COMMAND_HISTORY |
                        tesseract_msgs::GetEnvironmentInformationRequest::KINEMATICS_INFORMATION;

    bool status = ros::service::call(R"(/)" + monitor_namespace + DEFAULT_GET_ENVIRONMENT_INFORMATION_SERVICE, res);
    if (!status || !res.response.success)
    {
      ROS_ERROR_STREAM_NAMED(monitor_namespace, "Failed to get monitor environment information!");
      return nullptr;
    }

    tesseract_environment::Commands commands;
    try
    {
      commands = tesseract_rosutils::fromMsg(res.response.command_history);
    }
    catch (...)
    {
      ROS_ERROR_STREAM_NAMED(monitor_namespace, "Failed to convert command history message!");
      return nullptr;
    }

    auto env = std::make_shared<tesseract_environment::Environment>();
    if (!env->init<S>(commands))
    {
      ROS_ERROR_STREAM_NAMED(monitor_namespace, "Failed to initialize environment!");
      return nullptr;
    }

    auto manip_manager = std::make_shared<tesseract::ManipulatorManager>();
    auto srdf = std::make_shared<tesseract_scene_graph::SRDFModel>();
    manip_manager->init(env, srdf);

    if (!tesseract_rosutils::fromMsg(*manip_manager, res.response.kinematics_information))
    {
      ROS_ERROR_STREAM_NAMED(monitor_namespace, "Failed to populate manipulator manager from kinematics information!");
      return nullptr;
    }

    auto thor = std::make_shared<tesseract::Tesseract>();
    if (!thor->init(*env, *manip_manager))
    {
      ROS_ERROR_STREAM_NAMED(monitor_namespace, "Failed to initialize tesseract!");
      return nullptr;
    }

    return thor;
  }

protected:
  ros::NodeHandle nh_;
  std::vector<std::string> ns_;

  bool sendCommands(const std::string& ns, const std::vector<tesseract_msgs::EnvironmentCommand>& commands) const;
};
}  // namespace tesseract_monitoring
#endif  // TESSERACT_MONITORING_TESSERACT_MONITOR_INTERFACE_H
