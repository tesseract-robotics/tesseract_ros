/**
 * @file tesseract_monitor_interface.cpp
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

#include <tesseract_common/macros.h>
TESSERACT_COMMON_IGNORE_WARNINGS_PUSH
#include <ros/service.h>
#include <ros/console.h>
#include <tesseract_msgs/ModifyEnvironment.h>
#include <tesseract_msgs/EnvironmentCommand.h>
#include <tesseract_msgs/GetEnvironmentInformation.h>
TESSERACT_COMMON_IGNORE_WARNINGS_POP

#include <tesseract_monitoring/constants.h>
#include <tesseract_monitoring/environment_monitor_interface.h>
#include <tesseract_rosutils/utils.h>

#include <tesseract_scene_graph/scene_state.h>
#include <tesseract_environment/environment.h>
#include <tesseract_environment/command.h>

namespace tesseract_monitoring
{
bool sendCommands(const std::string& env_name,
                  const std::string& ns,
                  const std::vector<tesseract_msgs::EnvironmentCommand>& commands)
{
  tesseract_msgs::ModifyEnvironment res;
  res.request.id = env_name;
  res.request.append = true;
  res.request.commands = commands;

  bool status = ros::service::call(R"(/)" + ns + DEFAULT_MODIFY_ENVIRONMENT_SERVICE, res);
  if (!status || !res.response.success)
  {
    ROS_ERROR_STREAM_NAMED(ns, "sendCommands: Failed to update monitored environment!");
    return false;
  }

  return true;
}

ROSEnvironmentMonitorInterface::ROSEnvironmentMonitorInterface(std::string env_name)
  : EnvironmentMonitorInterface(std::move(env_name))
{
}

bool ROSEnvironmentMonitorInterface::wait(std::chrono::duration<double> duration) const
{
  if (ns_.empty())
  {
    ROS_ERROR("TesseractMonitorInterface namespaces are empty and cannot wait!");
    return false;
  }

  const ros::WallTime start_time = ros::WallTime::now();
  ros::WallDuration wall_timeout;
  if (std::chrono::duration_cast<std::chrono::seconds>(duration).count() == 0)
    wall_timeout = ros::WallDuration(-1);
  else
    wall_timeout = ros::WallDuration(duration.count());

  for (const auto& ns : ns_)
  {
    bool results = waitForNamespace(ns, duration);
    if (!results)
      return false;

    if (wall_timeout >= ros::WallDuration(0))
    {
      const ros::WallTime current_time = ros::WallTime::now();

      if ((current_time - start_time) >= wall_timeout)
        return false;
    }
  }
  return true;
}

bool ROSEnvironmentMonitorInterface::waitForNamespace(const std::string& monitor_namespace,
                                                      std::chrono::duration<double> duration) const
{
  std::string service_name = R"(/)" + monitor_namespace + DEFAULT_GET_ENVIRONMENT_INFORMATION_SERVICE;
  const ros::WallTime start_time = ros::WallTime::now();
  ros::WallDuration wall_timeout;
  if (std::chrono::duration_cast<std::chrono::seconds>(duration).count() == 0)
    wall_timeout = ros::WallDuration(-1);
  else
    wall_timeout = ros::WallDuration(duration.count());

  while (ros::ok())
  {
    bool results = ros::service::exists(service_name, false);
    if (results)
    {
      tesseract_msgs::GetEnvironmentInformation res;
      res.request.flags = tesseract_msgs::GetEnvironmentInformationRequest::COMMAND_HISTORY;
      bool status = ros::service::call(R"(/)" + monitor_namespace + DEFAULT_GET_ENVIRONMENT_INFORMATION_SERVICE, res);
      if (status && res.response.success)
        return true;
    }

    if (wall_timeout >= ros::WallDuration(0))
    {
      const ros::WallTime current_time = ros::WallTime::now();
      if ((current_time - start_time) >= wall_timeout)
        return false;
    }

    ros::WallDuration(0.02).sleep();
  }

  return false;
}

void ROSEnvironmentMonitorInterface::addNamespace(std::string monitor_namespace)
{
  if (std::find(ns_.begin(), ns_.end(), monitor_namespace) == ns_.end())
    ns_.push_back(monitor_namespace);
}

void ROSEnvironmentMonitorInterface::removeNamespace(const std::string& monitor_namespace)
{
  auto it = std::remove_if(
      ns_.begin(), ns_.end(), [monitor_namespace](const std::string& ns) { return (ns == monitor_namespace); });
  ns_.erase(it, ns_.end());
}

std::vector<std::string>
ROSEnvironmentMonitorInterface::applyCommand(const tesseract_environment::Command& command) const
{
  std::vector<std::string> failed_namespace;
  failed_namespace.reserve(ns_.size());
  for (const auto& ns : ns_)
    if (!applyCommand(ns, command))
      failed_namespace.push_back(ns);

  return failed_namespace;
}

std::vector<std::string> ROSEnvironmentMonitorInterface::applyCommands(
    const std::vector<std::shared_ptr<const tesseract_environment::Command>>& commands) const
{
  std::vector<std::string> failed_namespace;
  failed_namespace.reserve(ns_.size());
  for (const auto& ns : ns_)
    if (!applyCommands(ns, commands))
      failed_namespace.push_back(ns);

  return failed_namespace;
}

std::vector<std::string>
ROSEnvironmentMonitorInterface::applyCommands(const std::vector<tesseract_environment::Command>& commands) const
{
  std::vector<std::string> failed_namespace;
  failed_namespace.reserve(ns_.size());
  for (const auto& ns : ns_)
    if (!applyCommands(ns, commands))
      failed_namespace.push_back(ns);

  return failed_namespace;
}

bool ROSEnvironmentMonitorInterface::applyCommand(const std::string& monitor_namespace,
                                                  const tesseract_environment::Command& command) const
{
  tesseract_msgs::EnvironmentCommand command_msg;
  if (tesseract_rosutils::toMsg(command_msg, command))
    return sendCommands(env_name_, monitor_namespace, { command_msg });

  ROS_ERROR_STREAM_NAMED(monitor_namespace,
                         "Failed to convert latest changes to message and update monitored environment!");
  return false;
}

bool ROSEnvironmentMonitorInterface::applyCommands(
    const std::string& monitor_namespace,
    const std::vector<std::shared_ptr<const tesseract_environment::Command>>& commands) const
{
  std::vector<tesseract_msgs::EnvironmentCommand> commands_msg;
  if (tesseract_rosutils::toMsg(commands_msg, commands, 0))
    return sendCommands(env_name_, monitor_namespace, commands_msg);

  ROS_ERROR_STREAM_NAMED(monitor_namespace,
                         "Failed to convert latest changes to message and update monitored environment!");
  return false;
}

bool ROSEnvironmentMonitorInterface::applyCommands(const std::string& monitor_namespace,
                                                   const std::vector<tesseract_environment::Command>& commands) const
{
  std::vector<tesseract_msgs::EnvironmentCommand> commands_msg;
  commands_msg.reserve(commands.size());
  for (const auto& cmd : commands)
  {
    tesseract_msgs::EnvironmentCommand cmd_msg;
    if (tesseract_rosutils::toMsg(cmd_msg, cmd))
    {
      commands_msg.push_back(cmd_msg);
    }
    else
    {
      ROS_ERROR_STREAM_NAMED(monitor_namespace,
                             "Failed to convert latest changes to message and update monitored environment!");
      return false;
    }
  }

  return sendCommands(env_name_, monitor_namespace, commands_msg);
}

tesseract_scene_graph::SceneState
ROSEnvironmentMonitorInterface::getEnvironmentState(const std::string& monitor_namespace) const
{
  tesseract_msgs::GetEnvironmentInformation res;
  res.request.flags = tesseract_msgs::GetEnvironmentInformationRequest::JOINT_STATES |
                      tesseract_msgs::GetEnvironmentInformationRequest::LINK_TRANSFORMS |
                      tesseract_msgs::GetEnvironmentInformationRequest::JOINT_TRANSFORMS;

  bool status = ros::service::call(R"(/)" + monitor_namespace + DEFAULT_GET_ENVIRONMENT_INFORMATION_SERVICE, res);
  if (!status || !res.response.success)
    throw std::runtime_error("getEnvironmentState: Failed to get monitor environment information!");

  tesseract_scene_graph::SceneState env_state;
  tesseract_rosutils::fromMsg(env_state.joints, res.response.joint_states);
  tesseract_rosutils::fromMsg(env_state.floating_joints, res.response.floating_joint_states);
  tesseract_rosutils::fromMsg(env_state.link_transforms, res.response.link_transforms);
  tesseract_rosutils::fromMsg(env_state.joint_transforms, res.response.joint_transforms);

  return env_state;
}

bool ROSEnvironmentMonitorInterface::setEnvironmentState(const std::string& monitor_namespace,
                                                         const std::unordered_map<std::string, double>& joints,
                                                         const tesseract_common::TransformMap& floating_joints) const
{
  tesseract_msgs::EnvironmentCommand command;
  command.command = tesseract_msgs::EnvironmentCommand::UPDATE_JOINT_STATE;
  tesseract_rosutils::toMsg(command.joint_state, joints);
  tesseract_rosutils::toMsg(command.floating_joint_states, floating_joints);
  return sendCommands(env_name_, monitor_namespace, { command });
}

bool ROSEnvironmentMonitorInterface::setEnvironmentState(const std::string& monitor_namespace,
                                                         const std::vector<std::string>& joint_names,
                                                         const std::vector<double>& joint_values,
                                                         const tesseract_common::TransformMap& floating_joints) const
{
  std::unordered_map<std::string, double> joints;
  for (std::size_t i = 0; i < joint_names.size(); ++i)
    joints[joint_names[i]] = joint_values[i];

  tesseract_msgs::EnvironmentCommand command;
  command.command = tesseract_msgs::EnvironmentCommand::UPDATE_JOINT_STATE;
  tesseract_rosutils::toMsg(command.joint_state, joints);
  tesseract_rosutils::toMsg(command.floating_joint_states, floating_joints);
  return sendCommands(env_name_, monitor_namespace, { command });
}

bool ROSEnvironmentMonitorInterface::setEnvironmentState(const std::string& monitor_namespace,
                                                         const std::vector<std::string>& joint_names,
                                                         const Eigen::Ref<const Eigen::VectorXd>& joint_values,
                                                         const tesseract_common::TransformMap& floating_joints) const
{
  std::unordered_map<std::string, double> joints;
  for (std::size_t i = 0; i < joint_names.size(); ++i)
    joints[joint_names[i]] = joint_values[static_cast<Eigen::Index>(i)];

  tesseract_msgs::EnvironmentCommand command;
  command.command = tesseract_msgs::EnvironmentCommand::UPDATE_JOINT_STATE;
  tesseract_rosutils::toMsg(command.joint_state, joints);
  tesseract_rosutils::toMsg(command.floating_joint_states, floating_joints);
  return sendCommands(env_name_, monitor_namespace, { command });
}

bool ROSEnvironmentMonitorInterface::setEnvironmentState(const std::string& monitor_namespace,
                                                         const tesseract_common::TransformMap& floating_joints) const
{
  tesseract_msgs::EnvironmentCommand command;
  command.command = tesseract_msgs::EnvironmentCommand::UPDATE_JOINT_STATE;
  tesseract_rosutils::toMsg(command.floating_joint_states, floating_joints);
  return sendCommands(env_name_, monitor_namespace, { command });
}

std::vector<std::string>
ROSEnvironmentMonitorInterface::setEnvironmentState(const std::unordered_map<std::string, double>& joints,
                                                    const tesseract_common::TransformMap& floating_joints) const
{
  std::vector<std::string> failed_namespace;
  failed_namespace.reserve(ns_.size());
  for (const auto& ns : ns_)
    if (!setEnvironmentState(ns, joints, floating_joints))
      failed_namespace.push_back(ns);

  return failed_namespace;
}

std::vector<std::string>
ROSEnvironmentMonitorInterface::setEnvironmentState(const std::vector<std::string>& joint_names,
                                                    const std::vector<double>& joint_values,
                                                    const tesseract_common::TransformMap& floating_joints) const
{
  std::vector<std::string> failed_namespace;
  failed_namespace.reserve(ns_.size());
  for (const auto& ns : ns_)
    if (!setEnvironmentState(ns, joint_names, joint_values, floating_joints))
      failed_namespace.push_back(ns);

  return failed_namespace;
}

std::vector<std::string>
ROSEnvironmentMonitorInterface::setEnvironmentState(const std::vector<std::string>& joint_names,
                                                    const Eigen::Ref<const Eigen::VectorXd>& joint_values,
                                                    const tesseract_common::TransformMap& floating_joints) const
{
  std::vector<std::string> failed_namespace;
  failed_namespace.reserve(ns_.size());
  for (const auto& ns : ns_)
    if (!setEnvironmentState(ns, joint_names, joint_values, floating_joints))
      failed_namespace.push_back(ns);

  return failed_namespace;
}

std::vector<std::string>
ROSEnvironmentMonitorInterface::setEnvironmentState(const tesseract_common::TransformMap& floating_joints) const
{
  std::vector<std::string> failed_namespace;
  failed_namespace.reserve(ns_.size());
  for (const auto& ns : ns_)
    if (!setEnvironmentState(ns, floating_joints))
      failed_namespace.push_back(ns);

  return failed_namespace;
}

std::unique_ptr<tesseract_environment::Environment>
ROSEnvironmentMonitorInterface::getEnvironment(const std::string& monitor_namespace) const
{
  tesseract_msgs::GetEnvironmentInformation res;
  res.request.flags = tesseract_msgs::GetEnvironmentInformationRequest::COMMAND_HISTORY;

  bool status = ros::service::call(R"(/)" + monitor_namespace + DEFAULT_GET_ENVIRONMENT_INFORMATION_SERVICE, res);
  if (!status || !res.response.success)
  {
    ROS_ERROR_STREAM_NAMED(monitor_namespace, "getEnvironment: Failed to get monitor environment information!");
    return nullptr;
  }

  tesseract_environment::Commands commands;
  try
  {
    commands = tesseract_rosutils::fromMsg(res.response.command_history);
  }
  catch (...)
  {
    ROS_ERROR_STREAM_NAMED(monitor_namespace, "getEnvironment: Failed to convert command history message!");
    return nullptr;
  }

  auto env = std::make_unique<tesseract_environment::Environment>();
  env->init(commands);

  return env;
}

}  // namespace tesseract_monitoring
