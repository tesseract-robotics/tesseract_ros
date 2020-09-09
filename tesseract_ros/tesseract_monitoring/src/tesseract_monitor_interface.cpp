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
#include <ros/console.h>
#include <tesseract_msgs/ModifyEnvironment.h>
#include <tesseract_msgs/EnvironmentCommand.h>
TESSERACT_COMMON_IGNORE_WARNINGS_POP

#include <tesseract_monitoring/tesseract_monitor_interface.h>

namespace tesseract_monitoring
{
void TesseractMonitorInterface::addNamespace(std::string monitor_namespace)
{
  if (std::find(ns_.begin(), ns_.end(), monitor_namespace) == ns_.end())
    ns_.push_back(monitor_namespace);
}

void TesseractMonitorInterface::removeNamespace(const std::string& monitor_namespace)
{
  auto it = std::remove_if(
      ns_.begin(), ns_.end(), [monitor_namespace](const std::string& ns) { return (ns == monitor_namespace); });
  ns_.erase(it, ns_.end());
}

bool sendCommands(const std::string& ns, const std::vector<tesseract_msgs::EnvironmentCommand>& commands)
{
  tesseract_msgs::ModifyEnvironment res;
  res.request.id = ns;
  res.request.append = true;
  res.request.commands = commands;

  bool status = ros::service::call(R"(/)" + ns + "/modify_tesseract", res);
  if (!status || !res.response.success)
  {
    ROS_ERROR_STREAM_NAMED(ns, "Failed to update monitored environment!");
    return false;
  }

  return true;
}

std::vector<std::string> TesseractMonitorInterface::applyCommand(const tesseract_environment::Command& command) const
{
  std::vector<std::string> failed_namespace;
  failed_namespace.reserve(ns_.size());
  for (const auto& ns : ns_)
    if (!applyCommand(ns, command))
      failed_namespace.push_back(ns);

  return failed_namespace;
}

std::vector<std::string> TesseractMonitorInterface::applyCommands(const tesseract_environment::Commands& commands) const
{
  std::vector<std::string> failed_namespace;
  failed_namespace.reserve(ns_.size());
  for (const auto& ns : ns_)
    if (!applyCommands(ns, commands))
      failed_namespace.push_back(ns);

  return failed_namespace;
}

std::vector<std::string>
TesseractMonitorInterface::applyCommands(const std::vector<tesseract_environment::Command>& commands) const
{
  std::vector<std::string> failed_namespace;
  failed_namespace.reserve(ns_.size());
  for (const auto& ns : ns_)
    if (!applyCommands(ns, commands))
      failed_namespace.push_back(ns);

  return failed_namespace;
}

bool TesseractMonitorInterface::applyCommand(const std::string& monitor_namespace,
                                             const tesseract_environment::Command& command) const
{
  tesseract_msgs::EnvironmentCommand command_msg;
  if (tesseract_rosutils::toMsg(command_msg, command))
  {
    return sendCommands(monitor_namespace, { command_msg });
  }
  else
  {
    ROS_ERROR_STREAM_NAMED(monitor_namespace,
                           "Failed to convert latest changes to message and update monitored environment!");
    return false;
  }
}

bool TesseractMonitorInterface::applyCommands(const std::string& monitor_namespace,
                                              const tesseract_environment::Commands& commands) const
{
  std::vector<tesseract_msgs::EnvironmentCommand> commands_msg;
  if (tesseract_rosutils::toMsg(commands_msg, commands, 0))
  {
    return sendCommands(monitor_namespace, commands_msg);
  }
  else
  {
    ROS_ERROR_STREAM_NAMED(monitor_namespace,
                           "Failed to convert latest changes to message and update monitored environment!");
    return false;
  }
}

bool TesseractMonitorInterface::applyCommands(const std::string& monitor_namespace,
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

  return sendCommands(monitor_namespace, commands_msg);
}
}  // namespace tesseract_monitoring
