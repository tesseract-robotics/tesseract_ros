/**
 * @file tesseract_planning_server.h
 * @brief A planning server with a default set of motion planners
 *
 * @author Levi Armstrong
 * @date August 18, 2020
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
#ifndef TESSERACT_ROS_TESSERACT_PLANNING_SERVER_H
#define TESSERACT_ROS_TESSERACT_PLANNING_SERVER_H

#include <tesseract_common/macros.h>
TESSERACT_COMMON_IGNORE_WARNINGS_PUSH
#include <memory>
#include <string>
TESSERACT_COMMON_IGNORE_WARNINGS_POP

#include <tesseract_environment/fwd.h>
#include <tesseract_task_composer/core/fwd.h>
#include <tesseract_command_language/fwd.h>

namespace tesseract_planning_server
{
class TesseractPlanningServer
{
public:
  static const std::string DEFAULT_PLANNER_PROFILES_PARAM;  // "/tesseract_planner_profiles"

  static const std::string DEFAULT_GET_MOTION_PLAN_ACTION;  // "/tesseract_get_motion_plan"

  TesseractPlanningServer(const std::string& robot_description, std::string name);
  TesseractPlanningServer(std::unique_ptr<tesseract_environment::Environment> env, std::string name);
  ~TesseractPlanningServer();
  TesseractPlanningServer(const TesseractPlanningServer&) = delete;
  TesseractPlanningServer& operator=(const TesseractPlanningServer&) = delete;
  TesseractPlanningServer(TesseractPlanningServer&&) = delete;
  TesseractPlanningServer& operator=(TesseractPlanningServer&&) = delete;

  tesseract_environment::EnvironmentMonitor& getEnvironmentMonitor();
  const tesseract_environment::EnvironmentMonitor& getEnvironmentMonitor() const;

  tesseract_planning::TaskComposerServer& getTaskComposerServer();
  const tesseract_planning::TaskComposerServer& getTaskComposerServer() const;

  tesseract_environment::EnvironmentCache& getEnvironmentCache();
  const tesseract_environment::EnvironmentCache& getEnvironmentCache() const;

  tesseract_planning::ProfileDictionary& getProfileDictionary();
  const tesseract_planning::ProfileDictionary& getProfileDictionary() const;

private:
  struct Implementation;
  std::unique_ptr<Implementation> impl_;
};
}  // namespace tesseract_planning_server
#endif  // TESSERACT_ROS_TESSERACT_PLANNING_SERVER_H
