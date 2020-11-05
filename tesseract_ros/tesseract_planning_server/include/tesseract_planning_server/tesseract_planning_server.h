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
#include <deque>
#include <shared_mutex>
#include <ros/ros.h>
#include <actionlib/server/simple_action_server.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <tesseract_msgs/GetMotionPlanAction.h>
TESSERACT_COMMON_IGNORE_WARNINGS_POP

#include <tesseract_monitoring/environment_monitor.h>
#include <tesseract_motion_planners/descartes/profile/descartes_profile.h>
#include <tesseract_motion_planners/trajopt/profile/trajopt_profile.h>
#include <tesseract_motion_planners/ompl/profile/ompl_profile.h>
#include <tesseract_motion_planners/descartes/profile/descartes_profile.h>
#include <tesseract_motion_planners/simple/profile/simple_planner_profile.h>

namespace tesseract_planning_server
{
class TesseractPlanningServer
{
public:
  static const std::string DEFAULT_PLANNER_PROFILES_PARAM;  // "/tesseract_planner_profiles"

  static const std::string DEFAULT_GET_MOTION_PLAN_ACTION;  // "/tesseract_get_motion_plan"

  TesseractPlanningServer(const std::string& robot_description,
                          std::string name,
                          std::string discrete_plugin = "",
                          std::string continuous_plugin = "");

  TesseractPlanningServer(std::shared_ptr<tesseract::Tesseract> tesseract,
                          std::string name,
                          std::string discrete_plugin = "",
                          std::string continuous_plugin = "");

  ~TesseractPlanningServer() = default;
  TesseractPlanningServer(const TesseractPlanningServer&) = delete;
  TesseractPlanningServer& operator=(const TesseractPlanningServer&) = delete;
  TesseractPlanningServer(TesseractPlanningServer&&) = delete;
  TesseractPlanningServer& operator=(TesseractPlanningServer&&) = delete;

  tesseract_monitoring::EnvironmentMonitor& getEnvironmentMonitor();
  const tesseract_monitoring::EnvironmentMonitor& getEnvironmentMonitor() const;

  void onMotionPlanningCallback(const tesseract_msgs::GetMotionPlanGoalConstPtr& goal);

  /**
   * @brief Set the cache size used to hold tesseract objects for motion planning
   * @param size The size of the cache.
   */
  void setCacheSize(long size);

  /**
   * @brief Get the cache size used to hold tesseract objects for motion planning
   * @return The size of the cache.
   */
  long getCacheSize() const;

  /** @brief If the environment has changed it will rebuild the cache of tesseract objects */
  void refreshCache();

  /**
   * @brief This will pop a Tesseract object from the queue
   * @details This will first call refreshCache to ensure it has an updated tesseract then proceed
   */
  tesseract::Tesseract::Ptr getCachedTesseract();

protected:
  ros::NodeHandle nh_;

  /** @brief The environment monitor to keep the planning server updated with the latest */
  tesseract_monitoring::EnvironmentMonitor environment_;

  /** @brief The motion planning action server */
  actionlib::SimpleActionServer<tesseract_msgs::GetMotionPlanAction> motion_plan_server_;

  /** @brief Trajopt available composite profiles */
  tesseract_planning::TrajOptCompositeProfileMap trajopt_composite_profiles_;

  /**@brief The trajopt available plan profiles */
  tesseract_planning::TrajOptPlanProfileMap trajopt_plan_profiles_;

  /** @brief The Descartes available plan profiles */
  tesseract_planning::DescartesPlanProfileMap<double> descartes_plan_profiles_;

  /** @brief The OMPL available plan profiles */
  tesseract_planning::OMPLPlanProfileMap ompl_plan_profiles_;

  /** @brief The Simple Planner available plan profiles */
  tesseract_planning::SimplePlannerPlanProfileMap simple_plan_profiles_;

  /** @brief The Simple Planner available composite profiles */
  tesseract_planning::SimplePlannerCompositeProfileMap simple_composite_profiles_;

  /** @brief TF buffer to track TCP transforms */
  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;

  /** @brief TF listener to lookup TCP transforms */
  tf2_ros::TransformListener tf_listener_;

  /** @brief The environment revision number at the time the cache was populated */
  int cache_env_revision_{ 0 };

  /** @brief The assigned cache size */
  std::size_t cache_size_{ 5 };

  /** @brief A vector of cached Tesseact objects */
  std::deque<tesseract::Tesseract::Ptr> cache_;

  /** @brief The mutex used when reading and writing to cache_ */
  mutable std::shared_mutex cache_mutex_;

  void ctor();

  void loadDefaultPlannerProfiles();

  Eigen::Isometry3d tfFindTCP(const tesseract_planning::ManipulatorInfo& manip_info);
};
}  // namespace tesseract_planning_server
#endif  // TESSERACT_ROS_TESSERACT_PLANNING_SERVER_H
