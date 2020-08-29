/**
 * @file plotting.h
 * @brief Tesseract ROS Basic plotting functions.
 *
 * @author Levi Armstrong
 * @date April 15, 2018
 * @version TODO
 * @bug No known bugs
 *
 * @copyright Copyright (c) 2013, Southwest Research Institute
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
#ifndef TESSERACT_ROSUTILS_PLOTTING_H
#define TESSERACT_ROSUTILS_PLOTTING_H

#include <tesseract_common/macros.h>
TESSERACT_COMMON_IGNORE_WARNINGS_PUSH
#include <ros/publisher.h>
#include <tesseract_msgs/Trajectory.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>
TESSERACT_COMMON_IGNORE_WARNINGS_POP

#include <tesseract_visualization/visualization.h>

namespace tesseract_rosutils
{
/** @brief The BasicPlotting class */
class ROSPlotting : public tesseract_visualization::Visualization
{
public:
  ROSPlotting(std::string root_link = "world", std::string topic_namespace = "tesseract");

  void plotTrajectory(const tesseract_msgs::Trajectory& traj);

  void plotTrajectory(const std::vector<std::string>& joint_names,
                      const Eigen::Ref<const tesseract_common::TrajArray>& traj) override;

  bool init(tesseract::Tesseract::ConstPtr thor) override;

  void plotTrajectory(const tesseract_common::JointTrajectory& traj) override;

  void plotTrajectory(const tesseract_planning::Instruction& instruction) override;

  void plotTrajectory(const std::vector<std::string>& joint_names,
                      const Eigen::Ref<const tesseract_common::TrajArray>& traj,
                      const tesseract_environment::Environment::ConstPtr& env);

  void plotToolPath(const tesseract_planning::Instruction& instruction) override;

  void plotContactResults(const std::vector<std::string>& link_names,
                          const tesseract_collision::ContactResultVector& dist_results,
                          const Eigen::Ref<const Eigen::VectorXd>& safety_distances) override;

  void plotArrow(const Eigen::Ref<const Eigen::Vector3d>& pt1,
                 const Eigen::Ref<const Eigen::Vector3d>& pt2,
                 const Eigen::Ref<const Eigen::Vector4d>& rgba,
                 double scale) override;

  void plotAxis(const Eigen::Isometry3d& axis, double scale) override;

  void clear() override;

  void waitForInput() override;

  static visualization_msgs::Marker getMarkerArrowMsg(int& id_counter,
                                                      const std::string& frame_id,
                                                      const std::string& ns,
                                                      const ros::Time& time_stamp,
                                                      const Eigen::Ref<const Eigen::Vector3d>& pt1,
                                                      const Eigen::Ref<const Eigen::Vector3d>& pt2,
                                                      const Eigen::Ref<const Eigen::Vector4d>& rgba,
                                                      const double scale);

  static visualization_msgs::Marker getMarkerCylinderMsg(int& id_counter,
                                                         const std::string& frame_id,
                                                         const std::string& ns,
                                                         const ros::Time& time_stamp,
                                                         const Eigen::Ref<const Eigen::Vector3d>& pt1,
                                                         const Eigen::Ref<const Eigen::Vector3d>& pt2,
                                                         const Eigen::Ref<const Eigen::Vector4d>& rgba,
                                                         const double scale);

  static visualization_msgs::MarkerArray
  getContactResultsMarkerArrayMsg(int& id_counter,
                                  const std::string& frame_id,
                                  const std::string& ns,
                                  const ros::Time& time_stamp,
                                  const std::vector<std::string>& link_names,
                                  const tesseract_collision::ContactResultVector& dist_results,
                                  const Eigen::Ref<const Eigen::VectorXd>& safety_distances);

private:
  std::string root_link_;         /**< Root link of markers */
  std::string topic_namespace_;   /**< Namespace used when publishing markers */
  int marker_counter_;            /**< Counter when plotting */
  ros::Publisher scene_pub_;      /**< Scene publisher */
  ros::Publisher trajectory_pub_; /**< Trajectory publisher */
  ros::Publisher collisions_pub_; /**< Collision Data publisher */
  ros::Publisher arrows_pub_;     /**< Used for publishing arrow markers */
  ros::Publisher axes_pub_;       /**< Used for publishing axis markers */
};
using ROSPlottingPtr = std::shared_ptr<ROSPlotting>;
using ROSPlottingConstPtr = std::shared_ptr<const ROSPlotting>;
}  // namespace tesseract_rosutils

#endif
