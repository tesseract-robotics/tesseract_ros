/**
 * @file conversions.h
 * @brief Tesseract ROS Utils conversions
 *
 * @author Levi Armstrong
 * @date April 15, 2018
 * @version TODO
 * @bug No known bugs
 *
 * @copyright Copyright (c) 2018, Southwest Research Institute
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
#ifndef TESSERACT_ROSUTILS_CONVERSIONS_H
#define TESSERACT_ROSUTILS_CONVERSIONS_H

#include <tesseract_common/macros.h>
TESSERACT_COMMON_IGNORE_WARNINGS_PUSH
#include <Eigen/Core>
#include <Eigen/Geometry>
#include <sensor_msgs/JointState.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/PoseArray.h>
#include <trajectory_msgs/JointTrajectory.h>
#include <tesseract_msgs/ProcessPlanSegment.h>
#include <tesseract_msgs/ProcessPlan.h>
TESSERACT_COMMON_IGNORE_WARNINGS_POP

#include <tesseract_common/types.h>

namespace tesseract_rosutils
{
/**
 * @brief Convert STD Vector to Eigen Vector
 * @param vector The STD Vector to be converted
 * @return Eigen::VectorXd
 */
Eigen::VectorXd toEigen(const std::vector<double>& vector);

/**
 * @brief Converts JointState position to Eigen vector in the order provided by joint_names
 * @param joint_state The JointState
 * @param joint_names The vector joint names used to order the output
 * @return Eigen::VectorXd in the same order as joint_names
 */
Eigen::VectorXd toEigen(const sensor_msgs::JointState& joint_state, const std::vector<std::string>& joint_names);

/**
 * @brief Append a process segment to an existing joint_trajectory
 * @param joint_trajectory Trajectory to add the process segment
 * @param process_plan_segment Process plan segment
 * @return
 */
bool toJointTrajectory(trajectory_msgs::JointTrajectory& joint_trajectory,
                       const tesseract_msgs::ProcessPlanSegment& process_plan_segment);

/**
 * @brief Convert a Process Plan to a single Joint Trajectory for visualization
 * This does not clear the trajectory passed in it appends.
 * @param joint_trajectory Joint Trajectory Message
 * @param process_plan Process Plan
 * @return True if successful, otherwise false
 */
bool toJointTrajectory(trajectory_msgs::JointTrajectory& joint_trajectory,
                       const tesseract_msgs::ProcessPlan& process_plan);

/**
 * @brief Convert a joint trajector to csv formate and write to file
 * @param joint_trajectory Joint trajectory to be writen to file
 * @param file_path The location to save the file
 * @param separator The separator to use
 * @return true if successful
 */
bool toCSVFile(const trajectory_msgs::JointTrajectory& joint_trajectory,
               const std::string& file_path,
               char separator = ',');

/**
 * @brief Convert CSVFile to JointTrajectory
 * @param file_path File path to CSV
 * @param separator The separator to use
 * @return Joint Trajectory
 */
trajectory_msgs::JointTrajectory jointTrajectoryFromCSVFile(const std::string& file_path, char separator = ',');

/**
 * @brief Convert a Tesseract Trajectory to Process Plan Path
 * @param trajectory Tesseract Trajectory
 * @param joint_names Joint names corresponding to the tesseract trajectory
 * @return A process plan path
 */
tesseract_msgs::ProcessPlanPath toProcessPlanPath(const tesseract_common::JointTrajectory& joint_trajectory);

}  // namespace tesseract_rosutils
#endif  // TESSERACT_ROSUTILS_CONVERSIONS_H
