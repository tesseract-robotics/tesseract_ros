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

///**
// * @brief Convert a cartesian pose to cartesian waypoint.
// * @param pose The cartesian pose
// * @param change_base A tranformation applied to the pose = change_base * pose
// * @return WaypointPtr
// */
// inline tesseract_motion_planners::Waypoint::Ptr
// toWaypoint(const geometry_msgs::Pose& pose,
//           const Eigen::Isometry3d& change_base = Eigen::Isometry3d::Identity(),
//           const std::string& parent_link = "")
//{
//  Eigen::Isometry3d pose_eigen;
//  tf::poseMsgToEigen(pose, pose_eigen);
//  return std::make_shared<tesseract_motion_planners::CartesianWaypoint>(change_base * pose_eigen, parent_link);
//}

///**
// * @brief Convert a vector of cartesian poses to vector of cartesian waypoints
// * @param poses The vector of cartesian poses
// * @param change_base A tranformation applied to the pose = change_base * pose
// * @return std::vector<WaypointPtr>
// */
// inline std::vector<tesseract_motion_planners::Waypoint::Ptr>
// toWaypoint(const std::vector<geometry_msgs::Pose>& poses,
//           const Eigen::Isometry3d& change_base = Eigen::Isometry3d::Identity(),
//           const std::string& parent_link = "")
//{
//  std::vector<tesseract_motion_planners::Waypoint::Ptr> waypoints;
//  waypoints.reserve(poses.size());
//  for (const auto& pose : poses)
//    waypoints.push_back(toWaypoint(pose, change_base, parent_link));

//  return waypoints;
//}

///**
// * @brief Convert a list of vector of cartesian poses to list of vector of cartesian waypoints
// * @param pose_arrays The list of vector of cartesian poses
// * @param change_base A tranformation applied to the pose = change_base * pose
// * @return std::vector<std::vector<WaypointPtr>>
// */
// inline std::vector<std::vector<tesseract_motion_planners::Waypoint::Ptr>>
// toWaypoint(const std::vector<geometry_msgs::PoseArray>& pose_arrays,
//           const Eigen::Isometry3d& change_base = Eigen::Isometry3d::Identity(),
//           const std::string& parent_link = "")
//{
//  std::vector<std::vector<tesseract_motion_planners::Waypoint::Ptr>> paths;
//  paths.reserve(pose_arrays.size());
//  for (const auto& pose_array : pose_arrays)
//    paths.push_back(toWaypoint(pose_array.poses, change_base, parent_link));

//  return paths;
//}

///**
// * @brief Convert a vector of double to joint waypoint
// * @param pose The joint positions
// * @return WaypointPtr
// */
// inline tesseract_motion_planners::Waypoint::Ptr toWaypoint(const std::vector<double>& pose,
//                                                           const std::vector<std::string>& names)
//{
//  return std::make_shared<tesseract_motion_planners::JointWaypoint>(toEigen(pose), names);
//}

// inline tesseract_motion_planners::Waypoint::Ptr toWaypoint(const sensor_msgs::JointState& joint_state)
//{
//  assert(joint_state.name.size() == joint_state.position.size());
//  std::vector<std::string> joint_names = joint_state.name;
//  Eigen::VectorXd joint_positions(joint_state.position.size());
//  for (long i = 0; i < static_cast<long>(joint_state.position.size()); ++i)
//    joint_positions[i] = joint_state.position[static_cast<size_t>(i)];

//  return std::make_shared<tesseract_motion_planners::JointWaypoint>(joint_positions, joint_names);
//}

///**
// * @brief Convert a process plan segment to a process plan segment message
// * @param segment_results The process segment plan
// * @param joint_names Joint names corresponding to the tesseract trajectory
// * @return Process Segment Message
// */
// inline tesseract_msgs::ProcessPlanSegment
// toProcessPlanSegement(const tesseract_process_planners::ProcessSegmentPlan& process_plan_segment)
//{
//  tesseract_msgs::ProcessPlanSegment process_segment;
//  process_segment.approach = toProcessPlanPath(process_plan_segment.approach);
//  process_segment.process = toProcessPlanPath(process_plan_segment.process);
//  process_segment.departure = toProcessPlanPath(process_plan_segment.departure);
//  return process_segment;
//}

}  // namespace tesseract_rosutils
#endif  // TESSERACT_ROSUTILS_CONVERSIONS_H
