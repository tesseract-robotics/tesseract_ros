/**
 * @file conversions.cpp
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
#include <tesseract_common/macros.h>
TESSERACT_COMMON_IGNORE_WARNINGS_PUSH
#include <ros/console.h>
#include <eigen_conversions/eigen_msg.h>
#include <iostream>
#include <fstream>
#include <boost/algorithm/string/classification.hpp>
#include <boost/algorithm/string.hpp>
TESSERACT_COMMON_IGNORE_WARNINGS_POP

#include <tesseract_rosutils/conversions.h>
#include <tesseract_rosutils/utils.h>

namespace tesseract_rosutils
{
Eigen::VectorXd toEigen(const std::vector<double>& vector)
{
  return Eigen::VectorXd::Map(vector.data(), static_cast<long>(vector.size()));
}

Eigen::VectorXd toEigen(const sensor_msgs::JointState& joint_state, const std::vector<std::string>& joint_names)
{
  Eigen::VectorXd position;
  position.resize(static_cast<long>(joint_names.size()));
  int i = 0;
  for (const auto& joint_name : joint_names)
  {
    auto it = std::find(joint_state.name.begin(), joint_state.name.end(), joint_name);
    assert(it != joint_state.name.end());
    size_t index = static_cast<size_t>(std::distance(joint_state.name.begin(), it));
    position[i] = joint_state.position[index];
    ++i;
  }

  return position;
}

tesseract_msgs::ProcessPlanPath toProcessPlanPath(const tesseract_common::JointTrajectory& joint_trajectory)
{
  tesseract_msgs::ProcessPlanPath path;
  if (joint_trajectory.trajectory.size() != 0)
    tesseract_rosutils::toMsg(path.trajectory, joint_trajectory);
  return path;
}

bool toJointTrajectory(trajectory_msgs::JointTrajectory& joint_trajectory,
                       const tesseract_msgs::ProcessPlanSegment& process_plan_segment)
{
  double t = 0;
  if (!joint_trajectory.points.empty())
    t = joint_trajectory.points.back().time_from_start.toSec();

  for (const auto& point : process_plan_segment.approach.trajectory.points)
  {
    joint_trajectory.points.push_back(point);
    joint_trajectory.points.back().time_from_start.fromSec(t + point.time_from_start.toSec());
  }
  t = joint_trajectory.points.back().time_from_start.toSec();

  for (const auto& point : process_plan_segment.process.trajectory.points)
  {
    joint_trajectory.points.push_back(point);
    joint_trajectory.points.back().time_from_start.fromSec(t + point.time_from_start.toSec());
  }
  t = joint_trajectory.points.back().time_from_start.toSec();

  for (const auto& point : process_plan_segment.departure.trajectory.points)
  {
    joint_trajectory.points.push_back(point);
    joint_trajectory.points.back().time_from_start.fromSec(t + point.time_from_start.toSec());
  }

  return true;
}

bool toJointTrajectory(trajectory_msgs::JointTrajectory& joint_trajectory,
                       const tesseract_msgs::ProcessPlan& process_plan)
{
  double t = 0;
  if (!joint_trajectory.points.empty())
    t = joint_trajectory.points.back().time_from_start.toSec();

  for (const auto& point : process_plan.from_start.trajectory.points)
  {
    joint_trajectory.points.push_back(point);
    joint_trajectory.points.back().time_from_start.fromSec(t + point.time_from_start.toSec());
  }
  t = joint_trajectory.points.back().time_from_start.toSec();

  if (!process_plan.from_start.trajectory.points.empty())
  {
    joint_trajectory.joint_names = process_plan.segments[0].process.trajectory.joint_names;
    joint_trajectory.header = process_plan.segments[0].process.trajectory.header;
  }

  // Append process segments and transitions
  for (size_t i = 0; i < process_plan.segments.size(); ++i)
  {
    toJointTrajectory(joint_trajectory, process_plan.segments[i]);
    t = joint_trajectory.points.back().time_from_start.toSec();

    if (i < (process_plan.segments.size() - 1))
    {
      for (const auto& point : process_plan.transitions[i].from_end.trajectory.points)
      {
        joint_trajectory.points.push_back(point);
        joint_trajectory.points.back().time_from_start.fromSec(t + point.time_from_start.toSec());
      }
      t = joint_trajectory.points.back().time_from_start.toSec();
    }
  }

  // Append path to home
  for (const auto& point : process_plan.to_end.trajectory.points)
  {
    joint_trajectory.points.push_back(point);
    joint_trajectory.points.back().time_from_start.fromSec(t + point.time_from_start.toSec());
  }

  return true;
}

bool toCSVFile(const trajectory_msgs::JointTrajectory& joint_trajectory, const std::string& file_path, char separator)
{
  std::ofstream myfile;
  myfile.open(file_path);

  // Write Joint names as header
  std::copy(joint_trajectory.joint_names.begin(),
            joint_trajectory.joint_names.end(),
            std::ostream_iterator<std::string>(myfile, &separator));
  myfile << ",\n";
  for (const auto& point : joint_trajectory.points)
  {
    std::copy(point.positions.begin(), point.positions.end(), std::ostream_iterator<double>(myfile, &separator));
    myfile << "," + std::to_string(point.time_from_start.toSec()) + ",\n";
  }
  myfile.close();
  return true;
}

trajectory_msgs::JointTrajectory jointTrajectoryFromCSVFile(const std::string& file_path, char separator)
{
  trajectory_msgs::JointTrajectory joint_trajectory;
  std::ifstream csv_file(file_path);

  // Make sure the file is open
  if (!csv_file.is_open())
    throw std::runtime_error("Could not open csv file");

  // Helper vars
  std::string line, column_name;
  int num_joints = 0;

  // Read the column names
  if (csv_file.good())
  {
    // Extract the first line in the file
    std::getline(csv_file, line);

    // Create a stringstream from line
    std::stringstream ss(line);

    // Extract joint name
    std::vector<std::string> column_names;
    while (std::getline(ss, column_name, separator))
      column_names.push_back(column_name);

    for (std::size_t i = 0; i < column_names.size() - 1; ++i)
      joint_trajectory.joint_names.push_back(column_names[i]);

    num_joints = column_names.size() - 1;
  }

  // Read data, line by line
  while (std::getline(csv_file, line))
  {
    std::vector<std::string> tokens;
    boost::split(tokens, line, boost::is_any_of(std::string(&separator)), boost::token_compress_on);
    if (!tesseract_common::isNumeric(tokens))
      throw std::runtime_error("jointTrajectoryFromCSVFile: Invalid format");

    trajectory_msgs::JointTrajectoryPoint point;
    double tfs = 0;
    tesseract_common::toNumeric<double>(tokens.back(), tfs);
    point.time_from_start.fromSec(tfs);
    for (std::size_t i = 0; i < num_joints; ++i)
    {
      double val = 0;
      tesseract_common::toNumeric<double>(tokens[i], val);
      point.positions.push_back(val);
    }
    joint_trajectory.points.push_back(point);
  }

  // Close file
  csv_file.close();

  return joint_trajectory;
}

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
