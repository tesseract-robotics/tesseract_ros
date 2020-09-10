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

    num_joints = static_cast<int>(column_names.size()) - 1;
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
    for (std::size_t i = 0; i < static_cast<std::size_t>(num_joints); ++i)
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

}  // namespace tesseract_rosutils
