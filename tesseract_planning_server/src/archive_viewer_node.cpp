/**
 * @file archive_viewer_node.cpp
 * @brief Node for viewing motion planning archives
 *
 * @author Matthew Powelson
 * @date September 22, 2020
 * @version TODO
 * @bug No known bugs
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
#include <ros/ros.h>
#include <system_error>
#include <boost/system/error_code.hpp>
#include <boost/range/iterator_range.hpp>

#include <tesseract_command_language/core/serialization.h>

TESSERACT_COMMON_IGNORE_WARNINGS_POP

#include <tesseract_msgs/PlanningRequestArchive.h>
#include <tesseract_msgs/PlanningResponseArchive.h>
#include <tesseract_rosutils/utils.h>
#include <tesseract_rosutils/plotting.h>
#include <tesseract_common/types.h>

const std::string ARCHIVE_TOPIC_NAME = "/planning_response_archive";

using namespace tesseract_rosutils;
using namespace tesseract_planning;

class ArchiveViewer
{
public:
  ArchiveViewer()
  {
    plotter_ = std::make_shared<ROSPlotting>("world", "archive_viewer");
    plotter_->waitForConnection(5);
    ros::NodeHandle nh;
    publisher_ = nh.advertise<tesseract_msgs::PlanningResponseArchive>(ARCHIVE_TOPIC_NAME, 1, true);
  }

  bool viewArchive(const std::string& filepath)
  {
    auto archive = fromFile<tesseract_msgs::PlanningResponseArchive>(filepath);

    // Convert to objects
    tesseract_msgs::PlanningRequestArchive request_archive = archive.planning_request;
    Instruction instructions{ NullInstruction() };
    Instruction seed{ NullInstruction() };
    Instruction results{ NullInstruction() };
    if (!request_archive.instructions.empty())
      instructions = Serialization::fromArchiveStringXML<Instruction>(request_archive.instructions);
    if (!request_archive.seed.empty())
      seed = Serialization::fromArchiveStringXML<Instruction>(request_archive.seed);
    if (!archive.results.empty())
      results = Serialization::fromArchiveStringXML<Instruction>(archive.results);

    // Print debugging info
    ROS_INFO_STREAM("Request Name: " << request_archive.name);

    instructions.print("Instructions: ");
    seed.print("Seed: ");
    results.print("Results: ");

    // Publish for plotting in rviz
    publisher_.publish(archive);

    return true;
  }

  bool viewArchiveDirectory(const std::string& dir_path)
  {
    const tesseract_common::fs::path path(dir_path);

    for (auto& entry : boost::make_iterator_range(tesseract_common::fs::directory_iterator(path), {}))
    {
      std::string filepath = entry.path().string();
      ROS_INFO_STREAM("Viewing archive: " << filepath);

      viewArchive(filepath);
      plotter_->waitForInput();
    }

    ROS_INFO("All archives viewed");

    return true;
  }

private:
  tesseract_rosutils::ROSPlottingPtr plotter_;
  ros::Publisher publisher_;
};

int main(int argc, char** argv)
{
  ros::init(argc, argv, "archive_viewer");
  ros::NodeHandle pnh("~");

  std::string path;

  if (!pnh.getParam("path", path))
  {
    ROS_ERROR("Missing required parameter path!");
    return 1;
  }

  ArchiveViewer viewer;

  namespace fs = boost::filesystem;

  const tesseract_common::fs::path boost_path(path);
  boost::system::error_code ec;

  if (tesseract_common::fs::is_directory(path, ec))
  {
    ROS_INFO("Path given is a directory.");
    viewer.viewArchiveDirectory(boost_path.string());
  }
  if (ec)
  {
    ROS_ERROR_STREAM("Error in is_directory: " << ec.message());
  }

  if (tesseract_common::fs::is_regular_file(path, ec))
  {
    ROS_INFO("Path given is a file");
    viewer.viewArchive(boost_path.string());
  }
  if (ec)
  {
    ROS_ERROR_STREAM("Error in is_regular_file: " << ec.message());
  }

  ros::waitForShutdown();

  return 0;
}
