/**
 * @file online_planning_example_node.cpp
 * @brief This example demonstrates using trajopt to plan in an "online" manner. As the environment is changed (using
 * the joint state publisher), the system will attempt to avoid collisions with the moving object and follow the moving
 * target
 *
 * Note: If the target moves too quickly the solver can get stuck in an infeasible point. That is the nature of how the
 * solver is working. Higher level intelligence, a larger step size, or changing the target to a cost can solve that
 * problem.
 *
 * @author Matthew Powelson
 * @date June 9, 2020
 * @version TODO
 * @bug No known bugs
 * @copyright Copyright (c) 2017, Southwest Research Institute
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
#include <ros/node_handle.h>
#include <std_srvs/SetBool.h>
#include <sensor_msgs/JointState.h>
#include <ros/subscriber.h>
#include <ros/service.h>
TESSERACT_COMMON_IGNORE_WARNINGS_POP

#include <tesseract_examples/online_planning_example.h>
#include <tesseract_scene_graph/graph.h>
#include <tesseract_environment/environment.h>
#include <tesseract_monitoring/environment_monitor.h>
#include <tesseract_rosutils/plotting.h>
#include <tesseract_rosutils/utils.h>

using namespace tesseract_examples;
using namespace tesseract_rosutils;

/** @brief Default ROS parameter for robot description */
const std::string ROBOT_DESCRIPTION_PARAM = "robot_description";

/** @brief Default ROS parameter for robot description */
const std::string ROBOT_SEMANTIC_PARAM = "robot_description_semantic";

/** @brief RViz Example Namespace */
const std::string EXAMPLE_MONITOR_NAMESPACE = "tesseract_ros_examples";

/** @brief Dynamic object joint states topic */
const std::string DYNAMIC_OBJECT_JOINT_STATE = "/joint_states";

int main(int argc, char** argv)
{
  ros::init(argc, argv, "online_planning_example_node");
  ros::NodeHandle pnh("~");
  ros::NodeHandle nh;

  bool plotting{ true };
  bool rviz{ true };
  int steps{ 12 };
  double box_size{ 0.01 };
  bool update_start_state{ false };
  bool use_continuous{ false };

  // Get ROS Parameters
  pnh.param("plotting", plotting, plotting);
  pnh.param("rviz", rviz, rviz);
  pnh.param<int>("steps", steps, steps);
  pnh.param<double>("box_size", box_size, box_size);
  pnh.param<bool>("update_start_state", update_start_state, update_start_state);
  pnh.param<bool>("use_continuous", use_continuous, use_continuous);

  // Initial setup
  std::string urdf_xml_string, srdf_xml_string;
  nh.getParam(ROBOT_DESCRIPTION_PARAM, urdf_xml_string);
  nh.getParam(ROBOT_SEMANTIC_PARAM, srdf_xml_string);

  auto env = std::make_shared<tesseract_environment::Environment>();
  auto locator = std::make_shared<tesseract_rosutils::ROSResourceLocator>();
  if (!env->init(urdf_xml_string, srdf_xml_string, locator))
    exit(1);

  // Create monitor
  auto monitor = std::make_shared<tesseract_monitoring::ROSEnvironmentMonitor>(env, EXAMPLE_MONITOR_NAMESPACE);
  if (rviz)
    monitor->startPublishingEnvironment();

  ROSPlottingPtr plotter;
  if (plotting)
    plotter = std::make_shared<ROSPlotting>(env->getSceneGraph()->getRoot());

  OnlinePlanningExample example(env, plotter, steps, box_size, update_start_state, use_continuous);

  auto fn1 = [&example](std_srvs::SetBool::Request& req, std_srvs::SetBool::Response& res) {
    example.toggleRealtime(req.data);
    res.success = true;
    return true;
  };

  auto fn2 = [&example](const sensor_msgs::JointState::ConstPtr& joint_state) {
    example.updateState(joint_state->name, joint_state->position);
  };

  // Set up ROS interfaces
  ros::Subscriber joint_state_subscriber = nh.subscribe<sensor_msgs::JointState>(DYNAMIC_OBJECT_JOINT_STATE, 1, fn2);
  ros::ServiceServer toggle_realtime_service =
      nh.advertiseService<std_srvs::SetBool::Request, std_srvs::SetBool::Response>("toggle_realtime", fn1);

  example.run();
}
