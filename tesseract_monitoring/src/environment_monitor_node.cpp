#include <tesseract_common/macros.h>
TESSERACT_COMMON_IGNORE_WARNINGS_PUSH
#include <ros/ros.h>
#include <ros/console.h>
TESSERACT_COMMON_IGNORE_WARNINGS_POP

#include <tesseract_environment/environment.h>
#include <tesseract_monitoring/environment_monitor.h>

using namespace tesseract_environment;

const std::string ROBOT_DESCRIPTION_PARAM = "robot_description"; /**< Default ROS parameter for robot description */

int main(int argc, char** argv)
{
  ros::init(argc, argv, "tesseract_environment_monitor");
  ros::NodeHandle pnh("~");
  std::string robot_description;
  std::string joint_state_topic;
  std::string monitor_namespace;
  std::string monitored_namespace;
  bool publish_environment{ false };

  if (!pnh.getParam("monitor_namespace", monitor_namespace))
  {
    ROS_ERROR("Missing required parameter monitor_namespace!");
    return 1;
  }

  pnh.param<std::string>("monitored_namespace", monitored_namespace, "");
  pnh.param<std::string>("robot_description", robot_description, ROBOT_DESCRIPTION_PARAM);
  pnh.param<std::string>("joint_state_topic", joint_state_topic, "");
  pnh.param<bool>("publish_environment", publish_environment, publish_environment);

  tesseract_monitoring::EnvironmentMonitor monitor(robot_description, monitor_namespace);

  if (publish_environment)
    monitor.startPublishingEnvironment();

  if (!monitored_namespace.empty())
    monitor.startMonitoringEnvironment(monitored_namespace);

  if (joint_state_topic.empty())
    monitor.startStateMonitor();
  else
    monitor.startStateMonitor(joint_state_topic);

  ros::AsyncSpinner spinner(4);
  spinner.start();

  ros::waitForShutdown();

  return 0;
}
