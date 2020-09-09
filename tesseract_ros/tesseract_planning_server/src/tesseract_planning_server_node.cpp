#include <tesseract_common/macros.h>
TESSERACT_COMMON_IGNORE_WARNINGS_PUSH
#include <ros/ros.h>
TESSERACT_COMMON_IGNORE_WARNINGS_POP

#include <tesseract_environment/core/environment.h>
#include <tesseract_planning_server/tesseract_planning_server.h>

using namespace tesseract_environment;

const std::string ROBOT_DESCRIPTION_PARAM = "robot_description"; /**< Default ROS parameter for robot description */

int main(int argc, char** argv)
{
  ros::init(argc, argv, "tesseract_planning_server");
  ros::NodeHandle pnh("~");
  std::string robot_description;
  std::string descrete_plugin;
  std::string continuous_plugin;
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
  pnh.param<std::string>("descrete_plugin", descrete_plugin, "");
  pnh.param<std::string>("continuous_plugin", continuous_plugin, "");
  pnh.param<bool>("publish_environment", publish_environment, publish_environment);

  tesseract_planning_server::TesseractPlanningServer planning_server(
      robot_description, monitor_namespace, descrete_plugin, continuous_plugin);

  if (publish_environment)
    planning_server.getEnvironmentMonitor().startPublishingEnvironment(
        tesseract_monitoring::EnvironmentMonitor::UPDATE_ENVIRONMENT);

  if (!monitored_namespace.empty())
    planning_server.getEnvironmentMonitor().startMonitoringEnvironment(monitored_namespace);

  ros::AsyncSpinner spinner(4);
  spinner.start();

  ros::waitForShutdown();

  return 0;
}
