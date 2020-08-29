#ifndef TESSERACT_ROS_TESSERACT_PLANNING_SERVER_H
#define TESSERACT_ROS_TESSERACT_PLANNING_SERVER_H

#include <tesseract_common/macros.h>
TESSERACT_COMMON_IGNORE_WARNINGS_PUSH
#include <ros/ros.h>
TESSERACT_COMMON_IGNORE_WARNINGS_POP

#include <actionlib/server/simple_action_server.h>
#include <tesseract_msgs/GetMotionPlanAction.h>
#include <tesseract_motion_planners/descartes/profile/descartes_profile.h>
#include <tesseract_monitoring/environment_monitor.h>
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

protected:
  ros::NodeHandle nh_;
  ros::NodeHandle root_nh_;

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

  void loadDefaultPlannerProfiles();
};
}  // namespace tesseract_planning_server
#endif  // TESSERACT_ROS_TESSERACT_PLANNING_SERVER_H
