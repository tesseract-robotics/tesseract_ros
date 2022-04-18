#include <tesseract_rviz/environment_plugin/workbench_display.h>
#include <tesseract_rviz/environment_plugin/ros_environment_widget.h>
#include <tesseract_rviz/environment_plugin/environment_monitor_properties.h>
#include <tesseract_rviz/environment_plugin/joint_trajectory_monitor_properties.h>

#include <tesseract_widgets/workbench/workbench_widget.h>
#include <tesseract_widgets/environment/environment_widget_config.h>
#include <tesseract_widgets/common/joint_trajectory_set.h>
#include <tesseract_widgets/joint_trajectory/joint_trajectory_widget.h>

#include <rviz/panel_dock_widget.h>

namespace tesseract_rviz
{
struct WorkbenchDisplayPrivate
{
  WorkbenchDisplayPrivate()
  {
    workbench_display_counter++;
    workbench_display_id = workbench_display_counter;
    workbench_display_ns = "workbench_display_" + std::to_string(workbench_display_id);
  }

  tesseract_gui::WorkbenchWidget* widget{ nullptr };
  ROSEnvironmentWidget* environment_widget{ nullptr };

  std::unique_ptr<EnvironmentMonitorProperties> monitor_properties{ nullptr };
  std::unique_ptr<JointTrajectoryMonitorProperties> joint_trajectory_properties{ nullptr };

  /** @brief Keeps track of how many WorkbenchDisplay's have been created for the default namespace */
  static int workbench_display_counter;  // NOLINT

  /** @brief Keeps track of which WorkbenchDisplay this is */
  int workbench_display_id{ -1 };

  std::string workbench_display_ns;
};

int WorkbenchDisplayPrivate::workbench_display_counter = -1;  // NOLINT

WorkbenchDisplay::WorkbenchDisplay() : data_(std::make_unique<WorkbenchDisplayPrivate>())
{
  auto* monitor_property =
      new rviz::Property("Environment Properties", "", "Tesseract environment properties", this, nullptr, this);
  auto* joint_trajectory_property = new rviz::Property(
      "Joint Trajectory Properties", "", "Tesseract joint trajectory properties", this, nullptr, this);

  data_->monitor_properties =
      std::make_unique<EnvironmentMonitorProperties>(this, data_->workbench_display_ns, monitor_property);
  data_->joint_trajectory_properties =
      std::make_unique<JointTrajectoryMonitorProperties>(this, joint_trajectory_property);
}

WorkbenchDisplay::~WorkbenchDisplay()
{
  auto* panel = getAssociatedWidgetPanel();
  panel->close();
}

void WorkbenchDisplay::onInitialize()
{
  Display::onInitialize();
  data_->environment_widget = new tesseract_rviz::ROSEnvironmentWidget(scene_manager_, scene_node_);  // NOLINT
  data_->widget = new tesseract_gui::WorkbenchWidget(data_->environment_widget);                      // NOLINT
  setAssociatedWidget(data_->widget);

  data_->monitor_properties->onInitialize(data_->environment_widget);
  data_->joint_trajectory_properties->onInitialize(&data_->widget->getJointTrajectoryWidget());
}

void WorkbenchDisplay::reset() { Display::reset(); }

void WorkbenchDisplay::update(float wall_dt, float ros_dt)
{
  Display::update(wall_dt, ros_dt);

  data_->widget->onRender();
}

void WorkbenchDisplay::load(const rviz::Config& config)
{
  rviz::Display::load(config);
  data_->monitor_properties->load(config);
}

void WorkbenchDisplay::save(rviz::Config config) const
{
  data_->monitor_properties->save(config);
  rviz::Display::save(config);
}

}  // namespace tesseract_rviz
