#include <tesseract_rviz/environment_plugin/workbench_display.h>
#include <tesseract_rviz/environment_plugin/ros_environment_widget.h>
#include <tesseract_rviz/environment_plugin/environment_monitor_properties.h>
#include <tesseract_rviz/environment_plugin/environment_visual_properties.h>
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

  tesseract_gui::WorkbenchWidget* widget;

  ROSEnvironmentWidget* environment_widget;
  std::unordered_map<std::string, tesseract_gui::EnvironmentWidgetConfig::Ptr> environment_configs;

  std::unique_ptr<EnvironmentMonitorProperties> monitor_properties;
  std::unique_ptr<EnvironmentVisualProperties> visual_properties;
  std::unique_ptr<JointTrajectoryMonitorProperties> joint_trajectory_properties;

  /** @brief Keeps track of how many WorkbenchDisplay's have been created for the default namespace */
  static int workbench_display_counter;

  /** @brief Keeps track of which WorkbenchDisplay this is */
  int workbench_display_id;

  std::string workbench_display_ns;
};

int WorkbenchDisplayPrivate::workbench_display_counter = -1;

WorkbenchDisplay::WorkbenchDisplay() : data_(std::make_unique<WorkbenchDisplayPrivate>())
{
  auto monitor_property =
      new rviz::Property("Environment Properties", "", "Tesseract environment properties", this, nullptr, this);
  auto visual_property =
      new rviz::Property("Scene Properties", "", "Tesseract environment visualization properties", this, nullptr, this);
  auto joint_trajectory_property = new rviz::Property(
      "Joint Trajectory Properties", "", "Tesseract joint trajectory properties", this, nullptr, this);

  data_->monitor_properties =
      std::make_unique<EnvironmentMonitorProperties>(this, data_->workbench_display_ns, monitor_property);
  data_->visual_properties = std::make_unique<EnvironmentVisualProperties>(this, visual_property);
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
  data_->environment_widget = new tesseract_rviz::ROSEnvironmentWidget(scene_manager_, scene_node_);
  data_->widget = new tesseract_gui::WorkbenchWidget(data_->environment_widget);
  setAssociatedWidget(data_->widget);

  data_->monitor_properties->onInitialize(data_->environment_widget);
  data_->visual_properties->onInitialize(data_->environment_widget);
  data_->joint_trajectory_properties->onInitialize(&data_->widget->getJointTrajectoryWidget());

  connect(&data_->widget->getJointTrajectoryWidget(),
          SIGNAL(configureJointTrajectorySet(QString, tesseract_common::JointTrajectorySet)),
          this,
          SLOT(onConfigureJointTrajectorySet(QString, tesseract_common::JointTrajectorySet)));
  connect(&data_->widget->getJointTrajectoryWidget(),
          SIGNAL(jointTrajectorySetRemoved(QString)),
          this,
          SLOT(onJointTrajectorySetRemoved(QString)));
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
  data_->visual_properties->load(config);
}

void WorkbenchDisplay::save(rviz::Config config) const
{
  data_->monitor_properties->save(config);
  data_->visual_properties->save(config);
  rviz::Display::save(config);
}

void WorkbenchDisplay::onConfigureJointTrajectorySet(const QString& uuid,
                                                     const tesseract_common::JointTrajectorySet& joint_trajectory_set)
{
  tesseract_gui::EnvironmentWidgetConfig::Ptr environment_config;

  auto it = data_->environment_configs.find(uuid.toStdString());
  if (it == data_->environment_configs.end())
  {
    environment_config = std::make_shared<tesseract_gui::EnvironmentWidgetConfig>();
    if (joint_trajectory_set.getEnvironment() != nullptr)
    {
      environment_config->setEnvironment(joint_trajectory_set.getEnvironment()->clone());
      tesseract_common::JointState joint_state = joint_trajectory_set.getInitialState();
      std::unordered_map<std::string, double> initial_state;
      for (std::size_t i = 0; i < joint_state.joint_names.size(); ++i)
        initial_state[joint_state.joint_names[i]] = joint_state.position[static_cast<Eigen::Index>(i)];
      environment_config->environment().setState(initial_state);
      data_->environment_configs[uuid.toStdString()] = environment_config;
    }
  }
  else
  {
    environment_config = it->second;
  }

  data_->environment_widget->setConfiguration(environment_config);
}

void WorkbenchDisplay::onJointTrajectorySetRemoved(const QString& uuid)
{
  data_->environment_configs.erase(uuid.toStdString());
}

}  // namespace tesseract_rviz
