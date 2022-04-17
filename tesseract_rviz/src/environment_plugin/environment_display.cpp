#include <tesseract_rviz/environment_plugin/environment_display.h>
#include <tesseract_rviz/environment_plugin/ros_environment_widget.h>
#include <tesseract_rviz/environment_plugin/environment_monitor_properties.h>

#include <rviz/panel_dock_widget.h>

namespace tesseract_rviz
{
struct EnvironmentDisplayPrivate
{
  EnvironmentDisplayPrivate()
  {
    environment_display_counter++;
    environment_display_id = environment_display_counter;
    environment_display_ns = "env_display_" + std::to_string(environment_display_id);
  }

  ROSEnvironmentWidget* widget;

  std::unique_ptr<EnvironmentMonitorProperties> monitor_properties;

  /** @brief Keeps track of how many EnvironmentDisplay's have been created for the default namespace */
  static int environment_display_counter;

  /** @brief Keeps track of which EnvironmentDisplay this is */
  int environment_display_id;

  std::string environment_display_ns;
};

int EnvironmentDisplayPrivate::environment_display_counter = -1;

EnvironmentDisplay::EnvironmentDisplay() : data_(std::make_unique<EnvironmentDisplayPrivate>())
{
  auto monitor_property =
      new rviz::Property("Environment Properties", "", "Tesseract environment properties", this, nullptr, this);

  data_->monitor_properties =
      std::make_unique<EnvironmentMonitorProperties>(this, data_->environment_display_ns, monitor_property);
}

EnvironmentDisplay::~EnvironmentDisplay()
{
  auto* panel = getAssociatedWidgetPanel();
  panel->close();
}

void EnvironmentDisplay::onInitialize()
{
  Display::onInitialize();
  data_->widget = new tesseract_rviz::ROSEnvironmentWidget(scene_manager_, scene_node_);
  setAssociatedWidget(data_->widget);

  data_->monitor_properties->onInitialize(data_->widget);
}

void EnvironmentDisplay::reset() { Display::reset(); }

void EnvironmentDisplay::update(float wall_dt, float ros_dt)
{
  Display::update(wall_dt, ros_dt);

  data_->widget->onRender();
}

void EnvironmentDisplay::load(const rviz::Config& config)
{
  rviz::Display::load(config);
  data_->monitor_properties->load(config);
}

void EnvironmentDisplay::save(rviz::Config config) const
{
  data_->monitor_properties->save(config);
  rviz::Display::save(config);
}

}  // namespace tesseract_rviz
