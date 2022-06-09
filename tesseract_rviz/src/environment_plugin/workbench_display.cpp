#include <tesseract_rviz/environment_plugin/workbench_display.h>
#include <tesseract_rviz/environment_plugin/ros_environment_widget.h>
#include <tesseract_rviz/environment_plugin/ros_manipulation_widget.h>
#include <tesseract_rviz/environment_plugin/environment_monitor_properties.h>
#include <tesseract_rviz/environment_plugin/joint_trajectory_monitor_properties.h>
#include <tesseract_rviz/environment_plugin/set_theme_tool.h>

#include <tesseract_qt/workbench/workbench_widget.h>
#include <tesseract_qt/environment/environment_widget_config.h>
#include <tesseract_qt/joint_trajectory/joint_trajectory_widget.h>
#include <tesseract_qt/common/joint_trajectory_set.h>
#include <tesseract_qt/common/theme_utils.h>
#include <tesseract_qt/common/icon_utils.h>

#include <rviz/panel_dock_widget.h>
#include <rviz/display_context.h>
#include <rviz/window_manager_interface.h>

#include <OgreSceneNode.h>

#include <QApplication>
#include <QLayout>

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

  std::shared_ptr<SetThemeTool> theme_tool;

  tesseract_gui::WorkbenchWidget* widget{ nullptr };
  ROSEnvironmentWidget* environment_widget{ nullptr };
  tesseract_gui::JointTrajectoryWidget* joint_trajectory_widget{ nullptr };
  ROSManipulationWidget* manipulation_widget{ nullptr };

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

  setIcon(tesseract_gui::icons::getTesseractIcon());
  data_->environment_widget = new tesseract_rviz::ROSEnvironmentWidget(scene_manager_, scene_node_);  // NOLINT
  data_->joint_trajectory_widget = new tesseract_gui::JointTrajectoryWidget();                        // NOLINT
  data_->manipulation_widget = new ROSManipulationWidget(context_, scene_node_);                      // NOLINT
  // NOLINTNEXTLINE
  data_->widget = new tesseract_gui::WorkbenchWidget(
      data_->environment_widget, data_->joint_trajectory_widget, data_->manipulation_widget);

  setAssociatedWidget(data_->widget);

  getAssociatedWidget()->layout()->setSizeConstraint(QLayout::SetNoConstraint);
  getAssociatedWidget()->setSizePolicy(QSizePolicy::Expanding, QSizePolicy::Expanding);
  getAssociatedWidgetPanel()->setIcon(tesseract_gui::icons::getTesseractIcon());
  getAssociatedWidgetPanel()->layout()->setSizeConstraint(QLayout::SetNoConstraint);
  getAssociatedWidgetPanel()->setSizePolicy(QSizePolicy::Expanding, QSizePolicy::Expanding);

  disconnect(
      getAssociatedWidgetPanel(), SIGNAL(visibilityChanged(bool)), this, SLOT(associatedPanelVisibilityChange(bool)));

  data_->monitor_properties->onInitialize(data_->environment_widget);
  data_->joint_trajectory_properties->onInitialize(data_->joint_trajectory_widget);

  data_->theme_tool = SetThemeTool::instance();
  if (!data_->theme_tool->isInitialized())
    data_->theme_tool->initialized(context_);
}

void WorkbenchDisplay::onEnable()
{
  Display::onEnable();
  data_->widget->onEnable();
}

void WorkbenchDisplay::onDisable() { Display::onDisable(); }

void WorkbenchDisplay::reset() { Display::reset(); }

void WorkbenchDisplay::update(float wall_dt, float ros_dt)
{
  Display::update(wall_dt, ros_dt);

  data_->widget->onRender(wall_dt);
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

void WorkbenchDisplay::onEnableChanged()
{
  QApplication::setOverrideCursor(QCursor(Qt::WaitCursor));
  queueRender();
  /* We get here, by two different routes:
   * - First, we might have disabled the display.
   *   In this case we want to close/hide the associated widget.
   *   But there is an exception: tabbed DockWidgets shouldn't be hidden, because then we would loose the
   * tab.
   * - Second, the corresponding widget changed visibility and we got here via
   * associatedPanelVisibilityChange().
   *   In this case, it's usually counterproductive to show/hide the widget here.
   *   Typical cases are: main window was minimized/unminimized, tab was switched.
   */
  if (isEnabled())
  {
    scene_node_->setVisible(true);

    if (getAssociatedWidgetPanel() != nullptr)
    {
      getAssociatedWidgetPanel()->show();
      getAssociatedWidgetPanel()->setEnabled(true);
    }
    else if (getAssociatedWidget() != nullptr)
    {
      getAssociatedWidget()->show();
      getAssociatedWidgetPanel()->setEnabled(true);
    }

    if (isEnabled())  // status might have changed, e.g. if show() failed
      onEnable();
  }
  else
  {
    onDisable();

    if (getAssociatedWidgetPanel() != nullptr)
      getAssociatedWidgetPanel()->setDisabled(true);
    else if (getAssociatedWidget() != nullptr)
      getAssociatedWidget()->setDisabled(true);

    scene_node_->setVisible(false);
  }
  QApplication::restoreOverrideCursor();
}

}  // namespace tesseract_rviz
