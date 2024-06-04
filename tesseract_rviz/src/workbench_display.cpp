#include <tesseract_rviz/workbench_display.h>
#include <tesseract_rviz/environment_monitor_properties.h>
#include <tesseract_rviz/joint_trajectory_monitor_properties.h>
#include <tesseract_rviz/set_theme_tool.h>

#include <tesseract_qt/workbench/workbench_widget.h>
#include <tesseract_qt/joint_trajectory/widgets/joint_trajectory_widget.h>
#include <tesseract_qt/common/component_info.h>
#include <tesseract_qt/common/component_info_manager.h>
#include <tesseract_qt/common/joint_trajectory_set.h>
#include <tesseract_qt/common/theme_utils.h>
#include <tesseract_qt/common/icon_utils.h>
#include <tesseract_qt/common/events/render_events.h>

#include <rviz/panel_dock_widget.h>
#include <rviz/display_context.h>
#include <rviz/window_manager_interface.h>

#include <OgreSceneNode.h>

#include <QApplication>
#include <QLayout>

namespace tesseract_rviz
{
struct WorkbenchDisplay::Implementation
{
  Implementation()
  {
    workbench_display_counter++;
    workbench_display_id = workbench_display_counter;
    workbench_display_ns = "workbench_display_" + std::to_string(workbench_display_id);
  }

  std::shared_ptr<SetThemeTool> theme_tool;

  tesseract_gui::WorkbenchWidget* widget{ nullptr };

  std::unique_ptr<EnvironmentMonitorProperties> monitor_properties{ nullptr };
  std::unique_ptr<JointTrajectoryMonitorProperties> joint_trajectory_properties{ nullptr };

  /** @brief Keeps track of how many WorkbenchDisplay's have been created for the default namespace */
  static int workbench_display_counter;  // NOLINT

  /** @brief Keeps track of which WorkbenchDisplay this is */
  int workbench_display_id{ -1 };

  std::string workbench_display_ns;
};

int WorkbenchDisplay::Implementation::workbench_display_counter = -1;  // NOLINT

WorkbenchDisplay::WorkbenchDisplay() : data_(std::make_unique<Implementation>())
{
  auto* monitor_property = new rviz::Property("Environment Properties", "", "Tesseract environment properties", this);
  auto* joint_trajectory_property =
      new rviz::Property("Joint Trajectory Properties", "", "Tesseract joint trajectory properties", this);

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
  // NOLINTNEXTLINE
  data_->widget = new tesseract_gui::WorkbenchWidget(data_->monitor_properties->getComponentInfo());

  setAssociatedWidget(data_->widget);

  getAssociatedWidget()->layout()->setSizeConstraint(QLayout::SetNoConstraint);
  getAssociatedWidget()->setSizePolicy(QSizePolicy::Expanding, QSizePolicy::Expanding);
  getAssociatedWidgetPanel()->setIcon(tesseract_gui::icons::getTesseractIcon());
  getAssociatedWidgetPanel()->layout()->setSizeConstraint(QLayout::SetNoConstraint);
  getAssociatedWidgetPanel()->setSizePolicy(QSizePolicy::Expanding, QSizePolicy::Expanding);

  disconnect(
      getAssociatedWidgetPanel(), SIGNAL(visibilityChanged(bool)), this, SLOT(associatedPanelVisibilityChange(bool)));

  connect(data_->monitor_properties.get(),
          SIGNAL(componentInfoChanged(std::shared_ptr<const tesseract_gui::ComponentInfo>)),
          this,
          SLOT(onComponentInfoChanged(std::shared_ptr<const tesseract_gui::ComponentInfo>)));

  data_->monitor_properties->onInitialize(scene_manager_, scene_node_);
  data_->joint_trajectory_properties->onInitialize();

  data_->theme_tool = SetThemeTool::instance();
  if (!data_->theme_tool->isInitialized())
    data_->theme_tool->initialized(context_);
}

void WorkbenchDisplay::onEnable()
{
  Display::onEnable();
  //  data_->widget->onEnable();
}

void WorkbenchDisplay::onDisable() { Display::onDisable(); }

void WorkbenchDisplay::reset() { Display::reset(); }

void WorkbenchDisplay::update(float wall_dt, float ros_dt)
{
  Display::update(wall_dt, ros_dt);

  if (data_->widget != nullptr)
  {
    tesseract_gui::events::PreRender event(data_->widget->getComponentInfo()->getSceneName());
    QApplication::sendEvent(qApp, &event);
  }
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

void WorkbenchDisplay::onComponentInfoChanged(std::shared_ptr<const tesseract_gui::ComponentInfo> component_info)
{
  data_->widget->setComponentInfo(std::move(component_info));
  data_->joint_trajectory_properties->setComponentInfo(data_->widget->getJointTrajectoryWidget().getComponentInfo());
}

}  // namespace tesseract_rviz
