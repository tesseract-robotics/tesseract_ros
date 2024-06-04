#include <tesseract_rviz/environment_display.h>
#include <tesseract_rviz/environment_monitor_properties.h>
#include <tesseract_rviz/set_theme_tool.h>

#include <tesseract_qt/common/component_info.h>
#include <tesseract_qt/common/component_info_manager.h>
#include <tesseract_qt/common/icon_utils.h>
#include <tesseract_qt/common/events/render_events.h>

#include <tesseract_qt/environment/widgets/environment_widget.h>

#include <OgreSceneNode.h>

#include <rviz/panel_dock_widget.h>

#include <QApplication>
#include <QLayout>

namespace tesseract_rviz
{
struct EnvironmentDisplay::Implementation
{
  Implementation()
  {
    environment_display_counter++;
    environment_display_id = environment_display_counter;
    environment_display_ns = "env_display_" + std::to_string(environment_display_id);
  }

  std::shared_ptr<SetThemeTool> theme_tool;

  tesseract_gui::EnvironmentWidget* widget{ nullptr };

  std::unique_ptr<EnvironmentMonitorProperties> monitor_properties{ nullptr };

  /** @brief Keeps track of how many EnvironmentDisplay's have been created for the default namespace */
  static int environment_display_counter;

  /** @brief Keeps track of which EnvironmentDisplay this is */
  int environment_display_id{ -1 };

  std::string environment_display_ns;
};

int EnvironmentDisplay::Implementation::environment_display_counter = -1;

EnvironmentDisplay::EnvironmentDisplay() : data_(std::make_unique<Implementation>())
{
  auto monitor_property = new rviz::Property("Environment Properties", "", "Tesseract environment properties", this);

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
  setIcon(tesseract_gui::icons::getTesseractIcon());
  data_->widget = new tesseract_gui::EnvironmentWidget(data_->monitor_properties->getComponentInfo());

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

  data_->theme_tool = SetThemeTool::instance();
  if (!data_->theme_tool->isInitialized())
    data_->theme_tool->initialized(context_);
}

void EnvironmentDisplay::reset() { Display::reset(); }

void EnvironmentDisplay::update(float wall_dt, float ros_dt)
{
  Display::update(wall_dt, ros_dt);

  if (data_->widget != nullptr)
  {
    tesseract_gui::events::PreRender event(data_->widget->getComponentInfo()->getSceneName());
    QApplication::sendEvent(qApp, &event);
  }
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

void EnvironmentDisplay::onEnable()
{
  Display::onEnable();
  //  data_->widget->setEnable(true);
}

void EnvironmentDisplay::onDisable() { Display::onDisable(); }

void EnvironmentDisplay::onEnableChanged()
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

void EnvironmentDisplay::onComponentInfoChanged(std::shared_ptr<const tesseract_gui::ComponentInfo> component_info)
{
  data_->widget->setComponentInfo(component_info);
}

}  // namespace tesseract_rviz
