#ifndef TESSERACT_RVIZ_ENVIRONMENT_MONITORING_H
#define TESSERACT_RVIZ_ENVIRONMENT_MONITORING_H

#include <tesseract_common/macros.h>
TESSERACT_COMMON_IGNORE_WARNINGS_PUSH
#include <rviz/display.h>
TESSERACT_COMMON_IGNORE_WARNINGS_POP

#ifndef Q_MOC_RUN

TESSERACT_COMMON_IGNORE_WARNINGS_PUSH
#include <ros/ros.h>
#include <ros/service_server.h>
#include <ros/subscriber.h>
#include <tesseract_monitoring/environment_monitor.h>
#include <std_msgs/ColorRGBA.h>
#include <tesseract_environment/core/environment.h>
TESSERACT_COMMON_IGNORE_WARNINGS_POP

#endif
#include <tesseract_rviz/render_tools/visualization_widget.h>

namespace rviz
{
class Property;
class RosTopicProperty;
class StringProperty;
class FloatProperty;
}  // namespace rviz

namespace tesseract_rviz
{
class EnvironmentWidget : public QObject
{
  Q_OBJECT

public:
  using Ptr = std::shared_ptr<EnvironmentWidget>;
  using ConstPtr = std::shared_ptr<const EnvironmentWidget>;

  EnvironmentWidget(rviz::Property* widget, rviz::Display* display, const std::string& widget_ns = std::string());

  virtual ~EnvironmentWidget();

  void onInitialize(VisualizationWidget::Ptr visualization,
                    tesseract_environment::Environment::Ptr env,
                    rviz::DisplayContext* context,
                    const ros::NodeHandle& update_nh,
                    bool update_state);

  void onEnable();
  void onDisable();
  void onUpdate();
  void onReset();

  /** @brief Returns the ID of this EnvironmentWidget instance which is associated with the default namespace */
  int getId() const { return environment_widget_id_; }

private Q_SLOTS:
  void changedRootLinkName();
  void changedDisplayMode();
  void changedDisplayModeString();
  void changedURDFSceneAlpha();
  void changedEnableLinkHighlight();
  void changedEnableVisualVisible();
  void changedEnableCollisionVisible();
  void changedAllLinks();

protected:
  rviz::Property* widget_;
  rviz::Display* display_;
  VisualizationWidget::Ptr visualization_;
  tesseract_environment::Environment::Ptr env_;
  ros::NodeHandle nh_;
  std::unique_ptr<tesseract_monitoring::EnvironmentMonitor> monitor_;
  int revision_{ 0 }; /**< The current revision of the visualization environment */
  bool update_required_;
  bool update_state_;    /**< @brief Update visualization current state from environment message */
  bool load_tesseract_;  // for delayed initialization
  std::map<std::string, std_msgs::ColorRGBA> highlights_;
  std::chrono::high_resolution_clock::duration state_timestamp_{
    std::chrono::high_resolution_clock::now().time_since_epoch()
  };

  void loadEnvironment();

  /** @brief Apply a list of commands to the environment. This used by both services and topics for updating environment
   * visualization */
  bool applyEnvironmentCommands(const tesseract_environment::Command& command);

  rviz::Property* main_property_;
  rviz::EnumProperty* display_mode_property_;
  rviz::StringProperty* display_mode_string_property_;
  rviz::StringProperty* environment_namespace_property_;
  rviz::StringProperty* root_link_name_property_;
  rviz::FloatProperty* alpha_property_;
  rviz::BoolProperty* enable_link_highlight_;
  rviz::BoolProperty* enable_visual_visible_;
  rviz::BoolProperty* enable_collision_visible_;
  rviz::BoolProperty* show_all_links_;

private:
  /** @brief Keeps track of how many EnvironmentWidgets have been created for the default namespace */
  static int environment_widget_counter_;
  /** @brief Keeps track of which EnvironmentWidget this is */
  int environment_widget_id_;

  std::string widget_ns_;
};
}  // namespace tesseract_rviz
#endif  // TESSERACT_RVIZ_ENVIRONMENT_MONITORING_H
