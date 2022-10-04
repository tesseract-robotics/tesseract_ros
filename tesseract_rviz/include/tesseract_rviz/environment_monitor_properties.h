#ifndef TESSERACT_RVIZ_ENVIRONMENT_MONITOR_PROPERTIES_H
#define TESSERACT_RVIZ_ENVIRONMENT_MONITOR_PROPERTIES_H

#include <memory>
#include <QObject>

namespace rviz
{
class Display;
class Config;
class Property;
}  // namespace rviz

namespace tesseract_gui
{
class EnvironmentWidgetConfig;
}

namespace tesseract_rviz
{
class ROSEnvironmentWidget;
class EnvironmentMonitorPropertiesPrivate;

class EnvironmentMonitorProperties : public QObject
{
  Q_OBJECT
public:
  EnvironmentMonitorProperties(rviz::Display* parent,
                               std::string monitor_namespace,
                               rviz::Property* main_property = nullptr);
  ~EnvironmentMonitorProperties() override;

  void onInitialize(ROSEnvironmentWidget* widget);

  /**
   * @brief Return the config based on the settings of the object
   * @return The environment config
   */
  std::shared_ptr<tesseract_gui::EnvironmentWidgetConfig> getConfig() const;

  void load(const rviz::Config& config);
  void save(rviz::Config config) const;

public Q_SLOTS:
  void onDisplayModeChanged();
  void onURDFDescriptionChanged();
  void onEnvironmentTopicChanged();
  void onJointStateTopicChanged();

protected:
  std::unique_ptr<EnvironmentMonitorPropertiesPrivate> data_;
};

}  // namespace tesseract_rviz

#endif  // TESSERACT_RVIZ_ENVIRONMENT_MONITOR_PROPERTIES_H
