#ifndef TESSERACT_RVIZ_ENVIRONMENT_MONITOR_PROPERTIES_H
#define TESSERACT_RVIZ_ENVIRONMENT_MONITOR_PROPERTIES_H

#include <memory>
#include <QObject>
#include <tesseract_qt/common/component_info.h>

namespace rviz
{
class Display;
class Config;
class Property;
}  // namespace rviz

namespace Ogre
{
class SceneManager;
class SceneNode;
}  // namespace Ogre

namespace tesseract_gui
{
class EnvironmentWidget;
}

namespace tesseract_rviz
{
class EnvironmentMonitorProperties : public QObject
{
  Q_OBJECT
public:
  EnvironmentMonitorProperties(rviz::Display* parent,
                               std::string monitor_namespace,
                               rviz::Property* main_property = nullptr);
  ~EnvironmentMonitorProperties() override;

  void onInitialize(Ogre::SceneManager* scene_manager, Ogre::SceneNode* scene_node);

  /**
   * @brief Return the component info based on the settings of the object
   * @return The component info
   */
  tesseract_gui::ComponentInfo getComponentInfo() const;

  void load(const rviz::Config& config);
  void save(rviz::Config config) const;

Q_SIGNALS:
  void componentInfoChanged(tesseract_gui::ComponentInfo component_info);

public Q_SLOTS:
  void onDisplayModeChanged();
  void onURDFDescriptionChanged();
  void onEnvironmentTopicChanged();
  void onJointStateTopicChanged();

protected:
  struct Implementation;
  std::unique_ptr<Implementation> data_;
};

}  // namespace tesseract_rviz

#endif  // TESSERACT_RVIZ_ENVIRONMENT_MONITOR_PROPERTIES_H
