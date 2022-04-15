#ifndef TESSERACT_RVIZ_ENVIRONMENT_VISUAL_PROPERTIES_H
#define TESSERACT_RVIZ_ENVIRONMENT_VISUAL_PROPERTIES_H

#include <memory>
#include <QObject>

namespace rviz
{
class Display;
class Config;
class Property;
}  // namespace rviz

namespace tesseract_rviz
{
class ROSEnvironmentWidget;
class EnvironmentVisualPropertiesPrivate;

class EnvironmentVisualProperties : public QObject
{
  Q_OBJECT
public:
  EnvironmentVisualProperties(rviz::Display* parent, rviz::Property* main_property = nullptr);
  ~EnvironmentVisualProperties() override;

  void onInitialize(ROSEnvironmentWidget* widget);

  void load(const rviz::Config& config);
  void save(rviz::Config config) const;

public Q_SLOTS:
  void onSceneAlphaChanged();
  void onSceneVisualVisibleChanged();
  void onSceneCollisionVisibleChanged();
  void onSceneWireBoxVisibleChanged();
  void onSceneLinkVisibleChanged();

protected:
  std::unique_ptr<EnvironmentVisualPropertiesPrivate> data_;
};

}  // namespace tesseract_rviz

#endif  // TESSERACT_RVIZ_ENVIRONMENT_VISUAL_PROPERTIES_H
