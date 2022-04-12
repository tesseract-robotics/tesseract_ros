#ifndef TESSERACT_RVIZ_ENVIRONMENT_DISPLAY_H
#define TESSERACT_RVIZ_ENVIRONMENT_DISPLAY_H

#include <rviz/display.h>
#include <tesseract_environment/environment.h>
#include <tesseract_widgets/common/entity_container.h>

namespace tesseract_rviz
{
struct EnvironmentDisplayPrivate;

class EnvironmentDisplay : public rviz::Display
{
  Q_OBJECT
public:
  EnvironmentDisplay();
  ~EnvironmentDisplay() override;

  void update(float wall_dt, float ros_dt) override;
  void reset() override;

  void load(const rviz::Config& config) override;
  void save(rviz::Config config) const override;

private Q_SLOTS:
  void onDisplayModeChanged();
  void onURDFDescriptionChanged();
  void onEnvironmentTopicChanged();
  void onJointStateTopicChanged();

  void onSceneAlphaChanged();
  void onSceneVisualVisibleChanged();
  void onSceneCollisionVisibleChanged();
  void onSceneWireBoxVisibleChanged();
  void onSceneLinkVisibleChanged();

protected:
  // overrides from Display
  void onInitialize() override;

  std::unique_ptr<EnvironmentDisplayPrivate> data_;
};
}  // namespace tesseract_rviz
#endif  // TESSERACT_RVIZ_ENVIRONMENT_DISPLAY_H
