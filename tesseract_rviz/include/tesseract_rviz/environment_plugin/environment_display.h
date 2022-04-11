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

  void onEnvironmentSet(const tesseract_environment::Environment& env);
  void onEnvironmentChanged(const tesseract_environment::Environment& env);
  void onEnvironmentCurrentStateChanged(const tesseract_environment::Environment& env);
  void onLinkVisibleChanged(const std::string& link_name, bool visible);
  void onLinkCollisionVisibleChanged(const std::string& link_name, bool visible);
  void onLinkVisualVisibleChanged(const std::string& link_name, bool visible);
  void onSelectedLinksChanged(const std::vector<std::string>& selected_links);

  void onSceneAlphaChanged();
  void onSceneVisualVisibleChanged();
  void onSceneCollisionVisibleChanged();
  void onSceneLinkVisibleChanged();

protected:
  // overrides from Display
  void onInitialize() override;
  void onEnable() override;
  void onDisable() override;
  void onUpdate();

  std::unique_ptr<EnvironmentDisplayPrivate> data_;

  void clear();
  void clearContainer(const tesseract_gui::EntityContainer& container);
};
}  // namespace tesseract_rviz
#endif  // TESSERACT_RVIZ_ENVIRONMENT_DISPLAY_H
