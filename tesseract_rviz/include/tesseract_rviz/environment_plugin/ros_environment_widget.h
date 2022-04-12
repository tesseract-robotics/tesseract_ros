#ifndef TESSERACT_RVIZ_ROS_ENVIRONMENT_WIDGET_H
#define TESSERACT_RVIZ_ROS_ENVIRONMENT_WIDGET_H

#include <memory>

#include <tesseract_widgets/environment/environment_widget.h>

namespace Ogre
{
class SceneManager;
class SceneNode;
}  // namespace Ogre

namespace tesseract_gui
{
class EntityContainer;
}  // namespace tesseract_gui

namespace tesseract_rviz
{
struct ROSEnvironmentWidgetPrivate;

class ROSEnvironmentWidget : public tesseract_gui::EnvironmentWidget
{
  Q_OBJECT
public:
  ROSEnvironmentWidget(Ogre::SceneManager* scene_manager, Ogre::SceneNode* scene_node);
  ~ROSEnvironmentWidget() override;

public Q_SLOTS:
  void onUpdate();
  void onLinkVisibleChanged(const std::string& link_name, bool visible);
  void onLinkVisualVisibleChanged(const std::string& link_name, bool visible);
  void onLinkCollisionVisibleChanged(const std::string& link_name, bool visible);
  void onSelectedLinksChanged(const std::vector<std::string>& selected_links);

private Q_SLOTS:
  void onEnvironmentSet(const tesseract_environment::Environment& env);
  void onEnvironmentChanged(const tesseract_environment::Environment& env);
  void onEnvironmentCurrentStateChanged(const tesseract_environment::Environment& env);

protected:
  std::unique_ptr<ROSEnvironmentWidgetPrivate> data_;

  void clear();
  void clearContainer(const tesseract_gui::EntityContainer& container);
};

}  // namespace tesseract_rviz
#endif  // TESSERACT_RVIZ_ROS_ENVIRONMENT_WIDGET_H
