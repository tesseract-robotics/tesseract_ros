#ifndef TESSERACT_RVIZ_ROS_ENVIRONMENT_WIDGET_H
#define TESSERACT_RVIZ_ROS_ENVIRONMENT_WIDGET_H

#include <memory>

#include <tesseract_qt/environment/environment_widget.h>

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

  tesseract_gui::EnvironmentWidget* clone() const override;

public Q_SLOTS:
  void onRender(float dt) override;
  void onLinkVisibilityChanged(const std::vector<std::string>& links);
  void onShowContactResults(const tesseract_collision::ContactResultVector& contact_results);

private Q_SLOTS:
  void onEnvironmentSet(const std::shared_ptr<tesseract_environment::Environment>& env);
  void onEnvironmentChanged(const tesseract_environment::Environment& env);
  void onEnvironmentCurrentStateChanged(const tesseract_environment::Environment& env);

private:
  std::unique_ptr<ROSEnvironmentWidgetPrivate> data_;

  void clear();
  void clearContainer(const tesseract_gui::EntityContainer& container);
};

}  // namespace tesseract_rviz
#endif  // TESSERACT_RVIZ_ROS_ENVIRONMENT_WIDGET_H
