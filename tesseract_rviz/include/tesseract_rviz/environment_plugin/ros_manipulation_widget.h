#ifndef TESSERACT_RVIZ_ROS_MANIPULATION_WIDGET_H
#define TESSERACT_RVIZ_ROS_MANIPULATION_WIDGET_H
#include <tesseract_widgets/manipulation/manipulation_widget.h>

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
struct ROSManipulationWidgetPrivate;

class ROSManipulationWidget : public tesseract_gui::ManipulationWidget
{
  Q_OBJECT
public:
  ROSManipulationWidget(Ogre::SceneManager* scene_manager, Ogre::SceneNode* scene_node);
  ~ROSManipulationWidget() override;

public Q_SLOTS:
  void onRender() override;
  void onLinkVisibilityChanged(const std::vector<std::string>& links);
  void onGroupNameChanged(const QString& group_name);
  void onManipulationStateChanged(const tesseract_scene_graph::SceneState& state, int state_index);

private Q_SLOTS:
  void onEnvironmentSet(const std::shared_ptr<const tesseract_environment::Environment>& env);

private:
  std::unique_ptr<ROSManipulationWidgetPrivate> data_;

  void clear();
  void clearContainer(const tesseract_gui::EntityContainer& container);
};
}  // namespace tesseract_rviz
#endif  // ROS_MANIPULATION_WIDGET_H
