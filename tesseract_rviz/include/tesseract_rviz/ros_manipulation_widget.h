#ifndef TESSERACT_RVIZ_ROS_MANIPULATION_WIDGET_H
#define TESSERACT_RVIZ_ROS_MANIPULATION_WIDGET_H
#include <tesseract_qt/manipulation/manipulation_widget.h>

namespace Ogre
{
class SceneNode;
}  // namespace Ogre

namespace rviz
{
class DisplayContext;
}
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
  ROSManipulationWidget(rviz::DisplayContext* context, Ogre::SceneNode* scene_node);
  ~ROSManipulationWidget() override;

public Q_SLOTS:
  void onRender(float dt) override;
  void onLinkVisibilityChanged(const std::vector<std::string>& links);
  void onGroupNameChanged(const QString& group_name);
  void onManipulationStateChanged(const tesseract_scene_graph::SceneState& state, int state_index);

private Q_SLOTS:
  void onEnvironmentSet(const std::shared_ptr<const tesseract_environment::Environment>& env);
  void onModeChanged(int mode);
  void onTCPChanged();
  void markerFeedback(const std::string& reference_frame,
                      const Eigen::Isometry3d& transform,
                      const Eigen::Vector3d& mouse_point,
                      bool mouse_point_valid);
  void jointMarkerFeedback(const std::string& joint_name,
                           const std::string& reference_frame,
                           const Eigen::Isometry3d& transform,
                           const Eigen::Vector3d& mouse_point,
                           bool mouse_point_valid);

private:
  std::unique_ptr<ROSManipulationWidgetPrivate> data_;

  void clear();
  void clearContainer(const tesseract_gui::EntityContainer& container);
  void addInteractiveMarker();
};
}  // namespace tesseract_rviz
#endif  // ROS_MANIPULATION_WIDGET_H
