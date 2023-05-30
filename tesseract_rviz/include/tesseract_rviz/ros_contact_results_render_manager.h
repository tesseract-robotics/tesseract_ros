#ifndef TESSERACT_RVIZ_ROS_CONTACT_RESULTS_RENDER_MANAGER_H
#define TESSERACT_RVIZ_ROS_CONTACT_RESULTS_RENDER_MANAGER_H

#include <memory>
#include <QObject>

#include <tesseract_qt/common/events/contact_results_render_manager.h>

namespace Ogre
{
class SceneManager;
class SceneNode;
}  // namespace Ogre

namespace tesseract_gui
{
class EntityManager;
struct ComponentInfo;
}  // namespace tesseract_gui

namespace tesseract_rviz
{
class ROSContactResultsRenderManager : public tesseract_gui::ContactResultsRenderManager
{
public:
  ROSContactResultsRenderManager(std::shared_ptr<const tesseract_gui::ComponentInfo> component_info,
                                 Ogre::SceneManager* scene_manager,
                                 Ogre::SceneNode* scene_node);
  ~ROSContactResultsRenderManager();

private:
  struct Implementation;
  std::unique_ptr<Implementation> data_;

  void render() override;
};
}  // namespace tesseract_rviz

#endif  // TESSERACT_RVIZ_ROS_CONTACT_RESULTS_RENDER_MANAGER_H
