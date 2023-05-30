#ifndef TESSERACT_RVIZ_ROS_SCENE_GRAPH_RENDER_MANAGER_H
#define TESSERACT_RVIZ_ROS_SCENE_GRAPH_RENDER_MANAGER_H

#include <memory>
#include <QObject>

#include <tesseract_qt/common/events/scene_graph_render_manager.h>

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
class ROSSceneGraphRenderManager : public tesseract_gui::SceneGraphRenderManager
{
public:
  ROSSceneGraphRenderManager(std::shared_ptr<const tesseract_gui::ComponentInfo> component_info,
                             Ogre::SceneManager* scene_manager,
                             Ogre::SceneNode* scene_node);
  ~ROSSceneGraphRenderManager();

private:
  struct Implementation;
  std::unique_ptr<Implementation> data_;

  void render() override;
};
}  // namespace tesseract_rviz

#endif  // TESSERACT_RVIZ_SCENE_GRAPH_RENDER_MANAGER_H
