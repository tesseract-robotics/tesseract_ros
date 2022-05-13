#include <tesseract_rviz/environment_plugin/ros_manipulation_widget.h>
#include <tesseract_rviz/environment_plugin/conversions.h>
#include <tesseract_rviz/environment_plugin/conversions.h>
#include <tesseract_rviz/conversions.h>

#include <tesseract_widgets/common/entity_manager.h>
#include <tesseract_widgets/common/link_visibility_properties.h>

#include <tesseract_environment/environment.h>

#include <set>

namespace tesseract_rviz
{
struct ROSManipulationWidgetPrivate
{
  ROSManipulationWidgetPrivate()
    : entity_managers(
          { std::make_shared<tesseract_gui::EntityManager>(), std::make_shared<tesseract_gui::EntityManager>() }){};

  Ogre::SceneManager* scene_manager;
  Ogre::SceneNode* scene_node;

  std::array<tesseract_gui::EntityManager::Ptr, 2> entity_managers;

  /** @brief The current render group */
  QString render_group;

  /** @brief Indicates if something has changed */
  bool render_dirty;

  /** @brief Used for delayed initialization */
  bool render_reset;

  /** @brief Update visualization current state from environment message */
  std::array<bool, 2> render_states_dirty;
  std::array<tesseract_scene_graph::SceneState, 2> render_states;
  std::array<tesseract_scene_graph::Material::ConstPtr, 2> render_states_visual_material;
  std::array<tesseract_scene_graph::Material::ConstPtr, 2> render_states_collision_material;

  /** @brief The links with changed visibility properties */
  std::set<std::string> link_visibility_properties_changed;
};

ROSManipulationWidget::ROSManipulationWidget(Ogre::SceneManager* scene_manager, Ogre::SceneNode* scene_node)
  : data_(std::make_unique<ROSManipulationWidgetPrivate>())
{
  data_->scene_manager = scene_manager;
  data_->scene_node = scene_node;

  addOgreResourceLocation();

  auto red = std::make_shared<tesseract_scene_graph::Material>("manipulation_red");
  red->color = Eigen::Vector4d(1, 0, 0, 0.75);

  auto green = std::make_shared<tesseract_scene_graph::Material>("manipulation_green");
  green->color = Eigen::Vector4d(0, 1, 0, 0.75);

  auto blue = std::make_shared<tesseract_scene_graph::Material>("manipulation_blue");
  blue->color = Eigen::Vector4d(0, 0, 1, 0.75);

  data_->render_states_visual_material[0] = red;
  data_->render_states_visual_material[1] = green;
  data_->render_states_collision_material[0] = blue;
  data_->render_states_collision_material[1] = blue;

  connect(this,
          SIGNAL(environmentSet(std::shared_ptr<const tesseract_environment::Environment>)),
          this,
          SLOT(onEnvironmentSet(std::shared_ptr<const tesseract_environment::Environment>)));

  connect(this, SIGNAL(groupNameChanged(QString)), this, SLOT(onGroupNameChanged(QString)));

  connect(this,
          SIGNAL(manipulationStateChanged(tesseract_scene_graph::SceneState, int)),
          this,
          SLOT(onManipulationStateChanged(tesseract_scene_graph::SceneState, int)));

  connect(this,
          SIGNAL(linkVisibilityChanged(std::vector<std::string>)),
          this,
          SLOT(onLinkVisibilityChanged(std::vector<std::string>)));
}

ROSManipulationWidget::~ROSManipulationWidget() = default;

void ROSManipulationWidget::clear()
{
  for (std::size_t i = 0; i < getStateCount(); ++i)
  {
    auto& entity_manager = data_->entity_managers[i];
    auto containers = entity_manager->getEntityContainers();
    for (auto& container : containers)
    {
      clearContainer(*container.second);
      container.second->clear();
      entity_manager->removeEntityContainer(container.first);
    }
  }
}

void ROSManipulationWidget::clearContainer(const tesseract_gui::EntityContainer& container)
{
  // Destroy Resources
  for (const auto& ns : container.getUntrackedEntities())
  {
    if (ns.first == container.RESOURCE_NS)
    {
      for (const auto& entity : ns.second)
        data_->scene_manager->destroyEntity(entity.unique_name);
    }
  }

  // Destroy Scene Nodes
  for (const auto& ns : container.getUntrackedEntities())
  {
    if (ns.first != container.RESOURCE_NS)
    {
      for (const auto& entity : ns.second)
        data_->scene_manager->destroySceneNode(entity.unique_name);
    }
  }

  for (const auto& ns : container.getTrackedEntities())
  {
    if (ns.first != container.RESOURCE_NS)
    {
      for (const auto& entity : ns.second)
        data_->scene_manager->destroySceneNode(entity.second.unique_name);
    }
  }
}

void ROSManipulationWidget::onEnvironmentSet(const std::shared_ptr<const tesseract_environment::Environment>& /*env*/)
{
  data_->render_group.clear();
  data_->render_dirty = true;
  data_->render_reset = true;
  data_->render_states_dirty = { true, true };
  data_->render_states = { tesseract_scene_graph::SceneState(), tesseract_scene_graph::SceneState() };
}

void ROSManipulationWidget::onLinkVisibilityChanged(const std::vector<std::string>& links)
{
  data_->link_visibility_properties_changed.insert(links.begin(), links.end());
  data_->render_dirty = true;
}

void ROSManipulationWidget::onGroupNameChanged(const QString& group_name)
{
  data_->render_group = group_name;
  data_->render_dirty = true;
  data_->render_reset = true;
  data_->render_states_dirty = { true, true };
  data_->render_states = { tesseract_scene_graph::SceneState(), tesseract_scene_graph::SceneState() };
}

void ROSManipulationWidget::onManipulationStateChanged(const tesseract_scene_graph::SceneState& state, int state_index)
{
  data_->render_dirty = true;
  data_->render_states_dirty[state_index] = true;
  data_->render_states[state_index] = state;
}

void ROSManipulationWidget::onRender()
{
  if (getEnvironment() == nullptr)
    return;

  if (data_->render_dirty)
  {
    auto lock = environment().lockRead();
    if (data_->render_reset)  // Remove all
    {
      auto& link_vis_props = getLinkVisibilityProperties();
      for (std::size_t i = 0; i < getStateCount(); ++i)
      {
        auto& entity_manager = data_->entity_managers[i];

        if (!entity_manager->empty())
        {
          clear();
          if (i == 0)
            link_vis_props.clear();
        }

        if (data_->render_dirty && !data_->render_group.isEmpty())
        {
          auto jg = environment().getJointGroup(data_->render_group.toStdString());
          std::vector<std::string> link_names = jg->getActiveLinkNames();
          for (const auto& link_name : link_names)
          {
            auto link = environment().getLink(link_name);
            auto entity_container = entity_manager->getEntityContainer(link->getName());
            Ogre::SceneNode* sn = loadLink(*data_->scene_manager,
                                           *entity_container,
                                           *link,
                                           data_->render_states_visual_material[i],
                                           data_->render_states_collision_material[i]);
            data_->scene_node->addChild(sn);
            if (i == 0)
              link_vis_props[link_name] = tesseract_gui::LinkVisibilityProperties();
          }
        }
      }
      data_->render_reset = false;
    }

    for (std::size_t i = 0; i < getStateCount(); ++i)
    {
      auto& entity_manager = data_->entity_managers[i];

      // Update start state visualization
      if (data_->render_states_dirty[i])
      {
        for (const auto& pair : data_->render_states[i].link_transforms)
        {
          if (entity_manager->hasEntityContainer(pair.first))
          {
            auto container = entity_manager->getEntityContainer(pair.first);
            Ogre::Vector3 position;
            Ogre::Quaternion orientation;
            toOgre(position, orientation, pair.second);

            auto entity = container->getTrackedEntity(tesseract_gui::EntityContainer::VISUAL_NS, pair.first);
            Ogre::SceneNode* sn = data_->scene_manager->getSceneNode(entity.unique_name);
            sn->setPosition(position);
            sn->setOrientation(orientation);
          }
        }
        data_->render_states_dirty[i] = false;
      }
    }

    {  // Update Link Visibility
      auto link_visibility_properties = getLinkVisibilityProperties();
      for (const auto& l : data_->link_visibility_properties_changed)
      {
        for (std::size_t i = 0; i < getStateCount(); ++i)
        {
          auto& entity_manager = data_->entity_managers[i];
          if (entity_manager->hasEntityContainer(l))
          {
            auto link_visibility_property = link_visibility_properties.at(l);
            auto entity_container = entity_manager->getEntityContainer(l);

            {  // Link Property
              if (entity_container->hasTrackedEntity(tesseract_gui::EntityContainer::VISUAL_NS, l))
              {
                auto entity = entity_container->getTrackedEntity(tesseract_gui::EntityContainer::VISUAL_NS, l);
                Ogre::SceneNode* sn = data_->scene_manager->getSceneNode(entity.unique_name);
                sn->setVisible(link_visibility_property.link, true);
              }
            }

            {  // Link Visual Property
              std::string visual_key = l + "::Visuals";
              if (entity_container->hasTrackedEntity(tesseract_gui::EntityContainer::VISUAL_NS, visual_key))
              {
                auto entity = entity_container->getTrackedEntity(tesseract_gui::EntityContainer::VISUAL_NS, visual_key);
                Ogre::SceneNode* sn = data_->scene_manager->getSceneNode(entity.unique_name);
                sn->setVisible(link_visibility_property.link && link_visibility_property.visual, true);
              }
            }

            {  // Link Collision Property
              std::string visual_key = l + "::Collisions";
              if (entity_container->hasTrackedEntity(tesseract_gui::EntityContainer::VISUAL_NS, visual_key))
              {
                auto entity = entity_container->getTrackedEntity(tesseract_gui::EntityContainer::VISUAL_NS, visual_key);
                Ogre::SceneNode* sn = data_->scene_manager->getSceneNode(entity.unique_name);
                sn->setVisible(link_visibility_property.link && link_visibility_property.collision, true);
              }
            }

            {  // Link WireBox Property
              std::string visual_key = l + "::WireBox";
              if (entity_container->hasTrackedEntity(tesseract_gui::EntityContainer::VISUAL_NS, visual_key))
              {
                auto entity = entity_container->getTrackedEntity(tesseract_gui::EntityContainer::VISUAL_NS, visual_key);
                Ogre::SceneNode* sn = data_->scene_manager->getSceneNode(entity.unique_name);
                sn->setVisible(link_visibility_property.link && link_visibility_property.wirebox, true);
              }
            }

            {  // Link Axis Property
              std::string visual_key = l + "::Axis";
              if (entity_container->hasTrackedEntity(tesseract_gui::EntityContainer::VISUAL_NS, visual_key))
              {
                auto entity = entity_container->getTrackedEntity(tesseract_gui::EntityContainer::VISUAL_NS, visual_key);
                Ogre::SceneNode* sn = data_->scene_manager->getSceneNode(entity.unique_name);
                sn->setVisible(link_visibility_property.link && link_visibility_property.axis, true);
              }
            }
          }
        }
      }
      data_->link_visibility_properties_changed.clear();
    }

    data_->render_dirty = false;
  }
}
}  // namespace tesseract_rviz
