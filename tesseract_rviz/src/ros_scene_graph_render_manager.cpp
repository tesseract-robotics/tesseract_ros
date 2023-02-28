#include <tesseract_rviz/ros_scene_graph_render_manager.h>
#include <tesseract_rviz/conversions.h>

#include <tesseract_qt/common/utils.h>
#include <tesseract_qt/common/entity_manager.h>
#include <tesseract_qt/common/entity_container.h>
#include <tesseract_qt/common/component_info.h>
#include <tesseract_qt/common/events/render_events.h>
#include <tesseract_qt/common/events/scene_graph_events.h>
#include <tesseract_qt/common/events/allowed_collision_matrix_events.h>

#include <tesseract_qt/common/link_visibility.h>

#include <QApplication>

#include <tesseract_scene_graph/scene_state.h>

#include <boost/uuid/uuid_io.hpp>

const std::string USER_VISIBILITY = "user_visibility";

namespace tesseract_rviz
{
struct ROSSceneGraphRenderManager::Implementation
{
  std::unordered_map<tesseract_gui::ComponentInfo, tesseract_gui::EntityManager::Ptr> entity_managers;

  Ogre::SceneManager* scene_manager;
  Ogre::SceneNode* scene_node;

  void clear()
  {
    for (auto& entity_manager : entity_managers)
      clear(*entity_manager.second);

    entity_managers.clear();
  }

  void clear(const tesseract_gui::ComponentInfo& ci)
  {
    auto it = entity_managers.find(ci);
    if (it != entity_managers.end())
    {
      clear(*it->second);
      entity_managers.erase(it);
    }
  }

  void clear(tesseract_gui::EntityContainer& container)
  {
    // Destroy Resources
    for (const auto& ns : container.getUntrackedEntities())
    {
      if (ns.first == container.RESOURCE_NS)
      {
        for (const auto& entity : ns.second)
          scene_manager->destroyEntity(entity.unique_name);
      }
    }

    // Destroy Scene Nodes
    for (const auto& ns : container.getUntrackedEntities())
    {
      if (ns.first != container.RESOURCE_NS)
      {
        for (const auto& entity : ns.second)
          scene_manager->destroySceneNode(entity.unique_name);
      }
    }

    for (const auto& ns : container.getTrackedEntities())
    {
      if (ns.first != container.RESOURCE_NS)
      {
        for (const auto& entity : ns.second)
          scene_manager->destroySceneNode(entity.second.unique_name);
      }
    }

    container.clear();
  }

  void clear(tesseract_gui::EntityManager& entity_manager)
  {
    for (auto& entity_container : entity_manager.getEntityContainers())
    {
      clear(*entity_container.second);
      entity_manager.removeEntityContainer(entity_container.first);
    }
    entity_manager.clear();
  }
};

ROSSceneGraphRenderManager::ROSSceneGraphRenderManager(tesseract_gui::ComponentInfo component_info,
                                                       Ogre::SceneManager* scene_manager,
                                                       Ogre::SceneNode* scene_node)
  : tesseract_gui::SceneGraphRenderManager(component_info), data_(std::make_unique<Implementation>())
{
  data_->scene_manager = scene_manager;
  data_->scene_node = scene_node;

  addOgreResourceLocation();

  qApp->installEventFilter(this);
}

ROSSceneGraphRenderManager::~ROSSceneGraphRenderManager() { data_->clear(); }

void ROSSceneGraphRenderManager::render()
{
  if (events_.empty())
    return;

  auto getEntityManager =
      [this](const tesseract_gui::ComponentInfo& component_info) -> tesseract_gui::EntityManager::Ptr {
    auto it = data_->entity_managers.find(component_info);
    if (it != data_->entity_managers.end())
      return it->second;

    auto entity_manager = std::make_shared<tesseract_gui::EntityManager>();
    data_->entity_managers[component_info] = entity_manager;
    return entity_manager;
  };

  for (const auto& event : events_)
  {
    if (event->type() == tesseract_gui::events::SceneGraphClear::kType)
    {
      auto& e = static_cast<tesseract_gui::events::SceneGraphClear&>(*event);
      data_->clear(e.getComponentInfo());
    }
    else if (event->type() == tesseract_gui::events::SceneGraphSet::kType)
    {
      auto& e = static_cast<tesseract_gui::events::SceneGraphSet&>(*event);
      data_->clear(e.getComponentInfo());
      tesseract_gui::EntityManager::Ptr entity_manager = getEntityManager(e.getComponentInfo());
      loadSceneGraph(*data_->scene_manager, *data_->scene_node, *entity_manager, *e.getSceneGraph(), "");
    }
    else if (event->type() == tesseract_gui::events::SceneGraphAddLink::kType)
    {
      auto& e = static_cast<tesseract_gui::events::SceneGraphAddLink&>(*event);
      tesseract_gui::EntityManager::Ptr entity_manager = getEntityManager(e.getComponentInfo());
      auto entity_container = entity_manager->getEntityContainer(e.getLink()->getName());
      Ogre::SceneNode* sn = loadLink(*data_->scene_manager, *entity_container, *e.getLink());
      data_->scene_node->addChild(sn);
    }
    else if (event->type() == tesseract_gui::events::SceneGraphRemoveLink::kType)
    {
      auto& e = static_cast<tesseract_gui::events::SceneGraphRemoveLink&>(*event);
      tesseract_gui::EntityManager::Ptr entity_manager = getEntityManager(e.getComponentInfo());
      if (entity_manager->hasEntityContainer(e.getLinkName()))
      {
        auto entity_container = entity_manager->getEntityContainer(e.getLinkName());
        data_->clear(*entity_container);
        entity_container->clear();
        entity_manager->removeEntityContainer(e.getLinkName());
      }
    }
    else if (event->type() == tesseract_gui::events::SceneGraphModifyLinkVisibility::kType)
    {
      auto& e = static_cast<tesseract_gui::events::SceneGraphModifyLinkVisibility&>(*event);
      tesseract_gui::EntityManager::Ptr entity_manager = getEntityManager(e.getComponentInfo());
      for (const auto& link_name : e.getLinkNames())
      {
        if (entity_manager->hasEntityContainer(link_name))
        {
          tesseract_gui::EntityContainer::Ptr entity_container = entity_manager->getEntityContainer(link_name);

          bool link_visible{ true };
          // Link Property
          if (entity_container->hasTrackedEntity(tesseract_gui::EntityContainer::VISUAL_NS, link_name))
          {
            auto entity = entity_container->getTrackedEntity(tesseract_gui::EntityContainer::VISUAL_NS, link_name);
            Ogre::SceneNode* sn = data_->scene_manager->getSceneNode(entity.unique_name);

            if (e.getVisibilityFlags() & tesseract_gui::LinkVisibilityFlags::LINK ||
                e.getVisibilityFlags() & tesseract_gui::LinkVisibilityFlags::ALL)
            {
              sn->getUserObjectBindings().setUserAny(USER_VISIBILITY, Ogre::Any(e.visible()));
            }

            link_visible = Ogre::any_cast<bool>(sn->getUserObjectBindings().getUserAny(USER_VISIBILITY));
            sn->setVisible(link_visible, true);
          }

          // Link Visual Property
          std::string visual_key = link_name + "::Visuals";
          if (entity_container->hasTrackedEntity(tesseract_gui::EntityContainer::VISUAL_NS, visual_key))
          {
            auto entity = entity_container->getTrackedEntity(tesseract_gui::EntityContainer::VISUAL_NS, visual_key);
            Ogre::SceneNode* sn = data_->scene_manager->getSceneNode(entity.unique_name);

            if (e.getVisibilityFlags() & tesseract_gui::LinkVisibilityFlags::VISUAL ||
                e.getVisibilityFlags() & tesseract_gui::LinkVisibilityFlags::ALL)
            {
              sn->getUserObjectBindings().setUserAny(USER_VISIBILITY, Ogre::Any(e.visible()));
            }

            bool visible = Ogre::any_cast<bool>(sn->getUserObjectBindings().getUserAny(USER_VISIBILITY));
            sn->setVisible(link_visible & visible, true);
          }

          // Link Collision Property
          visual_key = link_name + "::Collisions";
          if (entity_container->hasTrackedEntity(tesseract_gui::EntityContainer::VISUAL_NS, visual_key))
          {
            auto entity = entity_container->getTrackedEntity(tesseract_gui::EntityContainer::VISUAL_NS, visual_key);
            Ogre::SceneNode* sn = data_->scene_manager->getSceneNode(entity.unique_name);

            if (e.getVisibilityFlags() & tesseract_gui::LinkVisibilityFlags::COLLISION ||
                e.getVisibilityFlags() & tesseract_gui::LinkVisibilityFlags::ALL)
            {
              sn->getUserObjectBindings().setUserAny(USER_VISIBILITY, Ogre::Any(e.visible()));
            }

            bool visible = Ogre::any_cast<bool>(sn->getUserObjectBindings().getUserAny(USER_VISIBILITY));
            sn->setVisible(link_visible & visible, true);
          }

          // Link WireBox Property
          visual_key = link_name + "::WireBox";
          if (entity_container->hasTrackedEntity(tesseract_gui::EntityContainer::VISUAL_NS, visual_key))
          {
            auto entity = entity_container->getTrackedEntity(tesseract_gui::EntityContainer::VISUAL_NS, visual_key);
            Ogre::SceneNode* sn = data_->scene_manager->getSceneNode(entity.unique_name);

            if (e.getVisibilityFlags() & tesseract_gui::LinkVisibilityFlags::WIREBOX ||
                e.getVisibilityFlags() & tesseract_gui::LinkVisibilityFlags::ALL)
            {
              sn->getUserObjectBindings().setUserAny(USER_VISIBILITY, Ogre::Any(e.visible()));
            }

            bool visible = Ogre::any_cast<bool>(sn->getUserObjectBindings().getUserAny(USER_VISIBILITY));
            sn->setVisible(link_visible & visible, true);
          }

          // Link Axis Property
          visual_key = link_name + "::Axis";
          if (entity_container->hasTrackedEntity(tesseract_gui::EntityContainer::VISUAL_NS, visual_key))
          {
            auto entity = entity_container->getTrackedEntity(tesseract_gui::EntityContainer::VISUAL_NS, visual_key);
            Ogre::SceneNode* sn = data_->scene_manager->getSceneNode(entity.unique_name);

            if (e.getVisibilityFlags() & tesseract_gui::LinkVisibilityFlags::AXIS ||
                e.getVisibilityFlags() & tesseract_gui::LinkVisibilityFlags::ALL)
            {
              sn->getUserObjectBindings().setUserAny(USER_VISIBILITY, Ogre::Any(e.visible()));
            }

            bool visible = Ogre::any_cast<bool>(sn->getUserObjectBindings().getUserAny(USER_VISIBILITY));
            sn->setVisible(link_visible & visible, true);
          }
        }
      }
    }
    else if (event->type() == tesseract_gui::events::SceneGraphModifyLinkVisibilityALL::kType)
    {
      auto& e = static_cast<tesseract_gui::events::SceneGraphModifyLinkVisibilityALL&>(*event);
      tesseract_gui::EntityManager::Ptr entity_manager = getEntityManager(e.getComponentInfo());
      for (const auto& entity_container : entity_manager->getEntityContainers())
      {
        for (const auto& ns : entity_container.second->getTrackedEntities(tesseract_gui::EntityContainer::VISUAL_NS))
        {
          std::vector<std::string> sub_ns = tesseract_gui::getNamespaces(ns.first);
          if (sub_ns.size() == 2)
          {
            if (sub_ns[1] == "Visuals" || sub_ns[1] == "Collisions")
            {
              auto link_entity =
                  entity_container.second->getTrackedEntity(tesseract_gui::EntityContainer::VISUAL_NS, sub_ns[0]);
              Ogre::SceneNode* link_visual_node = data_->scene_manager->getSceneNode(link_entity.unique_name);

              if (e.getVisibilityFlags() & tesseract_gui::LinkVisibilityFlags::LINK)
              {
                link_visual_node->getUserObjectBindings().setUserAny(USER_VISIBILITY, Ogre::Any(e.visible()));
                bool link_visible =
                    Ogre::any_cast<bool>(link_visual_node->getUserObjectBindings().getUserAny(USER_VISIBILITY));

                Ogre::SceneNode* sn = data_->scene_manager->getSceneNode(ns.second.unique_name);
                bool visible = Ogre::any_cast<bool>(sn->getUserObjectBindings().getUserAny(USER_VISIBILITY));
                sn->setVisible(link_visible & visible, true);
              }

              if (sub_ns[1] == "Visuals" && e.getVisibilityFlags() & tesseract_gui::LinkVisibilityFlags::VISUAL)
              {
                if (e.visible())
                  link_visual_node->getUserObjectBindings().setUserAny(USER_VISIBILITY, Ogre::Any(e.visible()));

                bool link_visible =
                    Ogre::any_cast<bool>(link_visual_node->getUserObjectBindings().getUserAny(USER_VISIBILITY));
                Ogre::SceneNode* sn = data_->scene_manager->getSceneNode(ns.second.unique_name);
                sn->getUserObjectBindings().setUserAny(USER_VISIBILITY, Ogre::Any(e.visible()));
                sn->setVisible(link_visible & e.visible(), true);
              }

              if (sub_ns[1] == "Collisions" && e.getVisibilityFlags() & tesseract_gui::LinkVisibilityFlags::COLLISION)
              {
                if (e.visible())
                  link_visual_node->getUserObjectBindings().setUserAny(USER_VISIBILITY, Ogre::Any(e.visible()));

                bool link_visible =
                    Ogre::any_cast<bool>(link_visual_node->getUserObjectBindings().getUserAny(USER_VISIBILITY));
                Ogre::SceneNode* sn = data_->scene_manager->getSceneNode(ns.second.unique_name);
                sn->getUserObjectBindings().setUserAny(USER_VISIBILITY, Ogre::Any(e.visible()));
                sn->setVisible(link_visible & e.visible(), true);
              }
            }
            else if (sub_ns[1] == "Axis" && e.getVisibilityFlags() & tesseract_gui::LinkVisibilityFlags::AXIS)
            {
              Ogre::SceneNode* sn = data_->scene_manager->getSceneNode(ns.second.unique_name);
              sn->getUserObjectBindings().setUserAny(USER_VISIBILITY, Ogre::Any(e.visible()));
              sn->setVisible(e.visible(), true);
            }
            else if (sub_ns[1] == "WireBox" && e.getVisibilityFlags() & tesseract_gui::LinkVisibilityFlags::WIREBOX)
            {
              Ogre::SceneNode* sn = data_->scene_manager->getSceneNode(ns.second.unique_name);
              sn->getUserObjectBindings().setUserAny(USER_VISIBILITY, Ogre::Any(e.visible()));
              sn->setVisible(e.visible(), true);
            }
          }
        }
      }
    }
    else if (event->type() == tesseract_gui::events::SceneStateChanged::kType)
    {
      auto& e = static_cast<tesseract_gui::events::SceneStateChanged&>(*event);
      tesseract_gui::EntityManager::Ptr entity_manager = getEntityManager(e.getComponentInfo());
      for (const auto& pair : e.getState().link_transforms)
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
    }
  }

  events_.clear();
}

}  // namespace tesseract_rviz
