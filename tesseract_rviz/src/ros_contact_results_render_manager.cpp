#include <tesseract_rviz/ros_contact_results_render_manager.h>
#include <tesseract_rviz/conversions.h>
#include <tesseract_rviz/markers/arrow_marker.h>

#include <tesseract_qt/common/utils.h>
#include <tesseract_qt/common/entity_manager.h>
#include <tesseract_qt/common/entity_container.h>
#include <tesseract_qt/common/component_info.h>
#include <tesseract_qt/common/contact_results_types.h>
#include <tesseract_qt/common/tracked_object.h>
#include <tesseract_qt/common/events/render_events.h>
#include <tesseract_qt/common/events/contact_results_events.h>

#include <QApplication>

#include <boost/uuid/uuid_io.hpp>

static const std::string USER_VISIBILITY = "user_visibility";

namespace tesseract_rviz
{
struct ROSContactResultsRenderManager::Implementation
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

ROSContactResultsRenderManager::ROSContactResultsRenderManager(tesseract_gui::ComponentInfo component_info,
                                                               Ogre::SceneManager* scene_manager,
                                                               Ogre::SceneNode* scene_node)
  : tesseract_gui::ContactResultsRenderManager(component_info), data_(std::make_unique<Implementation>())
{
  data_->scene_manager = scene_manager;
  data_->scene_node = scene_node;

  addOgreResourceLocation();

  qApp->installEventFilter(this);
}

ROSContactResultsRenderManager::~ROSContactResultsRenderManager() { data_->clear(); }

void ROSContactResultsRenderManager::render()
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
    if (event->type() == tesseract_gui::events::ContactResultsClear::kType)
    {
      auto& e = static_cast<tesseract_gui::events::ContactResultsClear&>(*event);
      tesseract_gui::EntityManager::Ptr entity_manager = getEntityManager(e.getComponentInfo());
      data_->clear(*entity_manager);
    }
    else if (event->type() == tesseract_gui::events::ContactResultsRemove::kType)
    {
      auto& e = static_cast<tesseract_gui::events::ContactResultsRemove&>(*event);
      tesseract_gui::EntityManager::Ptr entity_manager = getEntityManager(e.getComponentInfo());
      const std::string parent_key = boost::uuids::to_string(e.getUUID());
      auto entity_container = entity_manager->getEntityContainer(parent_key);
      if (e.getChildUUID().is_nil())
      {
        data_->clear(*entity_container);
        entity_manager->removeEntityContainer(parent_key);
      }
      else
      {
        ROS_ERROR("ROSContactResultsRenderManager, removing child elements is currently not supported");
      }
    }
    else if (event->type() == tesseract_gui::events::ContactResultsSet::kType)
    {
      auto& e = static_cast<tesseract_gui::events::ContactResultsSet&>(*event);
      tesseract_gui::EntityManager::Ptr entity_manager = getEntityManager(e.getComponentInfo());
      data_->clear(*entity_manager);

      auto contacts = e.getContactResults();
      if (contacts.index() == 0)
      {
        const tesseract_gui::ContactResultVector& crv = std::get<tesseract_gui::ContactResultVector>(contacts);
        const std::string parent_key = boost::uuids::to_string(crv.getUUID());

        tesseract_gui::EntityContainer::Ptr entity_container = entity_manager->getEntityContainer(parent_key);
        auto entity = entity_container->addTrackedEntity(tesseract_gui::EntityContainer::VISUAL_NS, parent_key);
        auto* scene_node = data_->scene_manager->createSceneNode(entity.unique_name);
        scene_node->getUserObjectBindings().setUserAny(USER_VISIBILITY, Ogre::Any(false));
        data_->scene_node->addChild(scene_node);

        int cnt = 0;
        for (const auto& crt : crv())
        {
          const auto& cr = crt();
          Ogre::Vector3 p1(cr.nearest_points[0].x(), cr.nearest_points[0].y(), cr.nearest_points[0].z());
          Ogre::Vector3 p2(cr.nearest_points[1].x(), cr.nearest_points[1].y(), cr.nearest_points[1].z());
          Ogre::Vector3 direction = p2 - p1;
          direction.normalise();

          float head_length = 0.015;
          float shaft_diameter = 0.01;
          float head_diameter = 0.015;
          float shaft_length = cr.distance - head_length;
          std::array<float, 4> proportions = { shaft_length, shaft_diameter, head_length, head_diameter };

          auto arrow = std::make_unique<ArrowMarker>(
              "ContactResults", cnt++, p1, direction, proportions, data_->scene_manager, scene_node);
          arrow->setColor(Ogre::ColourValue(1, 0, 0));
          arrow->setVisible(false);
          arrow->getUserObjectBindings().setUserAny(USER_VISIBILITY, Ogre::Any(true));

          const std::string arrow_key_name = boost::uuids::to_string(crt.getUUID());
          entity_container->addTrackedUnmanagedObject(
              tesseract_gui::EntityContainer::VISUAL_NS, arrow_key_name, std::move(arrow));
        }
      }
      else
      {
        const tesseract_gui::ContactResultMap& crm = std::get<tesseract_gui::ContactResultMap>(contacts);

        int cnt = 0;
        for (const auto& pair : crm)
        {
          const std::string parent_key = boost::uuids::to_string(pair.second.getUUID());

          tesseract_gui::EntityContainer::Ptr entity_container = entity_manager->getEntityContainer(parent_key);
          auto entity = entity_container->addTrackedEntity(tesseract_gui::EntityContainer::VISUAL_NS, parent_key);
          auto* scene_node = data_->scene_manager->createSceneNode(entity.unique_name);
          scene_node->getUserObjectBindings().setUserAny(USER_VISIBILITY, Ogre::Any(false));
          data_->scene_node->addChild(scene_node);

          for (const auto& crt : pair.second())
          {
            const auto& cr = crt();
            Ogre::Vector3 p1(cr.nearest_points[0].x(), cr.nearest_points[0].y(), cr.nearest_points[0].z());
            Ogre::Vector3 p2(cr.nearest_points[1].x(), cr.nearest_points[1].y(), cr.nearest_points[1].z());
            Ogre::Vector3 direction = p2 - p1;
            direction.normalise();

            float head_length = 0.015;
            float shaft_diameter = 0.01;
            float head_diameter = 0.015;
            float shaft_length = cr.distance - head_length;
            std::array<float, 4> proportions = { shaft_length, shaft_diameter, head_length, head_diameter };

            auto arrow = std::make_unique<ArrowMarker>(
                "ContactResults", cnt++, p1, direction, proportions, data_->scene_manager, scene_node);
            arrow->setColor(Ogre::ColourValue(1, 0, 0));
            arrow->setVisible(false);
            arrow->getUserObjectBindings().setUserAny(USER_VISIBILITY, Ogre::Any(true));

            const std::string arrow_key_name = boost::uuids::to_string(crt.getUUID());
            entity_container->addTrackedUnmanagedObject(
                tesseract_gui::EntityContainer::VISUAL_NS, arrow_key_name, std::move(arrow));
          }
        }
      }
    }
    else if (event->type() == tesseract_gui::events::ContactResultsVisbility::kType)
    {
      auto& e = static_cast<tesseract_gui::events::ContactResultsVisbility&>(*event);
      tesseract_gui::EntityManager::Ptr entity_manager = getEntityManager(e.getComponentInfo());
      const std::string parent_key = boost::uuids::to_string(e.getUUID());
      auto entity_container = entity_manager->getEntityContainer(parent_key);
      if (entity_container->hasTrackedEntity(tesseract_gui::EntityContainer::VISUAL_NS, parent_key))
      {
        auto parent_entity = entity_container->getTrackedEntity(tesseract_gui::EntityContainer::VISUAL_NS, parent_key);
        auto* parent_scene_node = data_->scene_manager->getSceneNode(parent_entity.unique_name);
        auto parent_visibility =
            Ogre::any_cast<bool>(parent_scene_node->getUserObjectBindings().getUserAny(USER_VISIBILITY));
        if (e.getChildUUID().is_nil())
        {
          parent_visibility = e.getVisibility();
          parent_scene_node->getUserObjectBindings().setUserAny(USER_VISIBILITY, Ogre::Any(parent_visibility));

          auto objects = entity_container->getTrackedUnmanagedObjects(tesseract_gui::EntityContainer::VISUAL_NS);
          for (auto& obj : objects)
          {
            auto arrow = std::static_pointer_cast<ArrowMarker>(obj.second);
            auto user_visible = Ogre::any_cast<bool>(arrow->getUserObjectBindings().getUserAny(USER_VISIBILITY));
            arrow->setVisible(parent_visibility & user_visible);
          }
        }
        else
        {
          const std::string arrow_key_name = boost::uuids::to_string(e.getChildUUID());
          if (entity_container->hasTrackedUnmanagedObject(tesseract_gui::EntityContainer::VISUAL_NS, arrow_key_name))
          {
            tesseract_gui::UnmanagedObject obj =
                entity_container->getTrackedUnmanagedObject(tesseract_gui::EntityContainer::VISUAL_NS, arrow_key_name);
            auto arrow = std::static_pointer_cast<ArrowMarker>(obj);
            arrow->getUserObjectBindings().setUserAny(USER_VISIBILITY, Ogre::Any(e.getVisibility()));
            arrow->setVisible(parent_visibility & e.getVisibility());
          }
        }
      }
    }
    else if (event->type() == tesseract_gui::events::ContactResultsVisbilityAll::kType)
    {
      auto& e = static_cast<tesseract_gui::events::ContactResultsVisbilityAll&>(*event);
      const bool visibility = e.getVisibility();
      tesseract_gui::EntityManager::Ptr entity_manager = getEntityManager(e.getComponentInfo());
      for (auto& container : entity_manager->getEntityContainers())
      {
        auto parent_entity =
            container.second->getTrackedEntity(tesseract_gui::EntityContainer::VISUAL_NS, container.first);
        auto* parent_scene_node = data_->scene_manager->getSceneNode(parent_entity.unique_name);
        parent_scene_node->getUserObjectBindings().setUserAny(USER_VISIBILITY, Ogre::Any(visibility));
        for (auto& obj : container.second->getTrackedUnmanagedObjects(tesseract_gui::EntityContainer::VISUAL_NS))
        {
          auto arrow = std::static_pointer_cast<ArrowMarker>(obj.second);
          arrow->getUserObjectBindings().setUserAny(USER_VISIBILITY, Ogre::Any(visibility));
          arrow->setVisible(visibility);
        }
      }
    }
  }

  events_.clear();
}

}  // namespace tesseract_rviz
