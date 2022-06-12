#include <tesseract_rviz/ros_environment_widget.h>
#include <tesseract_rviz/conversions.h>
#include <tesseract_rviz/conversions.h>

#include <tesseract_rviz/markers/marker_base.h>
#include <tesseract_rviz/markers/arrow_marker.h>

#include <tesseract_qt/environment/environment_widget_config.h>
#include <tesseract_qt/common/entity_container.h>
#include <tesseract_qt/common/entity_manager.h>

#include <QLayout>

namespace tesseract_rviz
{
struct ROSEnvironmentWidgetPrivate
{
  ROSEnvironmentWidgetPrivate() : entity_manager(std::make_shared<tesseract_gui::EntityManager>()){};

  Ogre::SceneManager* scene_manager;
  Ogre::SceneNode* scene_node;

  tesseract_gui::EntityManager::Ptr entity_manager;

  /** @brief The current revision of the visualization environment */
  int render_revision{ 0 };
  bool render_dirty;

  /** @brief Update visualization current state from environment message */
  bool render_state_dirty;
  std::chrono::system_clock::time_point render_state_timestamp{ std::chrono::system_clock::now() };

  /** @brief Used for delayed initialization */
  bool render_reset;

  /** @brief The current list of links being rendered */
  std::vector<std::string> render_link_names;

  /** @brief The links with changed visibility properties */
  std::set<std::string> link_visibility_properties_changed;

  /** @brief The contact results to show */
  bool render_contact_results_dirty{ false };
  std::vector<std::unique_ptr<MarkerBase>> contact_results_markers;
  tesseract_collision::ContactResultVector contact_results_changed;
};

ROSEnvironmentWidget::ROSEnvironmentWidget(Ogre::SceneManager* scene_manager, Ogre::SceneNode* scene_node)
  : data_(std::make_unique<ROSEnvironmentWidgetPrivate>())
{
  layout()->setSizeConstraint(QLayout::SetNoConstraint);

  data_->scene_manager = scene_manager;
  data_->scene_node = scene_node;

  addOgreResourceLocation();

  connect(this,
          SIGNAL(environmentSet(std::shared_ptr<tesseract_environment::Environment>)),
          this,
          SLOT(onEnvironmentSet(std::shared_ptr<tesseract_environment::Environment>)));
  connect(this,
          SIGNAL(environmentChanged(tesseract_environment::Environment)),
          this,
          SLOT(onEnvironmentChanged(tesseract_environment::Environment)));
  connect(this,
          SIGNAL(environmentCurrentStateChanged(tesseract_environment::Environment)),
          this,
          SLOT(onEnvironmentCurrentStateChanged(tesseract_environment::Environment)));

  connect(this,
          SIGNAL(linkVisibilityChanged(std::vector<std::string>)),
          this,
          SLOT(onLinkVisibilityChanged(std::vector<std::string>)));

  connect(this,
          SIGNAL(showContactResults(tesseract_collision::ContactResultVector)),
          this,
          SLOT(onShowContactResults(tesseract_collision::ContactResultVector)));
}

ROSEnvironmentWidget::~ROSEnvironmentWidget() = default;

tesseract_gui::EnvironmentWidget* ROSEnvironmentWidget::clone() const
{
  return new ROSEnvironmentWidget(data_->scene_manager, data_->scene_node);
}

void ROSEnvironmentWidget::clear()
{
  auto containers = data_->entity_manager->getEntityContainers();
  for (auto& container : containers)
  {
    clearContainer(*container.second);
    container.second->clear();
    data_->entity_manager->removeEntityContainer(container.first);
  }
  data_->contact_results_markers.clear();
}

void ROSEnvironmentWidget::clearContainer(const tesseract_gui::EntityContainer& container)
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

void ROSEnvironmentWidget::onEnvironmentSet(const std::shared_ptr<tesseract_environment::Environment>& /*env*/)
{
  data_->render_revision = 0;
  data_->render_dirty = true;
  data_->render_reset = true;
  data_->render_state_dirty = true;
}

void ROSEnvironmentWidget::onEnvironmentChanged(const tesseract_environment::Environment& /*env*/)
{
  data_->render_dirty = true;
  data_->render_state_dirty = true;
}

void ROSEnvironmentWidget::onEnvironmentCurrentStateChanged(const tesseract_environment::Environment& /*env*/)
{
  data_->render_dirty = true;
  data_->render_state_dirty = true;
}

void ROSEnvironmentWidget::onLinkVisibilityChanged(const std::vector<std::string>& links)
{
  data_->link_visibility_properties_changed.insert(links.begin(), links.end());
  data_->render_dirty = true;
}

void ROSEnvironmentWidget::onShowContactResults(const tesseract_collision::ContactResultVector& contact_results)
{
  data_->contact_results_changed = contact_results;
  data_->render_contact_results_dirty = true;
  data_->render_dirty = true;
}

void ROSEnvironmentWidget::onRender(float dt)
{
  if (getEnvironment() == nullptr)
    return;

  if (data_->render_dirty)
  {
    if (data_->render_reset)  // Remove all
    {
      if (!data_->entity_manager->empty())
      {
        clear();
        data_->render_link_names.clear();
        data_->render_revision = 0;
        data_->render_state_timestamp = std::chrono::system_clock::now();
      }
      data_->render_reset = false;
    }

    {  // Check environment
      auto lock = environment().lockRead();
      auto revision = environment().getRevision();
      auto state_timestamp = environment().getCurrentStateTimestamp();
      if (data_->render_dirty || revision > data_->render_revision)
      {
        if (revision > data_->render_revision)
        {
          tesseract_environment::Commands commands = environment().getCommandHistory();

          bool links_removed{ false };
          for (std::size_t i = data_->render_revision; i < commands.size(); ++i)
          {
            const tesseract_environment::Command::ConstPtr& command = commands.at(i);
            switch (command->getType())
            {
              case tesseract_environment::CommandType::ADD_SCENE_GRAPH:
              {
                auto cmd = std::static_pointer_cast<const tesseract_environment::AddSceneGraphCommand>(command);
                auto link_names = loadSceneGraph(*data_->scene_manager,
                                                 *data_->scene_node,
                                                 *data_->entity_manager,
                                                 *cmd->getSceneGraph(),
                                                 cmd->getPrefix());
                data_->render_link_names.insert(data_->render_link_names.end(), link_names.begin(), link_names.end());
                break;
              }
              case tesseract_environment::CommandType::ADD_LINK:
              {
                auto cmd = std::static_pointer_cast<const tesseract_environment::AddLinkCommand>(command);
                auto entity_container = data_->entity_manager->getEntityContainer(cmd->getLink()->getName());
                Ogre::SceneNode* sn = loadLink(*data_->scene_manager, *entity_container, *cmd->getLink());
                data_->scene_node->addChild(sn);
                break;
              }
              case tesseract_environment::CommandType::CHANGE_LINK_VISIBILITY:
              {
                auto cmd = std::static_pointer_cast<const tesseract_environment::ChangeLinkVisibilityCommand>(command);
                if (data_->entity_manager->hasEntityContainer(cmd->getLinkName()))
                {
                  auto entity_container = data_->entity_manager->getEntityContainer(cmd->getLinkName());
                  if (entity_container->hasTrackedEntity(tesseract_gui::EntityContainer::VISUAL_NS, cmd->getLinkName()))
                  {
                    auto entity = entity_container->getTrackedEntity(tesseract_gui::EntityContainer::VISUAL_NS,
                                                                     cmd->getLinkName());
                    Ogre::SceneNode* sn = data_->scene_manager->getSceneNode(entity.unique_name);
                    sn->setVisible(cmd->getEnabled(), true);
                  }
                }
                break;
              }
              case tesseract_environment::CommandType::REMOVE_LINK:
              case tesseract_environment::CommandType::REMOVE_JOINT:
              {
                links_removed = true;
                break;
              }
              case tesseract_environment::CommandType::MOVE_LINK:
              case tesseract_environment::CommandType::MOVE_JOINT:
              case tesseract_environment::CommandType::REPLACE_JOINT:
              case tesseract_environment::CommandType::CHANGE_JOINT_ORIGIN:
              case tesseract_environment::CommandType::CHANGE_LINK_ORIGIN:
              {
                data_->render_state_dirty = true;
                break;
              }
              case tesseract_environment::CommandType::CHANGE_LINK_COLLISION_ENABLED:
              case tesseract_environment::CommandType::ADD_ALLOWED_COLLISION:
              case tesseract_environment::CommandType::REMOVE_ALLOWED_COLLISION:
              case tesseract_environment::CommandType::REMOVE_ALLOWED_COLLISION_LINK:
              case tesseract_environment::CommandType::CHANGE_JOINT_POSITION_LIMITS:
              case tesseract_environment::CommandType::CHANGE_JOINT_VELOCITY_LIMITS:
              case tesseract_environment::CommandType::CHANGE_JOINT_ACCELERATION_LIMITS:
              case tesseract_environment::CommandType::ADD_KINEMATICS_INFORMATION:
              case tesseract_environment::CommandType::CHANGE_COLLISION_MARGINS:
              case tesseract_environment::CommandType::ADD_CONTACT_MANAGERS_PLUGIN_INFO:
              case tesseract_environment::CommandType::SET_ACTIVE_CONTINUOUS_CONTACT_MANAGER:
              case tesseract_environment::CommandType::SET_ACTIVE_DISCRETE_CONTACT_MANAGER:
              {
                break;
              }
              // LCOV_EXCL_START
              default:
              {
                CONSOLE_BRIDGE_logError("RViz Tesseract Environment, Unhandled environment command");
              }
                // LCOV_EXCL_STOP
            }
          }

          if (links_removed)
          {
            std::vector<std::string> link_names = environment().getLinkNames();
            std::vector<std::string> diff;

            std::sort(link_names.begin(), link_names.end());
            std::sort(data_->render_link_names.begin(), data_->render_link_names.end());

            std::set_difference(data_->render_link_names.begin(),
                                data_->render_link_names.end(),
                                link_names.begin(),
                                link_names.end(),
                                std::inserter(diff, diff.begin()));

            for (const auto& removed_link : diff)
            {
              if (data_->entity_manager->hasEntityContainer(removed_link))
              {
                auto entity_container = data_->entity_manager->getEntityContainer(removed_link);
                clearContainer(*entity_container);
                entity_container->clear();
                data_->entity_manager->removeEntityContainer(removed_link);
              }
            }
          }
          data_->render_revision = revision;
        }
      }

      if (data_->render_state_dirty || state_timestamp > data_->render_state_timestamp)
      {
        tesseract_scene_graph::SceneState state = environment().getState();
        for (const auto& pair : state.link_transforms)
        {
          if (data_->entity_manager->hasEntityContainer(pair.first))
          {
            auto container = data_->entity_manager->getEntityContainer(pair.first);
            Ogre::Vector3 position;
            Ogre::Quaternion orientation;
            toOgre(position, orientation, pair.second);

            auto entity = container->getTrackedEntity(tesseract_gui::EntityContainer::VISUAL_NS, pair.first);
            Ogre::SceneNode* sn = data_->scene_manager->getSceneNode(entity.unique_name);
            sn->setPosition(position);
            sn->setOrientation(orientation);
          }
        }
        data_->render_state_dirty = false;
        data_->render_state_timestamp = state_timestamp;
      }
    }

    {  // Update Link Visibility
      auto link_visibility_properties = getLinkVisibilityProperties();
      for (const auto& l : data_->link_visibility_properties_changed)
      {
        if (data_->entity_manager->hasEntityContainer(l))
        {
          auto link_visibility_property = link_visibility_properties.at(l);
          auto entity_container = data_->entity_manager->getEntityContainer(l);

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
      data_->link_visibility_properties_changed.clear();
    }

    if (data_->render_contact_results_dirty)
    {
      // Update contact results
      auto entity_container = data_->entity_manager->getEntityContainer("ContactResults");
      if (!entity_container->empty())
      {
        clearContainer(*entity_container);
        entity_container->clear();
      }

      auto entity = entity_container->addTrackedEntity(tesseract_gui::EntityContainer::VISUAL_NS, "ContactResults");
      auto* scene_node = data_->scene_manager->createSceneNode(entity.unique_name);
      data_->scene_node->addChild(scene_node);

      data_->contact_results_markers.clear();
      int cnt = 0;
      for (const auto& cr : data_->contact_results_changed)
      {
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

        data_->contact_results_markers.push_back(std::move(arrow));
      }
      data_->contact_results_changed.clear();
      data_->render_contact_results_dirty = false;
    }
    data_->render_dirty = false;
  }
}

}  // namespace tesseract_rviz
