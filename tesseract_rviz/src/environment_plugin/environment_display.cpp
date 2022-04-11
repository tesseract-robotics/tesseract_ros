#include <tesseract_rviz/environment_plugin/environment_display.h>
#include <tesseract_rviz/environment_plugin/environment_panel.h>
#include <tesseract_rviz/environment_plugin/conversions.h>
#include <tesseract_rviz/conversions.h>

#include <tesseract_widgets/environment/environment_widget.h>
#include <tesseract_widgets/common/entity_container.h>
#include <tesseract_widgets/common/entity_manager.h>
#include <tesseract_monitoring/environment_monitor.h>
#include <tesseract_msgs/EnvironmentState.h>
#include <sensor_msgs/JointState.h>

#include <rviz/properties/bool_property.h>
#include <rviz/properties/color_property.h>
#include <rviz/properties/float_property.h>
#include <rviz/properties/property.h>
#include <rviz/properties/ros_topic_property.h>
#include <rviz/properties/string_property.h>
#include <rviz/properties/enum_property.h>
#include <rviz/panel_dock_widget.h>

namespace tesseract_rviz
{
struct EnvironmentDisplayPrivate
{
  EnvironmentDisplayPrivate();

  ros::NodeHandle nh;
  tesseract_monitoring::ROSEnvironmentMonitor::Ptr monitor;
  tesseract_gui::EnvironmentWidget* widget;

  rviz::EnumProperty* display_mode_property;
  rviz::StringProperty* urdf_description_string_property;
  rviz::RosTopicProperty* environment_topic_property;
  rviz::RosTopicProperty* joint_state_topic_property;
  rviz::FloatProperty* scene_alpha_property;
  rviz::BoolProperty* scene_visual_visible;
  rviz::BoolProperty* scene_collision_visible;
  rviz::BoolProperty* scene_links_visible;

  tesseract_gui::EntityManager::Ptr entity_manager;

  /** @brief The current revision of the visualization environment */
  int render_revision{ 0 };
  bool render_dirty;

  /** @brief Update visualization current state from environment message */
  bool render_state_dirty;
  std::chrono::system_clock::time_point render_state_timestamp{ std::chrono::system_clock::now() };

  /** @brief Used for delayed initialization */
  bool render_reset;

  std::unordered_map<std::string, bool> link_visible_changes;
  std::unordered_map<std::string, bool> link_collision_visible_changes;
  std::unordered_map<std::string, bool> link_visual_visible_changes;
  std::vector<std::string> link_selection_changes;
  std::vector<std::string> render_link_names;

  /** @brief Keeps track of how many EnvironmentWidgets have been created for the default namespace */
  static int environment_display_counter;

  /** @brief Keeps track of which EnvironmentWidget this is */
  int environment_display_id;

  std::string environment_display_ns;
};

EnvironmentDisplayPrivate::EnvironmentDisplayPrivate()
  : entity_manager(std::make_shared<tesseract_gui::EntityManager>())
{
  environment_display_counter++;
  environment_display_id = environment_display_counter;
  environment_display_ns = "env_display_" + std::to_string(environment_display_id);
}

int EnvironmentDisplayPrivate::environment_display_counter = -1;

EnvironmentDisplay::EnvironmentDisplay() : data_(std::make_unique<EnvironmentDisplayPrivate>())
{
  data_->display_mode_property = new rviz::EnumProperty("Display Mode",
                                                        "URDF",
                                                        "Leverage URDF or connect to monitor namespace",
                                                        this,
                                                        SLOT(onDisplayModeChanged()),
                                                        this);

  data_->display_mode_property->addOptionStd("URDF", 0);
  data_->display_mode_property->addOptionStd("Monitor", 1);

  data_->urdf_description_string_property = new rviz::StringProperty("URDF Parameter",
                                                                     "robot_description",
                                                                     "The URDF parameter to use for creating the "
                                                                     "environment within RViz",
                                                                     this,
                                                                     SLOT(onURDFDescriptionChanged()),
                                                                     this);

  data_->environment_topic_property =
      new rviz::RosTopicProperty("Monitor Topic",
                                 "/tesseract_environment",
                                 ros::message_traits::datatype<tesseract_msgs::EnvironmentState>(),
                                 "This will monitor this topic for environment changes.",
                                 this,
                                 SLOT(onEnvironmentTopicChanged()),
                                 this);

  data_->joint_state_topic_property =
      new rviz::RosTopicProperty("Joint State Topic",
                                 "/joint_states",
                                 ros::message_traits::datatype<sensor_msgs::JointState>(),
                                 "This will monitor this topic for joint state changes.",
                                 this,
                                 SLOT(onJointStateTopicChanged()),
                                 this);
  data_->scene_alpha_property = new rviz::FloatProperty(
      "Alpha", 1.0f, "Specifies the alpha for the links with geometry", this, SLOT(onSceneAlphaChanged()), this);
  data_->scene_alpha_property->setMin(0.0);
  data_->scene_alpha_property->setMax(1.0);

  data_->scene_visual_visible = new rviz::BoolProperty("Show Visual",
                                                       true,
                                                       "Whether to display the visual representation of the "
                                                       "environment.",
                                                       this,
                                                       SLOT(onSceneVisualVisibleChanged()),
                                                       this);

  data_->scene_collision_visible = new rviz::BoolProperty("Show Collision",
                                                          false,
                                                          "Whether to display the collision representation of the "
                                                          "environment.",
                                                          this,
                                                          SLOT(onSceneCollisionVisibleChanged()),
                                                          this);

  data_->scene_links_visible = new rviz::BoolProperty(
      "Show All Links", true, "Toggle all links visibility on or off.", this, SLOT(onSceneLinkVisibleChanged()), this);
}

EnvironmentDisplay::~EnvironmentDisplay()
{
  auto* panel = getAssociatedWidgetPanel();
  panel->close();
}

void EnvironmentDisplay::onInitialize()
{
  Display::onInitialize();
  data_->widget = new tesseract_gui::EnvironmentWidget();
  setAssociatedWidget(data_->widget);

  connect(data_->widget,
          SIGNAL(environmentSet(tesseract_environment::Environment)),
          this,
          SLOT(onEnvironmentSet(tesseract_environment::Environment)));
  connect(data_->widget,
          SIGNAL(environmentChanged(tesseract_environment::Environment)),
          this,
          SLOT(onEnvironmentChanged(tesseract_environment::Environment)));
  connect(data_->widget,
          SIGNAL(environmentCurrentStateChanged(tesseract_environment::Environment)),
          this,
          SLOT(onEnvironmentCurrentStateChanged(tesseract_environment::Environment)));
  connect(data_->widget,
          SIGNAL(linkVisibleChanged(std::string, bool)),
          this,
          SLOT(onLinkVisibleChanged(std::string, bool)));
  connect(data_->widget,
          SIGNAL(linkVisualVisibleChanged(std::string, bool)),
          this,
          SLOT(onLinkVisualVisibleChanged(std::string, bool)));
  connect(data_->widget,
          SIGNAL(linkCollisionVisibleChanged(std::string, bool)),
          this,
          SLOT(onLinkCollisionVisibleChanged(std::string, bool)));
  connect(data_->widget,
          SIGNAL(selectedLinksChanged(std::vector<std::string>)),
          this,
          SLOT(onSelectedLinksChanged(std::vector<std::string>)));

  onDisplayModeChanged();
}

void EnvironmentDisplay::reset()
{
  //  visualization_->clear();
  Display::reset();

  //  environment_monitor_->onReset();
  //  state_monitor_->onReset();
}

void EnvironmentDisplay::onEnable()
{
  Display::onEnable();

  //  environment_monitor_->onEnable();
  //  state_monitor_->onEnable();
}

void EnvironmentDisplay::onDisable()
{
  //  environment_monitor_->onDisable();
  //  state_monitor_->onDisable();

  Display::onDisable();
}

void EnvironmentDisplay::update(float wall_dt, float ros_dt)
{
  Display::update(wall_dt, ros_dt);

  onUpdate();
  //  environment_monitor_->onUpdate();
  //  state_monitor_->onUpdate();
}

void EnvironmentDisplay::load(const rviz::Config& config)
{
  rviz::Display::load(config);

  int mode{ 0 };
  if (config.mapGetInt("EnvMode", &mode))
    data_->display_mode_property->setValue(mode);

  QString urdf_description;
  if (config.mapGetString("EnvURDFDescription", &urdf_description))
    data_->urdf_description_string_property->setString(urdf_description);

  QString topic;
  if (config.mapGetString("EnvTopic", &topic))
    data_->environment_topic_property->setString(topic);

  if (config.mapGetString("EnvJointStateTopic", &topic))
    data_->joint_state_topic_property->setString(topic);
}

void EnvironmentDisplay::save(rviz::Config config) const
{
  config.mapSetValue("EnvMode", data_->display_mode_property->getOptionInt());
  config.mapSetValue("EnvURDFDescription", data_->urdf_description_string_property->getString());
  config.mapSetValue("EnvTopic", data_->environment_topic_property->getString());
  config.mapSetValue("EnvJointStateTopic", data_->joint_state_topic_property->getString());
  rviz::Display::save(config);
}

void EnvironmentDisplay::clear()
{
  auto containers = data_->entity_manager->getEntityContainers();
  for (auto& container : containers)
  {
    clearContainer(*container.second);
    container.second->clear();
    data_->entity_manager->removeEntityContainer(container.first);
  }
}

void EnvironmentDisplay::clearContainer(const tesseract_gui::EntityContainer& container)
{
  // Destroy Resources
  for (const auto& ns : container.getUntrackedEntities())
  {
    if (ns.first == container.RESOURCE_NS)
    {
      for (const auto& entity : ns.second)
        scene_manager_->destroyEntity(entity.unique_name);
    }
  }

  // Destroy Scene Nodes
  for (const auto& ns : container.getUntrackedEntities())
  {
    if (ns.first != container.RESOURCE_NS)
    {
      for (const auto& entity : ns.second)
        scene_manager_->destroySceneNode(entity.unique_name);
    }
  }

  for (const auto& ns : container.getTrackedEntities())
  {
    if (ns.first != container.RESOURCE_NS)
    {
      for (const auto& entity : ns.second)
        scene_manager_->destroySceneNode(entity.second.unique_name);
    }
  }
}

void EnvironmentDisplay::onDisplayModeChanged()
{
  if (data_->display_mode_property->getOptionInt() == 0)
  {
    data_->environment_topic_property->setHidden(true);
    data_->urdf_description_string_property->setHidden(false);
    onURDFDescriptionChanged();
  }
  else if (data_->display_mode_property->getOptionInt() == 1)
  {
    data_->urdf_description_string_property->setHidden(true);
    data_->environment_topic_property->setHidden(false);
    onEnvironmentTopicChanged();
  }
  onJointStateTopicChanged();
}

void EnvironmentDisplay::onURDFDescriptionChanged()
{
  std::string urdf_xml_string, srdf_xml_string;
  data_->nh.getParam(data_->urdf_description_string_property->getStdString(), urdf_xml_string);
  data_->nh.getParam(data_->urdf_description_string_property->getStdString() + "_semantic", srdf_xml_string);

  auto env = std::make_shared<tesseract_environment::Environment>();
  auto locator = std::make_shared<tesseract_rosutils::ROSResourceLocator>();
  if (env->init(urdf_xml_string, srdf_xml_string, locator))
  {
    if (data_->monitor != nullptr)
      data_->monitor->shutdown();

    data_->monitor = std::make_unique<tesseract_monitoring::ROSEnvironmentMonitor>(env, data_->environment_display_ns);
    if (data_->monitor != nullptr)
    {
      data_->widget->setEnvironment(env);
      onJointStateTopicChanged();
    }
  }
  else
  {
    setStatus(rviz::StatusProperty::Error, "Tesseract", "URDF file failed to parse");
  }
}

void EnvironmentDisplay::onEnvironmentTopicChanged()
{
  if (data_->monitor != nullptr)
    data_->monitor->shutdown();

  auto env = std::make_shared<tesseract_environment::Environment>();
  data_->monitor = std::make_unique<tesseract_monitoring::ROSEnvironmentMonitor>(env, data_->environment_display_ns);

  if (data_->monitor != nullptr)
  {
    data_->widget->setEnvironment(env);
    std::string ns = getEnvNamespaceFromTopic(data_->environment_topic_property->getStdString());
    if (!ns.empty())
      data_->monitor->startMonitoringEnvironment(ns);
    else
      setStatus(rviz::StatusProperty::Error, "Tesseract", "Invalid environment monitor topic!");

    onJointStateTopicChanged();
  }
}

void EnvironmentDisplay::onJointStateTopicChanged()
{
  if (data_->monitor != nullptr)
    data_->monitor->startStateMonitor(data_->joint_state_topic_property->getStdString());
}

void EnvironmentDisplay::onEnvironmentSet(const tesseract_environment::Environment& /*env*/)
{
  data_->render_revision = 0;
  data_->render_dirty = true;
  data_->render_reset = true;
  data_->render_state_dirty = true;
}

void EnvironmentDisplay::onEnvironmentChanged(const tesseract_environment::Environment& /*env*/)
{
  data_->render_dirty = true;
  data_->render_state_dirty = true;
}

void EnvironmentDisplay::onEnvironmentCurrentStateChanged(const tesseract_environment::Environment& /*env*/)
{
  data_->render_state_dirty = true;
}

void EnvironmentDisplay::onLinkVisibleChanged(const std::string& link_name, bool visible)
{
  data_->link_visible_changes[link_name] = visible;
  data_->render_dirty = true;
}

void EnvironmentDisplay::onLinkCollisionVisibleChanged(const std::string& link_name, bool visible)
{
  data_->link_collision_visible_changes[link_name] = visible;
  data_->render_dirty = true;
}

void EnvironmentDisplay::onLinkVisualVisibleChanged(const std::string& link_name, bool visible)
{
  data_->link_visual_visible_changes[link_name] = visible;
  data_->render_dirty = true;
}

void EnvironmentDisplay::onSelectedLinksChanged(const std::vector<std::string>& selected_links)
{
  data_->link_selection_changes = selected_links;
  data_->render_dirty = true;
}

void EnvironmentDisplay::onUpdate()
{
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
      auto lock = data_->widget->environment().lockRead();
      auto revision = data_->widget->environment().getRevision();
      auto state_timestamp = data_->widget->environment().getCurrentStateTimestamp();
      if (data_->render_dirty || revision > data_->render_revision)
      {
        if (revision > data_->render_revision)
        {
          tesseract_environment::Commands commands = data_->widget->environment().getCommandHistory();

          bool links_removed{ false };
          for (std::size_t i = data_->render_revision; i < commands.size(); ++i)
          {
            const tesseract_environment::Command::ConstPtr& command = commands.at(i);
            switch (command->getType())
            {
              case tesseract_environment::CommandType::ADD_SCENE_GRAPH:
              {
                auto cmd = std::static_pointer_cast<const tesseract_environment::AddSceneGraphCommand>(command);
                auto link_names = loadSceneGraph(
                    *scene_manager_, *scene_node_, *data_->entity_manager, *cmd->getSceneGraph(), cmd->getPrefix());
                data_->render_link_names.insert(data_->render_link_names.end(), link_names.begin(), link_names.end());
                break;
              }
              case tesseract_environment::CommandType::ADD_LINK:
              {
                auto cmd = std::static_pointer_cast<const tesseract_environment::AddLinkCommand>(command);
                auto entity_container = data_->entity_manager->getEntityContainer(cmd->getLink()->getName());
                Ogre::SceneNode* sn = loadLink(*scene_manager_, *entity_container, *cmd->getLink());
                scene_node_->addChild(sn);
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
                    Ogre::SceneNode* sn = scene_manager_->getSceneNode(entity.unique_name);
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
            std::vector<std::string> link_names = data_->widget->environment().getLinkNames();
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
        tesseract_scene_graph::SceneState state = data_->widget->environment().getState();
        for (const auto& pair : state.link_transforms)
        {
          if (data_->entity_manager->hasEntityContainer(pair.first))
          {
            auto container = data_->entity_manager->getEntityContainer(pair.first);
            Ogre::Vector3 position;
            Ogre::Quaternion orientation;
            toOgre(position, orientation, pair.second);

            auto entity = container->getTrackedEntity(tesseract_gui::EntityContainer::VISUAL_NS, pair.first);
            Ogre::SceneNode* sn = scene_manager_->getSceneNode(entity.unique_name);
            sn->setPosition(position);
            sn->setOrientation(orientation);
          }
        }
        data_->render_state_timestamp = state_timestamp;
      }
    }

    if (data_->render_dirty)
    {
      for (const auto& l : data_->link_visible_changes)
      {
        if (data_->entity_manager->hasEntityContainer(l.first))
        {
          auto entity_container = data_->entity_manager->getEntityContainer(l.first);
          if (entity_container->hasTrackedEntity(tesseract_gui::EntityContainer::VISUAL_NS, l.first))
          {
            auto entity = entity_container->getTrackedEntity(tesseract_gui::EntityContainer::VISUAL_NS, l.first);
            Ogre::SceneNode* sn = scene_manager_->getSceneNode(entity.unique_name);
            sn->setVisible(l.second, true);
          }
        }
      }
      data_->link_visible_changes.clear();

      for (const auto& l : data_->link_visual_visible_changes)
      {
        if (data_->entity_manager->hasEntityContainer(l.first))
        {
          auto entity_container = data_->entity_manager->getEntityContainer(l.first);
          std::string visual_key = l.first + "::Visuals";
          if (entity_container->hasTrackedEntity(tesseract_gui::EntityContainer::VISUAL_NS, visual_key))
          {
            auto entity = entity_container->getTrackedEntity(tesseract_gui::EntityContainer::VISUAL_NS, visual_key);
            Ogre::SceneNode* sn = scene_manager_->getSceneNode(entity.unique_name);
            sn->setVisible(l.second, true);
          }
        }
      }
      data_->link_visual_visible_changes.clear();

      for (const auto& l : data_->link_collision_visible_changes)
      {
        if (data_->entity_manager->hasEntityContainer(l.first))
        {
          auto entity_container = data_->entity_manager->getEntityContainer(l.first);
          std::string visual_key = l.first + "::Collisions";
          if (entity_container->hasTrackedEntity(tesseract_gui::EntityContainer::VISUAL_NS, visual_key))
          {
            auto entity = entity_container->getTrackedEntity(tesseract_gui::EntityContainer::VISUAL_NS, visual_key);
            Ogre::SceneNode* sn = scene_manager_->getSceneNode(entity.unique_name);
            sn->setVisible(l.second, true);
          }
        }
      }
      data_->link_collision_visible_changes.clear();

      //      for (const auto& id : data_->highlighted_entities)
      //      {
      //        auto visual_node = scene->VisualById(id);
      //        if (visual_node != nullptr)
      //          visual_node->SetVisible(false);
      //      }
      //      highlighted_entities_.clear();

      //      for (const auto& l : data_->link_selection_changes)
      //      {
      //        std::string visual_key = l + "::WireBox";
      //        auto lc = data_->entity_container.getVisual(visual_key);
      //        auto visual_node = scene->VisualById(lc);
      //        if (visual_node != nullptr)
      //        {
      //          visual_node->SetVisible(true);
      //          highlighted_entities_.push_back(lc);
      //        }
      //      }
    }
    data_->render_dirty = false;
  }
}

void EnvironmentDisplay::onSceneAlphaChanged() {}

void EnvironmentDisplay::onSceneVisualVisibleChanged()
{
  std::vector<std::string> link_names = data_->widget->environment().getLinkNames();
  for (const auto& link_name : link_names)
    onLinkVisualVisibleChanged(link_name, data_->scene_visual_visible->getBool());
}

void EnvironmentDisplay::onSceneCollisionVisibleChanged()
{
  std::vector<std::string> link_names = data_->widget->environment().getLinkNames();
  for (const auto& link_name : link_names)
    onLinkCollisionVisibleChanged(link_name, data_->scene_collision_visible->getBool());
}

void EnvironmentDisplay::onSceneLinkVisibleChanged()
{
  std::vector<std::string> link_names = data_->widget->environment().getLinkNames();
  for (const auto& link_name : link_names)
    onLinkVisibleChanged(link_name, data_->scene_links_visible->getBool());
}

}  // namespace tesseract_rviz
