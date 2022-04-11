#include <tesseract_rviz/environment_plugin/environment_panel.h>
#include <tesseract_rviz/environment_plugin/rostopic_combo_box.h>
#include <tesseract_monitoring/environment_monitor.h>

#include <tesseract_widgets/environment/environment_widget.h>
#include <tesseract_widgets/common/entity_container.h>
#include <tesseract_widgets/common/entity_manager.h>
#include <tesseract_msgs/EnvironmentState.h>

#include <rviz/visualization_manager.h>

#include <QHBoxLayout>
#include <QVBoxLayout>
#include <QGridLayout>
#include <QLabel>
#include <QLineEdit>
#include <QComboBox>
#include <QUuid>

#include <OgreSceneManager.h>

namespace tesseract_rviz
{

struct EnvironmentPanelPrivate
{
  EnvironmentPanelPrivate()
    : entity_manager(std::make_shared<tesseract_gui::EntityManager>())
  {
    entity_container = entity_manager->getEntityContainer(container_name);
  }

  tesseract_gui::EnvironmentWidget* env_widget;
  QComboBox* display_mode;
  ROSTopicComboBox* env_topic_editor;
  ROSTopicComboBox* joint_state_topic_editor;
  QString env_topic;
  QString joint_state_topic;

  std::string container_name{QUuid::createUuid().toString().toStdString()};
  tesseract_gui::EntityManager::Ptr entity_manager;
  tesseract_gui::EntityContainer::Ptr entity_container;

  Ogre::SceneManager* scene_manager;
  Ogre::SceneNode* scene_node;

  /** @brief The current revision of the visualization environment */
  int render_revision{ 0 };
  bool render_dirty;

  /** @brief Update visualization current state from environment message */
  bool render_state_dirty;

  /** @brief Used for delayed initialization */
  bool render_reset;

  std::unordered_map<std::string, bool> link_visible_changes;
  std::unordered_map<std::string, bool> link_collision_visible_changes;
  std::unordered_map<std::string, bool> link_visual_visible_changes;
  std::vector<std::string> link_selection_changes;


  /** @brief Keeps track of how many EnvironmentPanels have been created for the default namespace */
  static int environment_widget_counter; // NOLINT

  /** @brief Keeps track of which EnvironmentWidget this is */
  int environment_panel_id;

  /** @brief The widgets environment monitor namespace */
  std::string panel_ns;

  void clear();
};

void EnvironmentPanelPrivate::clear()
{
  scene_manager->destroySceneNode(scene_node);
  for (const auto& entity : entity_container->getVisuals())
    scene_manager->destroyEntity(entity.second.unique_name);

  for (const auto& entity : entity_container->getSensors())
    scene_manager->destroyEntity(entity.second.unique_name);

  for (const auto& entity : entity_container->getUntracked())
    scene_manager->destroyEntity(entity.unique_name);

  entity_container->clear();
}

EnvironmentPanel::EnvironmentPanel(QWidget* parent)
  : rviz::Panel(parent)
  , data_(std::make_unique<EnvironmentPanelPrivate>())
{
  setName("Tesseract Environment");

  QGridLayout* properties_layout = new QGridLayout();
  QLabel* display_mode_label = new QLabel( "Display Mode:" );
  display_mode_label->setToolTip("Leverage URDF or connect to monitor namespace");
  data_->display_mode = new QComboBox();
  data_->display_mode->setEditable(false);
  data_->display_mode->addItem("URDF");
  data_->display_mode->addItem("Monitor");

  properties_layout->addWidget(display_mode_label, 0, 0);
  properties_layout->addWidget(data_->display_mode, 0, 1);

  data_->env_topic_editor = new ROSTopicComboBox();
  data_->env_topic_editor->setMessageType<tesseract_msgs::EnvironmentState>();
  properties_layout->addWidget(new QLabel( "Environment Topic:" ), 1, 0);
  properties_layout->addWidget(data_->env_topic_editor, 1, 1);

  data_->joint_state_topic_editor = new ROSTopicComboBox();
  data_->joint_state_topic_editor->setMessageType<sensor_msgs::JointState>();
  properties_layout->addWidget(new QLabel( "Joint State Topic:" ), 2, 0);
  properties_layout->addWidget(data_->joint_state_topic_editor, 2, 1);

  data_->env_widget = new tesseract_gui::EnvironmentWidget();

  QVBoxLayout* layout = new QVBoxLayout;
  layout->addLayout( properties_layout );
  layout->addWidget( data_->env_widget );
  setLayout( layout );

  connect(data_->display_mode, SIGNAL(currentIndexChanged(int)), this, SLOT(onDisplayModeChanged(int)));
  connect(data_->env_topic_editor, SIGNAL(currentTextChanged(QString)), this, SLOT(onEnvironmentTopicChanged(QString)));
  connect(data_->joint_state_topic_editor, SIGNAL(currentTextChanged(QString)), this, SLOT(onJointStateTopicChanged(QString)));
}

EnvironmentPanel::~EnvironmentPanel()
{
  data_->clear();
};

void EnvironmentPanel::onInitialize()
{
  data_->scene_manager = vis_manager_->getSceneManager();
  data_->scene_node = data_->scene_manager->createSceneNode();
}

void EnvironmentPanel::load( const rviz::Config& config)
{
  rviz::Panel::load( config );
  if( config.mapGetString( "EnvTopic", &data_->env_topic ))
  {
    data_->env_topic_editor->setCurrentText(data_->env_topic);
//    updateTopic();
  }
  if( config.mapGetString( "JointStateTopic", &data_->joint_state_topic ))
  {
    data_->joint_state_topic_editor->setCurrentText(data_->joint_state_topic);
//    updateTopic();
  }
}

void EnvironmentPanel::save( rviz::Config config ) const
{
  config.mapSetValue( "EnvTopic", data_->env_topic );
  config.mapSetValue( "JointStateTopic", data_->joint_state_topic );
  rviz::Panel::save( config );
}

void EnvironmentPanel::onDisplayModeChanged(int index)
{

}

void EnvironmentPanel::onEnvironmentTopicChanged(const QString &text)
{
  tesseract_environment::Environment::Ptr env;

}

void EnvironmentPanel::onJointStateTopicChanged(const QString &text)
{

}

void EnvironmentPanel::onEnvironmentSet(const tesseract_environment::Environment& /*env*/)
{
  data_->render_revision = 0;
  data_->render_dirty = true;
  data_->render_reset = true;
  data_->render_state_dirty = true;
}

void EnvironmentPanel::onEnvironmentChanged(const tesseract_environment::Environment& /*env*/)
{
  data_->render_dirty = true;
  data_->render_state_dirty = true;
}

void EnvironmentPanel::onEnvironmentCurrentStateChanged(const tesseract_environment::Environment& /*env*/)
{
  data_->render_state_dirty = true;
}

void EnvironmentPanel::onLinkVisibleChanged(const std::string& link_name, bool visible)
{
  data_->link_visible_changes[link_name] = visible;
  data_->render_dirty = true;
}

void EnvironmentPanel::onLinkCollisionVisibleChanged(const std::string& link_name, bool visible)
{
  data_->link_collision_visible_changes[link_name] = visible;
  data_->render_dirty = true;
}

void EnvironmentPanel::onLinkVisualVisibleChanged(const std::string& link_name, bool visible)
{
  data_->link_visual_visible_changes[link_name] = visible;
  data_->render_dirty = true;
}

void EnvironmentPanel::onSelectedLinksChanged(const std::vector<std::string>& selected_links)
{
  data_->link_selection_changes = selected_links;
  data_->render_dirty = true;
}

void EnvironmentPanel::onRender()
{
//  if (data_->render_dirty)
//  {
//    if (data_->render_reset) // Remove all
//    {
//      if (!data_->entity_container.empty())
//      {
////        for (const auto& id : data_->entity_container.getVisuals())
////          scene->DestroyNodeById(id.second);

////        for (const auto& id : data_->entity_container.getSensors())
////          scene->DestroyNodeById(id.second);

////        data_->entity_container.clear();
////        render_link_names_.clear();
////        render_revision_ = 0;
////        render_state_timestamp_ = environment().getCurrentStateTimestamp();
//      }
//      data_->render_reset = false;
//    }

//    { // Check environment
//      auto lock = data_->env_widget->environment().lockRead();
//      auto revision = data_->env_widget->environment().getRevision();
//      auto state_timestamp = data_->env_widget->environment().getCurrentStateTimestamp();
//      if (data_->render_dirty || revision > data_->render_revision)
//      {
//        if (revision > data_->render_revision)
//        {
//          tesseract_environment::Commands commands = data_->env_widget->environment().getCommandHistory();

//          bool links_removed {false};
//          for (std::size_t i = data_->render_revision; i < commands.size(); ++i)
//          {
////            const tesseract_environment::Command::ConstPtr& command = commands.at(i);
////            switch (command->getType())
////            {
////              case tesseract_environment::CommandType::ADD_SCENE_GRAPH:
////              {
////                auto cmd = std::static_pointer_cast<const tesseract_environment::AddSceneGraphCommand>(command);
////                auto link_names = loadSceneGraph(*scene, *entity_container_, *cmd->getSceneGraph(), cmd->getPrefix());
////                render_link_names_.insert(render_link_names_.end(), link_names.begin(), link_names.end());
////                break;
////              }
////              case tesseract_environment::CommandType::ADD_LINK:
////              {
////                auto cmd = std::static_pointer_cast<const tesseract_environment::AddLinkCommand>(command);
////                ignition::rendering::VisualPtr root = scene->RootVisual();
////                root->AddChild(loadLink(*scene, *entity_container_, *cmd->getLink()));
////                render_link_names_.push_back(cmd->getLink()->getName());
////                break;
////              }
////              case tesseract_environment::CommandType::CHANGE_LINK_VISIBILITY:
////              {
////                auto cmd = std::static_pointer_cast<const tesseract_environment::ChangeLinkVisibilityCommand>(command);
////                auto lc = entity_container_->getVisual(cmd->getLinkName());
////                auto visual_node = scene->VisualById(lc);
////                if (visual_node != nullptr)
////                  visual_node->SetVisible(cmd->getEnabled());
////                break;
////              }
////              case tesseract_environment::CommandType::REMOVE_LINK:
////              case tesseract_environment::CommandType::REMOVE_JOINT:
////              {
////                links_removed = true;
////                break;
////              }
////              case tesseract_environment::CommandType::MOVE_LINK:
////              case tesseract_environment::CommandType::MOVE_JOINT:
////              case tesseract_environment::CommandType::REPLACE_JOINT:
////              case tesseract_environment::CommandType::CHANGE_JOINT_ORIGIN:
////              case tesseract_environment::CommandType::CHANGE_LINK_ORIGIN:
////              {
////                render_state_dirty_ = true;
////                break;
////              }
////              case tesseract_environment::CommandType::CHANGE_LINK_COLLISION_ENABLED:
////              case tesseract_environment::CommandType::ADD_ALLOWED_COLLISION:
////              case tesseract_environment::CommandType::REMOVE_ALLOWED_COLLISION:
////              case tesseract_environment::CommandType::REMOVE_ALLOWED_COLLISION_LINK:
////              case tesseract_environment::CommandType::CHANGE_JOINT_POSITION_LIMITS:
////              case tesseract_environment::CommandType::CHANGE_JOINT_VELOCITY_LIMITS:
////              case tesseract_environment::CommandType::CHANGE_JOINT_ACCELERATION_LIMITS:
////              case tesseract_environment::CommandType::ADD_KINEMATICS_INFORMATION:
////              case tesseract_environment::CommandType::CHANGE_COLLISION_MARGINS:
////              case tesseract_environment::CommandType::ADD_CONTACT_MANAGERS_PLUGIN_INFO:
////              case tesseract_environment::CommandType::SET_ACTIVE_CONTINUOUS_CONTACT_MANAGER:
////              case tesseract_environment::CommandType::SET_ACTIVE_DISCRETE_CONTACT_MANAGER:
////              {
////                break;
////              }
////              // LCOV_EXCL_START
////              default:
////              {
////                CONSOLE_BRIDGE_logError("IgnitionEnvironmentWidget, Unhandled environment command");
//////                success &= false;
////              }
////                // LCOV_EXCL_STOP
////            }
////          }

////          if (links_removed)
////          {
////            std::vector<std::string> link_names = environment().getLinkNames();
////            std::vector<std::string> diff;

////            std::sort(link_names.begin(), link_names.end());
////            std::sort(render_link_names_.begin(), render_link_names_.end());

////            std::set_difference(render_link_names_.begin(),
////                                render_link_names_.end(),
////                                link_names.begin(),
////                                link_names.end(),
////                                std::inserter(diff, diff.begin()));

////            for (const auto& removed_link : diff)
////            {
////              auto id = entity_container_->getVisual(removed_link);
////              scene->DestroyNodeById(id);
////            }
////          }
////          render_revision_ = revision;
//        }
//      }

////      if (data_->render_state_dirty || state_timestamp > data_->render_state_timestamp)
////      {
////        tesseract_scene_graph::SceneState state = data_->env_widget->environment().getState();
////        for (const auto& pair : state.link_transforms)
////        {
////          auto id = data_->entity_container.getVisual(pair.first);
//////          scene->VisualById(id)->SetWorldPose(ignition::math::eigen3::convert(pair.second));
////        }
////        render_state_timestamp_ = state_timestamp;
////      }
//      }
//    }

//    if (data_->render_dirty)
//    {
//      for (const auto& l : data_->link_visible_changes)
//      {
//        auto lc = data_->entity_container.getVisual(l.first);
////        auto visual_node = scene->VisualById(lc);
////        if (visual_node != nullptr)
////          visual_node->SetVisible(l.second);
//      }

//      for (const auto& l : data_->link_visual_visible_changes)
//      {
//        std::string visual_key = l.first + "::Visuals";
//        auto lc = data_->entity_container.getVisual(visual_key);
////        auto visual_node = scene->VisualById(lc);
////        if (visual_node != nullptr)
////          visual_node->SetVisible(l.second);
//      }

//      for (const auto& l : data_->link_collision_visible_changes)
//      {
//        std::string visual_key = l.first + "::Collisions";
//        auto lc = data_->entity_container.getVisual(visual_key);
////        auto visual_node = scene->VisualById(lc);
////        if (visual_node != nullptr)
////          visual_node->SetVisible(l.second);
//      }

////      for (const auto& id : data_->highlighted_entities)
////      {
////        auto visual_node = scene->VisualById(id);
////        if (visual_node != nullptr)
////          visual_node->SetVisible(false);
////      }
////      highlighted_entities_.clear();

////      for (const auto& l : data_->link_selection_changes)
////      {
////        std::string visual_key = l + "::WireBox";
////        auto lc = data_->entity_container.getVisual(visual_key);
////        auto visual_node = scene->VisualById(lc);
////        if (visual_node != nullptr)
////        {
////          visual_node->SetVisible(true);
////          highlighted_entities_.push_back(lc);
////        }
////      }
//    }
//    data_->render_dirty = false;
//  }
}
}
