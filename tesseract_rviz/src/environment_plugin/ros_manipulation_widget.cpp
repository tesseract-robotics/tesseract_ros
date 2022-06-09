#include <tesseract_rviz/environment_plugin/ros_manipulation_widget.h>
#include <tesseract_rviz/environment_plugin/conversions.h>
#include <tesseract_rviz/environment_plugin/conversions.h>
#include <tesseract_rviz/conversions.h>
#include <tesseract_rviz/markers/utils.h>
#include <tesseract_rviz/interactive_marker/interactive_marker.h>

#include <tesseract_qt/common/entity_manager.h>
#include <tesseract_qt/common/link_visibility_properties.h>

#include <tesseract_environment/environment.h>

#include <rviz/display_context.h>

#include <set>
#include <QLayout>

namespace tesseract_rviz
{
struct ROSManipulationWidgetPrivate
{
  ROSManipulationWidgetPrivate()
    : entity_managers(
          { std::make_shared<tesseract_gui::EntityManager>(), std::make_shared<tesseract_gui::EntityManager>() }){};

  rviz::DisplayContext* context;
  Ogre::SceneNode* scene_node;

  std::array<tesseract_gui::EntityManager::Ptr, 2> entity_managers;

  /** @brief The current render group */
  QString render_group;

  /** @brief Indicates if something has changed */
  bool render_dirty;

  /** @brief Used for delayed initialization */
  bool render_reset;

  /** @brief Update visualization current state from environment message */
  std::vector<bool> render_states_dirty;
  std::vector<tesseract_scene_graph::SceneState> render_states;
  std::vector<tesseract_scene_graph::Material::ConstPtr> render_states_visual_material;
  std::vector<tesseract_scene_graph::Material::ConstPtr> render_states_collision_material;

  /** @brief The links with changed visibility properties */
  std::set<std::string> link_visibility_properties_changed;

  bool render_mode_dirty;
  bool render_mode_reset;
  Ogre::SceneNode* root_interactive_node;
  InteractiveMarker::Ptr interactive_marker;
  std::map<std::string, InteractiveMarker::Ptr> joint_interactive_markers;
  std::map<std::string, std::string> joint_interactive_marker_link_names;
  double interactive_marker_scale{ 0.5 };
  double joint_interactive_marker_scale{ 0.5 };
};

ROSManipulationWidget::ROSManipulationWidget(rviz::DisplayContext* context, Ogre::SceneNode* scene_node)
  : data_(std::make_unique<ROSManipulationWidgetPrivate>())
{
  layout()->setSizeConstraint(QLayout::SetNoConstraint);

  data_->context = context;
  data_->scene_node = scene_node;
  data_->root_interactive_node = scene_node->createChildSceneNode();

  data_->render_states_dirty.resize(getStateCount());
  data_->render_states.resize(getStateCount());
  data_->render_states_visual_material.resize(getStateCount());
  data_->render_states_collision_material.resize(getStateCount());

  addOgreResourceLocation();

  auto red = std::make_shared<tesseract_scene_graph::Material>("manipulation_red");
  red->color = Eigen::Vector4d(1, 0, 0, 0.75);

  auto green = std::make_shared<tesseract_scene_graph::Material>("manipulation_green");
  green->color = Eigen::Vector4d(0, 1, 0, 0.75);

  auto blue = std::make_shared<tesseract_scene_graph::Material>("manipulation_blue");
  blue->color = Eigen::Vector4d(0, 0, 1, 0.75);

  data_->render_states_visual_material[0] = red;
  //  data_->render_states_visual_material[1] = green;
  data_->render_states_collision_material[0] = blue;
  //  data_->render_states_collision_material[1] = blue;

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

  connect(this, SIGNAL(modeChanged(int)), this, SLOT(onModeChanged(int)));
  connect(this, &ROSManipulationWidget::tcpNameChanged, this, &ROSManipulationWidget::onTCPChanged);
  connect(this, &ROSManipulationWidget::tcpOffsetNameChanged, this, &ROSManipulationWidget::onTCPChanged);
}

ROSManipulationWidget::~ROSManipulationWidget()
{
  if (data_->root_interactive_node)
    data_->context->getSceneManager()->destroySceneNode(data_->root_interactive_node->getName());
};

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
        data_->context->getSceneManager()->destroyEntity(entity.unique_name);
    }
  }

  // Destroy Scene Nodes
  for (const auto& ns : container.getUntrackedEntities())
  {
    if (ns.first != container.RESOURCE_NS)
    {
      for (const auto& entity : ns.second)
        data_->context->getSceneManager()->destroySceneNode(entity.unique_name);
    }
  }

  for (const auto& ns : container.getTrackedEntities())
  {
    if (ns.first != container.RESOURCE_NS)
    {
      for (const auto& entity : ns.second)
        data_->context->getSceneManager()->destroySceneNode(entity.second.unique_name);
    }
  }
}

void ROSManipulationWidget::onEnvironmentSet(const std::shared_ptr<const tesseract_environment::Environment>& /*env*/)
{
  data_->render_group.clear();
  data_->render_dirty = true;
  data_->render_reset = true;
  for (std::size_t i = 0; i < data_->render_states_dirty.size(); ++i)
    data_->render_states_dirty[i] = true;

  for (std::size_t i = 0; i < data_->render_states.size(); ++i)
    data_->render_states[i] = tesseract_scene_graph::SceneState();

  data_->render_mode_dirty = true;
  data_->render_mode_reset = true;
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

  for (std::size_t i = 0; i < data_->render_states_dirty.size(); ++i)
    data_->render_states_dirty[i] = true;

  for (std::size_t i = 0; i < data_->render_states.size(); ++i)
    data_->render_states[i] = tesseract_scene_graph::SceneState();

  data_->render_mode_dirty = true;
  data_->render_mode_reset = true;
}

void ROSManipulationWidget::onManipulationStateChanged(const tesseract_scene_graph::SceneState& state, int state_index)
{
  data_->render_dirty = true;
  data_->render_states_dirty[state_index] = true;
  data_->render_states[state_index] = state;
}

void ROSManipulationWidget::onModeChanged(int mode)
{
  data_->render_dirty = true;
  data_->render_mode_dirty = true;
}

void ROSManipulationWidget::onTCPChanged()
{
  // Update Cartesian interactive marker
  if (data_->interactive_marker != nullptr)
  {
    Eigen::Isometry3d pose = getActiveCartesianTransform(true);
    Ogre::Vector3 position;
    Ogre::Quaternion orientation;
    toOgre(position, orientation, pose);
    data_->interactive_marker->setPose(position, orientation, "");
  }
}

void ROSManipulationWidget::addInteractiveMarker()
{
  auto lock = environment().lockRead();
  tesseract_scene_graph::SceneState state = getActiveState();
  tesseract_kinematics::KinematicGroup kin_group = kinematicGroup();
  std::string tcp_name = getTCPName().toStdString();
  int state_index = getActiveStateIndex();
  int mode = getMode();
  std::vector<std::string> joint_names = kin_group.getJointNames();

  // Add 6 DOF interactive marker at the end of the manipulator
  data_->interactive_marker = boost::make_shared<InteractiveMarker>(
      "6DOF", "Move Robot", data_->root_interactive_node, data_->context, data_->interactive_marker_scale);
  make6Dof(*data_->interactive_marker);

  Eigen::Isometry3d pose = state.link_transforms.at(tcp_name);  // * tcp_offset_;
  Ogre::Vector3 position;
  Ogre::Quaternion orientation;
  toOgre(position, orientation, pose);
  data_->interactive_marker->setPose(position, orientation, "");

  data_->interactive_marker->setShowAxes(false);
  data_->interactive_marker->setShowVisualAids(false);
  data_->interactive_marker->setShowDescription(false);
  data_->interactive_marker->setVisible(mode == 1);

  connect(data_->interactive_marker.get(),
          SIGNAL(userFeedback(std::string, const Eigen::Isometry3d&, const Eigen::Vector3d&, bool)),
          this,
          SLOT(markerFeedback(std::string, const Eigen::Isometry3d&, const Eigen::Vector3d&, bool)));

  // Add joint specific interactive marker
  data_->joint_interactive_markers.clear();
  data_->joint_interactive_marker_link_names.clear();
  for (const auto& joint_name : joint_names)
  {
    std::string name = joint_name + "_interactive_marker";
    std::string disc = "Move joint: " + joint_name;
    InteractiveMarker::Ptr interactive_marker = boost::make_shared<InteractiveMarker>(
        name, disc, data_->root_interactive_node, data_->context, data_->joint_interactive_marker_scale);
    const auto& joint = environment().getJoint(joint_name);

    switch (joint->type)
    {
      case tesseract_scene_graph::JointType::PRISMATIC:
      {
        Eigen::Vector3d disc_axis(1, 0, 0);
        Eigen::Quaternionf q = Eigen::Quaterniond::FromTwoVectors(disc_axis, joint->axis).cast<float>();

        InteractiveMarkerControl::Ptr control =
            interactive_marker->createInteractiveControl("move_" + joint_name,
                                                         "Move prismatic joint: " + joint_name,
                                                         InteractiveMode::MOVE_AXIS,
                                                         OrientationMode::INHERIT,
                                                         true,
                                                         Ogre::Quaternion(q.w(), q.x(), q.y(), q.z()));
        makeArrow(*control, 0.5);
        makeArrow(*control, -0.5);
        data_->joint_interactive_markers[joint_name] = interactive_marker;
        break;
      }
      case tesseract_scene_graph::JointType::REVOLUTE:
      {
        Eigen::Vector3d disc_axis(1, 0, 0);
        Eigen::Quaternionf q = Eigen::Quaterniond::FromTwoVectors(disc_axis, joint->axis).cast<float>();
        InteractiveMarkerControl::Ptr control =
            interactive_marker->createInteractiveControl("rotate_x",
                                                         "Rotate around X Axis",
                                                         InteractiveMode::ROTATE_AXIS,
                                                         OrientationMode::INHERIT,
                                                         true,
                                                         Ogre::Quaternion(q.w(), q.x(), q.y(), q.z()));
        makeDisc(*control, 0.3f);
        data_->joint_interactive_markers[joint_name] = interactive_marker;
        break;
      }
      case tesseract_scene_graph::JointType::CONTINUOUS:
      {
        Eigen::Vector3d disc_axis(1, 0, 0);
        Eigen::Quaternionf q = Eigen::Quaterniond::FromTwoVectors(disc_axis, joint->axis).cast<float>();
        InteractiveMarkerControl::Ptr control =
            interactive_marker->createInteractiveControl("rotate_x",
                                                         "Rotate around X Axis",
                                                         InteractiveMode::ROTATE_AXIS,
                                                         OrientationMode::INHERIT,
                                                         true,
                                                         Ogre::Quaternion(q.w(), q.x(), q.y(), q.z()));
        makeDisc(*control, 0.3f);
        data_->joint_interactive_markers[joint_name] = interactive_marker;
        break;
      }
      default:
        assert(false);
    }

    data_->joint_interactive_marker_link_names[joint_name] = joint->child_link_name;
    Eigen::Isometry3d pose = state.link_transforms.at(joint->child_link_name);
    Ogre::Vector3 position;
    Ogre::Quaternion orientation;
    toOgre(position, orientation, pose);
    interactive_marker->setPose(position, orientation, "");
    interactive_marker->setShowAxes(false);
    interactive_marker->setShowVisualAids(false);
    interactive_marker->setShowDescription(false);
    interactive_marker->setVisible(mode == 0);

    auto fn = std::bind(&tesseract_rviz::ROSManipulationWidget::jointMarkerFeedback,
                        this,
                        joint_name,
                        std::placeholders::_1,
                        std::placeholders::_2,
                        std::placeholders::_3,
                        std::placeholders::_4);

    connect(interactive_marker.get(), &tesseract_rviz::InteractiveMarker::userFeedback, this, fn);
  }
}

void ROSManipulationWidget::markerFeedback(const std::string& reference_frame,
                                           const Eigen::Isometry3d& transform,
                                           const Eigen::Vector3d& /*mouse_point*/,
                                           bool /*mouse_point_valid*/)
{
  const tesseract_kinematics::KinematicGroup& kin_group = kinematicGroup();
  const tesseract_scene_graph::SceneState& state = getActiveState();
  const std::string working_frame = getWorkingFrame().toStdString();

  Eigen::Isometry3d tf_world{ transform };
  auto it = state.link_transforms.find(reference_frame);
  if (it != state.link_transforms.end())
    tf_world = it->second * transform;

  const Eigen::Isometry3d tf_working_frame = state.link_transforms.at(working_frame).inverse() * tf_world;
  setActiveCartesianTransform(tf_working_frame);
}

void ROSManipulationWidget::jointMarkerFeedback(const std::string& joint_name,
                                                const std::string& /*reference_frame*/,
                                                const Eigen::Isometry3d& transform,
                                                const Eigen::Vector3d& /*mouse_point*/,
                                                bool /*mouse_point_valid*/)
{
  const tesseract_kinematics::KinematicGroup& kin_group = kinematicGroup();
  tesseract_scene_graph::SceneState scene_state = getActiveState();
  int current_state_index = getActiveStateIndex();

  tesseract_scene_graph::Joint::ConstPtr joint = environment().getJoint(joint_name);
  double current_joint_value = scene_state.joints.at(joint_name);
  Eigen::Isometry3d child_pose =
      scene_state.link_transforms.at(data_->joint_interactive_marker_link_names.at(joint_name));
  Eigen::Isometry3d delta_pose = child_pose.inverse() * transform;

  Eigen::Vector3d delta_axis;
  double delta_joint_value = 0;
  switch (joint->type)
  {
    case tesseract_scene_graph::JointType::PRISMATIC:
    {
      delta_axis = delta_pose.translation().normalized();
      delta_joint_value = delta_pose.translation().norm();
      break;
    }
    case tesseract_scene_graph::JointType::REVOLUTE:
    {
      Eigen::AngleAxisd delta_rotation;
      delta_rotation.fromRotationMatrix(delta_pose.rotation());

      delta_axis = delta_rotation.axis();
      delta_joint_value = delta_rotation.angle();
      break;
    }
    case tesseract_scene_graph::JointType::CONTINUOUS:
    {
      Eigen::AngleAxisd delta_rotation;
      delta_rotation.fromRotationMatrix(delta_pose.rotation());

      delta_axis = delta_rotation.axis();
      delta_joint_value = delta_rotation.angle();
      break;
    }
    default:
      assert(false);
  }

  double new_joint_value;
  if (delta_axis.dot(joint->axis) > 0)
    new_joint_value = current_joint_value + delta_joint_value;
  else
    new_joint_value = current_joint_value - delta_joint_value;

  Eigen::MatrixX2d limits = kin_group.getLimits().joint_limits;
  int i = 0;
  std::unordered_map<std::string, double> state;
  for (const auto& j : kin_group.getJointNames())
  {
    if (joint_name == j)
    {
      if (new_joint_value > limits(i, 1))
        new_joint_value = limits(i, 1);
      else if (new_joint_value < limits(i, 0))
        new_joint_value = limits(i, 0);

      state[j] = new_joint_value;
    }
    else
    {
      state[j] = scene_state.joints[j];
    }

    ++i;
  }

  setActiveState(state);
}

void ROSManipulationWidget::onRender(float dt)
{
  if (!isValid())
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

        data_->render_states.at(i) = getState(static_cast<int>(i));

        if (!data_->render_group.isEmpty())
        {
          auto jg = environment().getJointGroup(data_->render_group.toStdString());
          std::vector<std::string> link_names = jg->getActiveLinkNames();
          for (const auto& link_name : link_names)
          {
            auto link = environment().getLink(link_name);
            auto entity_container = entity_manager->getEntityContainer(link->getName());
            Ogre::SceneNode* sn = loadLink(*data_->context->getSceneManager(),
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

    int current_state_index = getActiveStateIndex();
    for (std::size_t i = 0; i < getStateCount(); ++i)
    {
      auto& entity_manager = data_->entity_managers[i];

      // Update start state visualization
      if (data_->render_states_dirty[i])
      {
        const tesseract_scene_graph::SceneState& render_state = data_->render_states[i];
        for (const auto& pair : render_state.link_transforms)
        {
          if (entity_manager->hasEntityContainer(pair.first))
          {
            auto container = entity_manager->getEntityContainer(pair.first);
            Ogre::Vector3 position;
            Ogre::Quaternion orientation;
            toOgre(position, orientation, pair.second);

            auto entity = container->getTrackedEntity(tesseract_gui::EntityContainer::VISUAL_NS, pair.first);
            Ogre::SceneNode* sn = data_->context->getSceneManager()->getSceneNode(entity.unique_name);
            sn->setPosition(position);
            sn->setOrientation(orientation);
          }
        }

        // Interactive markers are only created of the active state
        if (i == current_state_index)
        {
          // Update Cartesian interactive marker
          if (data_->interactive_marker != nullptr && !data_->interactive_marker->isDragging())
          {
            Eigen::Isometry3d pose = getActiveCartesianTransform(true);
            Ogre::Vector3 position;
            Ogre::Quaternion orientation;
            toOgre(position, orientation, pose);
            data_->interactive_marker->setPose(position, orientation, "");
          }

          // Update joint interactive markers
          for (auto& jm : data_->joint_interactive_markers)
          {
            if (!jm.second->isDragging())
            {
              Eigen::Isometry3d pose =
                  render_state.link_transforms.at(data_->joint_interactive_marker_link_names[jm.first]);
              Ogre::Vector3 position;
              Ogre::Quaternion orientation;
              toOgre(position, orientation, pose);
              jm.second->setPose(position, orientation, "");
            }
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
                Ogre::SceneNode* sn = data_->context->getSceneManager()->getSceneNode(entity.unique_name);
                sn->setVisible(link_visibility_property.link, true);
              }
            }

            {  // Link Visual Property
              std::string visual_key = l + "::Visuals";
              if (entity_container->hasTrackedEntity(tesseract_gui::EntityContainer::VISUAL_NS, visual_key))
              {
                auto entity = entity_container->getTrackedEntity(tesseract_gui::EntityContainer::VISUAL_NS, visual_key);
                Ogre::SceneNode* sn = data_->context->getSceneManager()->getSceneNode(entity.unique_name);
                sn->setVisible(link_visibility_property.link && link_visibility_property.visual, true);
              }
            }

            {  // Link Collision Property
              std::string visual_key = l + "::Collisions";
              if (entity_container->hasTrackedEntity(tesseract_gui::EntityContainer::VISUAL_NS, visual_key))
              {
                auto entity = entity_container->getTrackedEntity(tesseract_gui::EntityContainer::VISUAL_NS, visual_key);
                Ogre::SceneNode* sn = data_->context->getSceneManager()->getSceneNode(entity.unique_name);
                sn->setVisible(link_visibility_property.link && link_visibility_property.collision, true);
              }
            }

            {  // Link WireBox Property
              std::string visual_key = l + "::WireBox";
              if (entity_container->hasTrackedEntity(tesseract_gui::EntityContainer::VISUAL_NS, visual_key))
              {
                auto entity = entity_container->getTrackedEntity(tesseract_gui::EntityContainer::VISUAL_NS, visual_key);
                Ogre::SceneNode* sn = data_->context->getSceneManager()->getSceneNode(entity.unique_name);
                sn->setVisible(link_visibility_property.link && link_visibility_property.wirebox, true);
              }
            }

            {  // Link Axis Property
              std::string visual_key = l + "::Axis";
              if (entity_container->hasTrackedEntity(tesseract_gui::EntityContainer::VISUAL_NS, visual_key))
              {
                auto entity = entity_container->getTrackedEntity(tesseract_gui::EntityContainer::VISUAL_NS, visual_key);
                Ogre::SceneNode* sn = data_->context->getSceneManager()->getSceneNode(entity.unique_name);
                sn->setVisible(link_visibility_property.link && link_visibility_property.axis, true);
              }
            }
          }
        }
      }
      data_->link_visibility_properties_changed.clear();
    }

    if (data_->render_mode_dirty)
    {
      if (data_->render_mode_reset)
      {
        data_->interactive_marker = nullptr;
        data_->joint_interactive_markers.clear();
        data_->joint_interactive_marker_link_names.clear();
        addInteractiveMarker();
        data_->render_mode_reset = false;
      }

      int mode = getMode();
      data_->interactive_marker->setVisible(mode == 1);
      for (auto& jm : data_->joint_interactive_markers)
        jm.second->setVisible(mode == 0);
    }
    data_->render_dirty = false;
  }

  if (data_->interactive_marker != nullptr)
    data_->interactive_marker->update(dt);

  for (auto& joint_marker : data_->joint_interactive_markers)
    joint_marker.second->update(dt);
}
}  // namespace tesseract_rviz
