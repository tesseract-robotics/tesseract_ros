#include <tesseract_rviz/environment_monitor_properties.h>
#include <tesseract_rviz/ros_scene_graph_render_manager.h>
#include <tesseract_rviz/ros_contact_results_render_manager.h>
#include <tesseract_rviz/conversions.h>

#include <tesseract_qt/common/component_info.h>
#include <tesseract_qt/common/environment_manager.h>
#include <tesseract_qt/common/environment_wrapper.h>

#include <tesseract_monitoring/environment_monitor.h>

#include <tesseract_msgs/EnvironmentState.h>
#include <sensor_msgs/JointState.h>

#include <rviz/display.h>
#include <rviz/properties/ros_topic_property.h>
#include <rviz/properties/string_property.h>
#include <rviz/properties/enum_property.h>
#include <rviz/panel_dock_widget.h>

#include <unordered_map>

namespace tesseract_rviz
{
struct EnvironmentMonitorProperties::Implementation
{
  ros::NodeHandle nh;
  rviz::Display* parent{ nullptr };
  rviz::Property* main_property{ nullptr };
  Ogre::SceneManager* scene_manager{ nullptr };
  Ogre::SceneNode* scene_node{ nullptr };
  std::string monitor_namespace;
  tesseract_environment::EnvironmentMonitor::Ptr monitor;

  tesseract_gui::ComponentInfo component_info{ "rviz_scene" };
  tesseract_gui::SceneGraphRenderManager::Ptr render_manager;
  tesseract_gui::ContactResultsRenderManager::Ptr contact_results_render_manager;

  rviz::EnumProperty* display_mode_property{ nullptr };
  rviz::StringProperty* urdf_description_string_property{ nullptr };
  rviz::RosTopicProperty* environment_topic_property{ nullptr };
  rviz::RosTopicProperty* joint_state_topic_property{ nullptr };
};

EnvironmentMonitorProperties::EnvironmentMonitorProperties(rviz::Display* parent,
                                                           std::string monitor_namespace,
                                                           rviz::Property* main_property)
  : data_(std::make_unique<Implementation>())
{
  data_->parent = parent;
  data_->monitor_namespace = monitor_namespace;

  data_->main_property = main_property;
  if (data_->main_property == nullptr)
    data_->main_property = data_->parent;

  data_->display_mode_property = new rviz::EnumProperty("Display Mode",
                                                        "URDF",
                                                        "Leverage URDF or connect to monitor namespace",
                                                        data_->main_property,
                                                        SLOT(onDisplayModeChanged()),
                                                        this);

  data_->display_mode_property->addOptionStd("URDF", 0);
  data_->display_mode_property->addOptionStd("Monitor", 1);

  data_->urdf_description_string_property = new rviz::StringProperty("URDF Parameter",
                                                                     "robot_description",
                                                                     "The URDF parameter to use for creating the "
                                                                     "environment within RViz",
                                                                     data_->main_property,
                                                                     SLOT(onURDFDescriptionChanged()),
                                                                     this);

  data_->environment_topic_property =
      new rviz::RosTopicProperty("Monitor Topic",
                                 "/tesseract_environment",
                                 ros::message_traits::datatype<tesseract_msgs::EnvironmentState>(),
                                 "This will monitor this topic for environment changes.",
                                 data_->main_property,
                                 SLOT(onEnvironmentTopicChanged()),
                                 this);

  data_->joint_state_topic_property =
      new rviz::RosTopicProperty("Joint State Topic",
                                 "/joint_states",
                                 ros::message_traits::datatype<sensor_msgs::JointState>(),
                                 "This will monitor this topic for joint state changes.",
                                 data_->main_property,
                                 SLOT(onJointStateTopicChanged()),
                                 this);
}

EnvironmentMonitorProperties::~EnvironmentMonitorProperties()
{
  tesseract_gui::EnvironmentManager::remove(data_->component_info);
}

void EnvironmentMonitorProperties::onInitialize(Ogre::SceneManager* scene_manager, Ogre::SceneNode* scene_node)
{
  data_->scene_manager = scene_manager;
  data_->scene_node = scene_node;
  onDisplayModeChanged();
}

tesseract_gui::ComponentInfo EnvironmentMonitorProperties::getComponentInfo() const { return data_->component_info; }

void EnvironmentMonitorProperties::load(const rviz::Config& config)
{
  QString mode;
  if (config.mapGetString("tesseract::EnvMonitorMode", &mode))
    data_->display_mode_property->setString(mode);

  QString urdf_description;
  if (config.mapGetString("tesseract::EnvMonitorURDFDescription", &urdf_description))
    data_->urdf_description_string_property->setString(urdf_description);

  QString topic;
  if (config.mapGetString("tesseract::EnvMonitorTopic", &topic))
    data_->environment_topic_property->setString(topic);

  if (config.mapGetString("tesseract::EnvMonitorJointStateTopic", &topic))
    data_->joint_state_topic_property->setString(topic);
}

void EnvironmentMonitorProperties::save(rviz::Config config) const
{
  config.mapSetValue("tesseract::EnvMonitorMode", data_->display_mode_property->getString());
  config.mapSetValue("tesseract::EnvMonitorURDFDescription", data_->urdf_description_string_property->getString());
  config.mapSetValue("tesseract::EnvMonitorTopic", data_->environment_topic_property->getString());
  config.mapSetValue("tesseract::EnvMonitorJointStateTopic", data_->joint_state_topic_property->getString());
}

void EnvironmentMonitorProperties::onDisplayModeChanged()
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

void EnvironmentMonitorProperties::onURDFDescriptionChanged()
{
  if (data_->scene_manager == nullptr || data_->scene_node == nullptr)
    return;

  tesseract_gui::EnvironmentManager::remove(data_->component_info);
  data_->render_manager = nullptr;
  data_->contact_results_render_manager = nullptr;

  std::string urdf_xml_string, srdf_xml_string;
  data_->nh.getParam(data_->urdf_description_string_property->getStdString(), urdf_xml_string);
  data_->nh.getParam(data_->urdf_description_string_property->getStdString() + "_semantic", srdf_xml_string);

  auto env = std::make_shared<tesseract_environment::Environment>();
  auto locator = std::make_shared<tesseract_rosutils::ROSResourceLocator>();
  if (env->init(urdf_xml_string, srdf_xml_string, locator))
  {
    if (data_->monitor != nullptr)
      data_->monitor->shutdown();

    data_->monitor = std::make_unique<tesseract_monitoring::ROSEnvironmentMonitor>(
        env, data_->urdf_description_string_property->getStdString());
    if (data_->monitor != nullptr)
    {
      data_->render_manager =
          std::make_shared<ROSSceneGraphRenderManager>(data_->component_info, data_->scene_manager, data_->scene_node);
      data_->contact_results_render_manager = std::make_shared<ROSContactResultsRenderManager>(
          data_->component_info, data_->scene_manager, data_->scene_node);

      Q_EMIT componentInfoChanged(data_->component_info);

      auto env_wrapper =
          std::make_shared<tesseract_gui::MonitorEnvironmentWrapper>(data_->component_info, data_->monitor);
      tesseract_gui::EnvironmentManager::set(env_wrapper);

      onJointStateTopicChanged();
    }
  }
  else
  {
    data_->parent->setStatus(rviz::StatusProperty::Error, "Tesseract", "URDF file failed to parse");
  }
}

void EnvironmentMonitorProperties::onEnvironmentTopicChanged()
{
  if (data_->scene_manager == nullptr || data_->scene_node == nullptr)
    return;

  if (data_->monitor != nullptr)
    data_->monitor->shutdown();

  tesseract_gui::EnvironmentManager::remove(data_->component_info);
  data_->render_manager = nullptr;
  data_->contact_results_render_manager = nullptr;

  auto env = std::make_shared<tesseract_environment::Environment>();
  data_->monitor = std::make_unique<tesseract_monitoring::ROSEnvironmentMonitor>(env, data_->monitor_namespace);

  if (data_->monitor != nullptr)
  {
    std::string ns = getEnvNamespaceFromTopic(data_->environment_topic_property->getStdString());
    if (!ns.empty())
      data_->monitor->startMonitoringEnvironment(ns);
    else
      data_->parent->setStatus(rviz::StatusProperty::Error, "Tesseract", "Invalid environment monitor topic!");

    data_->render_manager =
        std::make_shared<ROSSceneGraphRenderManager>(data_->component_info, data_->scene_manager, data_->scene_node);
    data_->contact_results_render_manager = std::make_shared<ROSContactResultsRenderManager>(
        data_->component_info, data_->scene_manager, data_->scene_node);

    Q_EMIT componentInfoChanged(data_->component_info);

    auto env_wrapper =
        std::make_shared<tesseract_gui::MonitorEnvironmentWrapper>(data_->component_info, data_->monitor);
    tesseract_gui::EnvironmentManager::set(env_wrapper);

    onJointStateTopicChanged();
  }
}

void EnvironmentMonitorProperties::onJointStateTopicChanged()
{
  if (data_->scene_manager == nullptr || data_->scene_node == nullptr)
    return;

  if (data_->monitor != nullptr)
    data_->monitor->startStateMonitor(data_->joint_state_topic_property->getStdString(), false);
}

}  // namespace tesseract_rviz
