#include <tesseract_rviz/environment_plugin/environment_display.h>
#include <tesseract_rviz/environment_plugin/ros_environment_widget.h>
#include <tesseract_rviz/environment_plugin/conversions.h>

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
  tesseract_rviz::ROSEnvironmentWidget* widget;

  rviz::EnumProperty* display_mode_property;
  rviz::StringProperty* urdf_description_string_property;
  rviz::RosTopicProperty* environment_topic_property;
  rviz::RosTopicProperty* joint_state_topic_property;
  rviz::FloatProperty* scene_alpha_property;
  rviz::BoolProperty* scene_visual_visible;
  rviz::BoolProperty* scene_collision_visible;
  rviz::BoolProperty* scene_links_visible;
  rviz::BoolProperty* scene_wirebox_visible;

  /** @brief Keeps track of how many EnvironmentWidgets have been created for the default namespace */
  static int environment_display_counter;

  /** @brief Keeps track of which EnvironmentWidget this is */
  int environment_display_id;

  std::string environment_display_ns;
};

EnvironmentDisplayPrivate::EnvironmentDisplayPrivate()
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

  data_->scene_wirebox_visible = new rviz::BoolProperty("Show Wire Box",
                                                        false,
                                                        "Whether to display the wire box representation of the "
                                                        "environment.",
                                                        this,
                                                        SLOT(onSceneWireBoxVisibleChanged()),
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
  data_->widget = new tesseract_rviz::ROSEnvironmentWidget(scene_manager_, scene_node_);
  setAssociatedWidget(data_->widget);

  onDisplayModeChanged();
}

void EnvironmentDisplay::reset() { Display::reset(); }

void EnvironmentDisplay::update(float wall_dt, float ros_dt)
{
  Display::update(wall_dt, ros_dt);

  data_->widget->onUpdate();
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

void EnvironmentDisplay::onSceneAlphaChanged() {}

void EnvironmentDisplay::onSceneVisualVisibleChanged()
{
  std::vector<std::string> link_names = data_->widget->environment().getLinkNames();
  for (const auto& link_name : link_names)
    data_->widget->onLinkVisualVisibleChanged(link_name, data_->scene_visual_visible->getBool());
}

void EnvironmentDisplay::onSceneCollisionVisibleChanged()
{
  std::vector<std::string> link_names = data_->widget->environment().getLinkNames();
  for (const auto& link_name : link_names)
    data_->widget->onLinkCollisionVisibleChanged(link_name, data_->scene_collision_visible->getBool());
}

void EnvironmentDisplay::onSceneWireBoxVisibleChanged()
{
  std::vector<std::string> link_names;
  if (data_->scene_wirebox_visible->getBool())
    link_names = data_->widget->environment().getLinkNames();

  data_->widget->onSelectedLinksChanged(link_names);
}

void EnvironmentDisplay::onSceneLinkVisibleChanged()
{
  std::vector<std::string> link_names = data_->widget->environment().getLinkNames();
  for (const auto& link_name : link_names)
    data_->widget->onLinkVisibleChanged(link_name, data_->scene_links_visible->getBool());

  if (data_->scene_links_visible->getBool())
  {
    onSceneVisualVisibleChanged();
    onSceneCollisionVisibleChanged();
    onSceneWireBoxVisibleChanged();
  }
}

}  // namespace tesseract_rviz
