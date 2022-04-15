#include <tesseract_rviz/environment_plugin/joint_trajectory_monitor_properties.h>
#include <tesseract_rviz/environment_plugin/conversions.h>

#include <tesseract_widgets/joint_trajectory/joint_trajectory_widget.h>
#include <tesseract_widgets/common/joint_trajectory_set.h>

#include <trajectory_msgs/JointTrajectory.h>
#include <tesseract_msgs/Trajectory.h>
#include <tesseract_rosutils/utils.h>

#include <rviz/display.h>
#include <rviz/properties/ros_topic_property.h>
#include <rviz/panel_dock_widget.h>

#include <ros/subscriber.h>
#include <unordered_map>

namespace tesseract_rviz
{
struct JointTrajectoryMonitorPropertiesPrivate
{
  ros::NodeHandle nh;
  rviz::Display* parent;
  rviz::Property* main_property;
  tesseract_gui::JointTrajectoryWidget* widget;

  rviz::BoolProperty* legacy_main;
  rviz::RosTopicProperty* legacy_joint_trajectory_topic_property;

  rviz::BoolProperty* tesseract_main;
  rviz::RosTopicProperty* tesseract_joint_trajectory_topic_property;

  ros::Subscriber legacy_joint_trajectory_sub;
  ros::Subscriber tesseract_joint_trajectory_sub;

  void legacyJointTrajectoryCallback(const trajectory_msgs::JointTrajectory::ConstPtr& msg);
  void tesseractJointTrajectoryCallback(const tesseract_msgs::Trajectory::ConstPtr& msg);
};

JointTrajectoryMonitorProperties::JointTrajectoryMonitorProperties(rviz::Display* parent, rviz::Property* main_property)
  : data_(std::make_unique<JointTrajectoryMonitorPropertiesPrivate>())
{
  data_->parent = parent;

  data_->main_property = main_property;
  if (data_->main_property == nullptr)
    data_->main_property = data_->parent;

  data_->legacy_main = new rviz::BoolProperty("Legacy Joint Trajectory",
                                              true,
                                              "This will monitor this topic for trajectory_msgs::JointTrajectory "
                                              "messages.",
                                              data_->main_property,
                                              SLOT(onLegacyJointTrajectoryChanged()),
                                              this);
  data_->legacy_main->setDisableChildrenIfFalse(true);

  data_->legacy_joint_trajectory_topic_property =
      new rviz::RosTopicProperty("topic",
                                 "/joint_trajectory",
                                 ros::message_traits::datatype<trajectory_msgs::JointTrajectory>(),
                                 "This will monitor this topic for trajectory_msgs::JointTrajectory messages.",
                                 data_->legacy_main,
                                 SLOT(onLegacyJointTrajectoryTopicChanged()),
                                 this);

  data_->tesseract_main = new rviz::BoolProperty("Tesseract Joint Trajectory",
                                                 true,
                                                 "This will monitor this topic for trajectory_msgs::JointTrajectory "
                                                 "messages.",
                                                 data_->main_property,
                                                 SLOT(onTesseractJointTrajectoryChanged()),
                                                 this);
  data_->tesseract_main->setDisableChildrenIfFalse(true);

  data_->tesseract_joint_trajectory_topic_property =
      new rviz::RosTopicProperty("Joint Trajectory Topic",
                                 "/tesseract_trajectory",
                                 ros::message_traits::datatype<tesseract_msgs::Trajectory>(),
                                 "This will monitor this topic for tesseract_msgs::Trajectory messages.",
                                 data_->tesseract_main,
                                 SLOT(onTesseractJointTrajectoryTopicChanged()),
                                 this);
}

JointTrajectoryMonitorProperties::~JointTrajectoryMonitorProperties() = default;

void JointTrajectoryMonitorProperties::onInitialize(tesseract_gui::JointTrajectoryWidget* widget)
{
  data_->widget = widget;
  onLegacyJointTrajectoryTopicConnect();
  onTesseractJointTrajectoryTopicConnect();
}

void JointTrajectoryMonitorProperties::load(const rviz::Config& config)
{
  QString topic;
  if (config.mapGetString("tesseract::LegacyJointTrajectoryTopic", &topic))
    data_->legacy_joint_trajectory_topic_property->setString(topic);

  if (config.mapGetString("tesseract::TesseractJointTrajectoryTopic", &topic))
    data_->tesseract_joint_trajectory_topic_property->setString(topic);
}

void JointTrajectoryMonitorProperties::save(rviz::Config config) const
{
  config.mapSetValue("tesseract::LegacyJointTrajectoryTopic",
                     data_->legacy_joint_trajectory_topic_property->getString());
  config.mapSetValue("tesseract::TesseractJointTrajectoryTopic",
                     data_->tesseract_joint_trajectory_topic_property->getString());
}

void JointTrajectoryMonitorProperties::onLegacyJointTrajectoryTopicConnect()
{
  data_->legacy_joint_trajectory_sub =
      data_->nh.subscribe(data_->legacy_joint_trajectory_topic_property->getStdString(),
                          20,
                          &JointTrajectoryMonitorPropertiesPrivate::legacyJointTrajectoryCallback,
                          data_.get());
}

void JointTrajectoryMonitorProperties::onTesseractJointTrajectoryTopicConnect()
{
  data_->tesseract_joint_trajectory_sub =
      data_->nh.subscribe(data_->tesseract_joint_trajectory_topic_property->getStdString(),
                          20,
                          &JointTrajectoryMonitorPropertiesPrivate::tesseractJointTrajectoryCallback,
                          data_.get());
}

void JointTrajectoryMonitorProperties::onLegacyJointTrajectoryTopicDisconnect()
{
  data_->legacy_joint_trajectory_sub.shutdown();
}

void JointTrajectoryMonitorProperties::onTesseractJointTrajectoryTopicDisconnect()
{
  data_->tesseract_joint_trajectory_sub.shutdown();
}

void JointTrajectoryMonitorProperties::onLegacyJointTrajectoryTopicChanged()
{
  onLegacyJointTrajectoryTopicDisconnect();
  onLegacyJointTrajectoryTopicConnect();
}

void JointTrajectoryMonitorProperties::onTesseractJointTrajectoryTopicChanged()
{
  onTesseractJointTrajectoryTopicDisconnect();
  onTesseractJointTrajectoryTopicConnect();
}

void JointTrajectoryMonitorProperties::onLegacyJointTrajectoryChanged()
{
  if (data_->legacy_main->getBool())
    onLegacyJointTrajectoryTopicConnect();
  else
    onLegacyJointTrajectoryTopicDisconnect();
}

void JointTrajectoryMonitorProperties::onTesseractJointTrajectoryChanged()
{
  if (data_->tesseract_main->getBool())
    onTesseractJointTrajectoryTopicConnect();
  else
    onTesseractJointTrajectoryTopicDisconnect();
}

void JointTrajectoryMonitorPropertiesPrivate::legacyJointTrajectoryCallback(
    const trajectory_msgs::JointTrajectory::ConstPtr& msg)
{
  if (msg->joint_names.empty())
    return;

  if (msg->points.empty())
    return;

  std::unordered_map<std::string, double> initial_state;
  for (std::size_t i = 0; i < msg->joint_names.size(); ++i)
    initial_state[msg->joint_names[i]] = msg->points[0].positions[i];

  tesseract_common::JointTrajectorySet trajectory_set(initial_state);
  tesseract_common::JointTrajectory joint_trajectory = tesseract_rosutils::fromMsg(*msg);
  trajectory_set.appendJointTrajectory(joint_trajectory);
  widget->addJointTrajectorySet(trajectory_set);
}

void JointTrajectoryMonitorPropertiesPrivate::tesseractJointTrajectoryCallback(
    const tesseract_msgs::Trajectory::ConstPtr& msg)
{
  try
  {
    tesseract_common::JointTrajectorySet trajectory_set;

    // Get environment initial state
    std::unordered_map<std::string, double> initial_state;
    for (const auto& pair_msg : msg->initial_state)
      initial_state[pair_msg.first] = pair_msg.second;

    // Get environment information
    tesseract_environment::Environment::UPtr environment = tesseract_rosutils::fromMsg(msg->environment);
    tesseract_environment::Commands commands = tesseract_rosutils::fromMsg(msg->commands);

    if (environment != nullptr)
    {
      trajectory_set = tesseract_common::JointTrajectorySet(initial_state, std::move(environment));
    }
    else if (!commands.empty())
    {
      trajectory_set = tesseract_common::JointTrajectorySet(initial_state, commands);
    }
    else
    {
      trajectory_set = tesseract_common::JointTrajectorySet(initial_state);
    }

    for (const auto& joint_trajectory_msg : msg->joint_trajectories)
    {
      tesseract_common::JointTrajectory joint_trajectory = tesseract_rosutils::fromMsg(joint_trajectory_msg);
      trajectory_set.appendJointTrajectory(joint_trajectory, joint_trajectory_msg.description);
      widget->addJointTrajectorySet(trajectory_set);
    }
  }
  catch (...)
  {
    parent->setStatus(rviz::StatusProperty::Error, "Tesseract", "Failed to process trajectory message!");
  }
}

}  // namespace tesseract_rviz
