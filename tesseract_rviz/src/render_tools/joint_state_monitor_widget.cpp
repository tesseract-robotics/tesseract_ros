#include <tesseract_common/macros.h>
TESSERACT_COMMON_IGNORE_WARNINGS_PUSH
#include <rviz/display_context.h>
#include <rviz/properties/ros_topic_property.h>
#include <rviz/window_manager_interface.h>

#include <tesseract_rosutils/utils.h>
TESSERACT_COMMON_IGNORE_WARNINGS_POP

#include <tesseract_rviz/render_tools/joint_state_monitor_widget.h>

namespace tesseract_rviz
{
JointStateMonitorWidget::JointStateMonitorWidget(rviz::Property* widget, rviz::Display* display)
  : widget_(widget), display_(display), visualization_(nullptr), env_(nullptr), update_required_(false)
{
  main_property_ = new rviz::Property(
      "Joint State Monitor", "", "Monitor a joint state topic and update the visualization", widget_, nullptr, this);

  joint_state_topic_property_ = new rviz::RosTopicProperty("Topic",
                                                           "joint_states",
                                                           ros::message_traits::datatype<sensor_msgs::JointState>(),
                                                           "The topic on which the sensor_msgs::JointState messages "
                                                           "are received",
                                                           main_property_,
                                                           SLOT(changedJointStateTopic()),
                                                           this);
}

JointStateMonitorWidget::~JointStateMonitorWidget() { joint_state_subscriber_.shutdown(); }

void JointStateMonitorWidget::onInitialize(VisualizationWidget::Ptr visualization,
                                           tesseract_environment::Environment::Ptr env,
                                           rviz::DisplayContext* /*context*/,
                                           const ros::NodeHandle& update_nh)
{
  visualization_ = std::move(visualization);
  env_ = std::move(env);
  nh_ = update_nh;
}

void JointStateMonitorWidget::changedJointStateTopic()
{
  joint_state_subscriber_.shutdown();

  joint_state_subscriber_ = nh_.subscribe(
      joint_state_topic_property_->getStdString(), 10, &JointStateMonitorWidget::newJointStateCallback, this);
}

void JointStateMonitorWidget::newJointStateCallback(const sensor_msgs::JointStateConstPtr& joint_state_msg)
{
  if (!env_->isInitialized())
    return;

  if (isUpdateRequired(*joint_state_msg))
  {
    tesseract_rosutils::processMsg(env_, *joint_state_msg);
    update_required_ = true;
  }
}

void JointStateMonitorWidget::onEnable() { changedJointStateTopic(); }

void JointStateMonitorWidget::onDisable() { joint_state_subscriber_.shutdown(); }

void JointStateMonitorWidget::onUpdate()
{
  if (visualization_ && update_required_ && env_)
  {
    update_required_ = false;
    visualization_->update(env_->getCurrentState()->link_transforms);
  }
}

void JointStateMonitorWidget::onReset() { changedJointStateTopic(); }

bool JointStateMonitorWidget::isUpdateRequired(const sensor_msgs::JointState& joint_state)
{
  std::unordered_map<std::string, double> joints = env_->getCurrentState()->joints;
  for (auto i = 0u; i < joint_state.name.size(); ++i)
    if (std::abs(joints[joint_state.name[i]] - joint_state.position[i]) > 1e-5)
      return true;

  return false;
}

}  // namespace tesseract_rviz
