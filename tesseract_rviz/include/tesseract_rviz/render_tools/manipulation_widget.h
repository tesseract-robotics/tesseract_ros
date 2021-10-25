/**
 * @file manipulation_widget.h
 * @brief A manipulators widget for moving the robot around in rviz.
 *
 * @author Levi Armstrong
 * @date Dec 18, 2017
 * @version TODO
 * @bug No known bugs
 *
 * @copyright Copyright (c) 2017, Southwest Research Institute
 *
 * @par License
 * Software License Agreement (Apache License)
 * @par
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 * http://www.apache.org/licenses/LICENSE-2.0
 * @par
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */
#ifndef TESSERACT_RVIZ_MANIPULATION_WIDGET_H
#define TESSERACT_RVIZ_MANIPULATION_WIDGET_H

#include <tesseract_common/macros.h>
TESSERACT_COMMON_IGNORE_WARNINGS_PUSH
#include <rviz/display.h>
#include <rviz/panel_dock_widget.h>
TESSERACT_COMMON_IGNORE_WARNINGS_POP

#ifndef Q_MOC_RUN
TESSERACT_COMMON_IGNORE_WARNINGS_PUSH
#include <ros/ros.h>
#include <tesseract_msgs/Trajectory.h>
#include <tesseract_environment/environment.h>
#include <sensor_msgs/JointState.h>
#include <boost/thread/mutex.hpp>
#include <tesseract_kinematics/core/inverse_kinematics.h>
TESSERACT_COMMON_IGNORE_WARNINGS_POP
#endif

#include <tesseract_rviz/property/button_property.h>
#include <tesseract_rviz/render_tools/visualization_widget.h>
#include <tesseract_rviz/interactive_marker/interactive_marker.h>

namespace rviz
{
class Property;
class RosTopicProperty;
class EnumProperty;
class FloatProperty;
class BoolProperty;
class StringProperty;
}  // namespace rviz

namespace Ogre
{
class SceneNode;
}

namespace tesseract_rviz
{
class ManipulationWidget : public QObject
{
  Q_OBJECT

public:
  using Ptr = std::shared_ptr<ManipulationWidget>;
  using ConstPtr = std::shared_ptr<const ManipulationWidget>;

  enum class ManipulatorState
  {
    START = 0,
    END = 1
  };

  ManipulationWidget(rviz::Property* widget, rviz::Display* display);

  virtual ~ManipulationWidget();

  void onInitialize(Ogre::SceneNode* root_node,
                    rviz::DisplayContext* context,
                    VisualizationWidget::Ptr visualization,
                    tesseract_environment::Environment::Ptr env,
                    const ros::NodeHandle& update_nh,
                    ManipulatorState state,
                    const QString& joint_state_topic);

  void onEnable();
  void onDisable();
  void onUpdate(float wall_dt);
  void onReset();

  void onNameChange(const QString& name);

Q_SIGNALS:
  void availableManipulatorsChanged(QStringList manipulators);
  void availableTCPFramesChanged(QStringList tcp_frames);
  void availableTCPOffsetsChanged(tesseract_common::TransformMap tcp_offsets);
  void availableWorkingFramesChanged(QStringList working_frames);

public
  Q_SLOT : void enableCartesianManipulation(bool enabled);
  void enableJointManipulation(bool enabled);
  void resetToCurrentState();
  bool changeManipulator(const QString& manipulator);
  bool changeTCPFrame(const QString& tcp_frame);
  bool changeTCPOffset(const QString& tcp_offset);
  bool changeWorkingFrame(const QString& working_frame);

private Q_SLOTS:
  void changedManipulator();
  void changedTCPFrame();
  void changedTCPOffset();
  void changedWorkingFrame();
  void changedJointStateTopic();
  void changedCartesianMarkerScale();
  void changedCartesianManipulationEnabled();
  void changedJointMarkerScale();
  void changedJointManipulationEnabled();
  void clickedResetToCurrentState();
  void userInputJointValuesChanged();
  void markerFeedback(const std::string& reference_frame,
                      const Eigen::Isometry3d& transform,
                      const Eigen::Vector3d& mouse_point,
                      bool mouse_point_valid);
  void jointMarkerFeedback(const std::string& joint_name,
                           const std::string& reference_frame,
                           const Eigen::Isometry3d& transform,
                           const Eigen::Vector3d& mouse_point,
                           bool mouse_point_valid);
  //  void trajectorySliderPanelVisibilityChange(bool enable);

protected:
  Ogre::SceneNode* root_interactive_node_;
  rviz::Property* widget_;
  rviz::Display* display_;
  rviz::DisplayContext* context_;
  VisualizationWidget::Ptr visualization_;
  tesseract_environment::Environment::Ptr env_;
  ros::NodeHandle nh_;
  ManipulatorState state_;
  InteractiveMarker::Ptr interactive_marker_;
  std::map<std::string, InteractiveMarker::Ptr> joint_interactive_markers_;
  std::set<std::string> manipulators_;
  tesseract_kinematics::KinematicGroup::UPtr manip_;
  Eigen::VectorXd inv_seed_;
  int env_revision_;
  std::unordered_map<std::string, double> joints_;
  tesseract_scene_graph::SceneState env_state_;
  Eigen::Isometry3d tcp_offset_;

  ros::Publisher joint_state_pub_;
  QStringList available_manipulators_;
  QStringList available_tcp_frames_;
  QStringList available_working_frames_;
  tesseract_common::TransformMap available_tcp_offsets_;

  //  TrajectoryPanel* trajectory_slider_panel_;
  //  rviz::PanelDockWidget* trajectory_slider_dock_panel_;

  // Properties
  bool enabled_;
  ButtonProperty* main_property_;
  rviz::EnumProperty* manipulator_property_;
  rviz::RosTopicProperty* joint_state_topic_property_;
  rviz::BoolProperty* cartesian_manipulation_property_;
  rviz::BoolProperty* joint_manipulation_property_;
  rviz::FloatProperty* cartesian_marker_scale_property_;
  rviz::FloatProperty* joint_marker_scale_property_;
  rviz::EnumProperty* tcp_frame_property_;
  rviz::EnumProperty* tcp_offset_property_;
  rviz::EnumProperty* working_frame_property_;
  rviz::Property* joint_values_property_;
  rviz::StringProperty* joint_config_property_;
  rviz::EnumProperty* joint_config_base_link_property_;
  rviz::EnumProperty* joint_config_tip_link_property_;
  rviz::EnumProperty* joint3_sign_property_;
  rviz::EnumProperty* joint5_sign_property_;

  void updateJointConfig();
  void updateEnvironmentVisualization();
  void updateCartesianMarkerVisualization();
  void udpateJointMarkerVisualization();
  void publishJointStates();
};
}  // namespace tesseract_rviz

#endif  // TESSERACT_RVIZ_MANIPULATION_WIDGET_H
