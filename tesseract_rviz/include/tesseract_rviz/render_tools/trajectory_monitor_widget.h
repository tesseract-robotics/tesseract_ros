/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2008, Willow Garage, Inc.
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of Willow Garage nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *********************************************************************/

/* Author: Dave Coleman */

#ifndef TESSERACT_RVIZ_TRAJECTORY_MONITOR_WIDGET
#define TESSERACT_RVIZ_TRAJECTORY_MONITOR_WIDGET

#include <tesseract_common/macros.h>
TESSERACT_COMMON_IGNORE_WARNINGS_PUSH
#include <rviz/display.h>
#include <rviz/panel_dock_widget.h>
#include <boost/thread/mutex.hpp>
TESSERACT_COMMON_IGNORE_WARNINGS_POP

#ifndef Q_MOC_RUN
TESSERACT_COMMON_IGNORE_WARNINGS_PUSH
#include <ros/ros.h>
#include <tesseract_msgs/Trajectory.h>
#include <tesseract_environment/environment.h>
#include <tesseract_common/types.h>
TESSERACT_COMMON_IGNORE_WARNINGS_POP
#endif

#include <tesseract_rviz/render_tools/visualize_trajectory_widget.h>

namespace rviz
{
class Shape;
class Property;
class IntProperty;
class StringProperty;
class BoolProperty;
class FloatProperty;
class RosTopicProperty;
class EditableEnumProperty;
class EnumProperty;
class ColorProperty;
class MovableText;
}  // namespace rviz

namespace tesseract_rviz
{
class TrajectoryMonitorWidget : public QObject
{
  Q_OBJECT

public:
  using Ptr = std::shared_ptr<TrajectoryMonitorWidget>;
  using ConstPtr = std::shared_ptr<const TrajectoryMonitorWidget>;

  TrajectoryMonitorWidget(rviz::Property* widget, rviz::Display* display);

  virtual ~TrajectoryMonitorWidget();

  void onInitialize(VisualizationWidget::Ptr visualization,
                    tesseract_environment::Environment::Ptr env,
                    rviz::DisplayContext* context,
                    const ros::NodeHandle& update_nh);

  void onEnable();
  void onDisable();
  void onReset();
  void onUpdate(float wall_dt);
  void onNameChange(const QString& name);

private Q_SLOTS:
  void changedTrajectoryTopic();

public:
  void incomingDisplayTrajectory(const tesseract_msgs::Trajectory::ConstPtr& msg);

protected:
  rviz::Property* widget_;
  rviz::Display* display_;
  rviz::DisplayContext* context_;
  ros::NodeHandle nh_;
  VisualizeTrajectoryWidget::Ptr visualize_trajectory_widget_;

  /**
   * @brief These are the command that were pushed to motion planning
   * @details Visualization only supports moving objects around in a trajectory visualization.
   * If it includes commands which delete or adds links visualization may not be correct.
   * @todo Need to maintain two environments one which tracks the monitored environment and then
   * when new request are recieved it clones this environment an applys the commands througout the
   * trajectory widget.
   */
  tesseract_environment::Commands trajectory_env_commands_;

  ros::Subscriber trajectory_topic_sub_;
  boost::mutex update_trajectory_message_;

  // Properties
  rviz::Property* main_property_;
  rviz::RosTopicProperty* trajectory_topic_property_;
};

}  // namespace tesseract_rviz

#endif
