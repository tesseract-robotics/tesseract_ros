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

#include <tesseract_common/macros.h>
TESSERACT_COMMON_IGNORE_WARNINGS_PUSH
#include <memory>

#include <boost/algorithm/string.hpp>
#include <boost/algorithm/string/replace.hpp>

#include <rviz/display_context.h>
#include <rviz/properties/bool_property.h>
#include <rviz/properties/color_property.h>
#include <rviz/properties/editable_enum_property.h>
#include <rviz/properties/enum_property.h>
#include <rviz/properties/float_property.h>
#include <rviz/properties/int_property.h>
#include <rviz/properties/property.h>
#include <rviz/properties/ros_topic_property.h>
#include <rviz/properties/string_property.h>
#include <rviz/window_manager_interface.h>

#include <tesseract_command_language/utils/utils.h>
#include <tesseract_rosutils/utils.h>
TESSERACT_COMMON_IGNORE_WARNINGS_POP

#include "tesseract_rviz/render_tools/trajectory_monitor_widget.h"
#include "tesseract_rviz/render_tools/visualization_widget.h"
#include "tesseract_rviz/render_tools/link_widget.h"

namespace tesseract_rviz
{
const double SLIDER_RESOLUTION = 0.001;

TrajectoryMonitorWidget::TrajectoryMonitorWidget(rviz::Property* widget, rviz::Display* display)
  : widget_(widget)
  , display_(display)
  , visualize_trajectory_widget_(std::make_shared<VisualizeTrajectoryWidget>(widget, display))  // should widget be
                                                                                                // this?
{
  main_property_ = new rviz::Property(
      "Trajectory Monitor", "", "Monitor a joint state topic and update the visualization", widget_, nullptr, this);

  trajectory_topic_property_ = new rviz::RosTopicProperty("Topic",
                                                          "/tesseract/display_tesseract_trajectory",
                                                          ros::message_traits::datatype<tesseract_msgs::Trajectory>(),
                                                          "The topic on which the tesseract_msgs::Trajectory messages "
                                                          "are received",
                                                          main_property_,
                                                          SLOT(changedTrajectoryTopic()),
                                                          this);
}

TrajectoryMonitorWidget::~TrajectoryMonitorWidget() {}

void TrajectoryMonitorWidget::onInitialize(VisualizationWidget::Ptr visualization,
                                           tesseract_environment::Environment::Ptr env,
                                           rviz::DisplayContext* context,
                                           const ros::NodeHandle& update_nh)
{
  // Save pointers for later use
  context_ = context;
  nh_ = update_nh;

  visualize_trajectory_widget_->onInitialize(visualization, env, context);
}

void TrajectoryMonitorWidget::onEnable()
{
  visualize_trajectory_widget_->onEnable();
  changedTrajectoryTopic();
}

void TrajectoryMonitorWidget::onDisable() { visualize_trajectory_widget_->onDisable(); }

void TrajectoryMonitorWidget::onUpdate(float wall_dt) { visualize_trajectory_widget_->onUpdate(wall_dt); }

void TrajectoryMonitorWidget::onReset() { visualize_trajectory_widget_->onReset(); }

void TrajectoryMonitorWidget::onNameChange(const QString& name) { visualize_trajectory_widget_->onNameChange(name); }

void TrajectoryMonitorWidget::changedTrajectoryTopic()
{
  trajectory_topic_sub_.shutdown();
  if (!trajectory_topic_property_->getStdString().empty())
  {
    trajectory_topic_sub_ = nh_.subscribe(
        trajectory_topic_property_->getStdString(), 5, &TrajectoryMonitorWidget::incomingDisplayTrajectory, this);
  }
}

void TrajectoryMonitorWidget::incomingDisplayTrajectory(const tesseract_msgs::Trajectory::ConstPtr& msg)
{
  visualize_trajectory_widget_->setDisplayTrajectory(msg);
}

}  // namespace tesseract_rviz
