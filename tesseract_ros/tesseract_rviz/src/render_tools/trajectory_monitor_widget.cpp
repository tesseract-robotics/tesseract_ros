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
#include <tesseract_command_language/deserialize.h>
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
  , visualization_(nullptr)
  , tesseract_(nullptr)
  , cached_visible_(false)
  , animating_path_(false)
  , drop_displaying_trajectory_(false)
  , trajectory_slider_panel_(nullptr)
  , trajectory_slider_dock_panel_(nullptr)
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

  display_mode_property_ = new rviz::EnumProperty(
      "Display Mode", "Loop", "How to display the trajectoy.", main_property_, SLOT(changedDisplayMode()), this);
  display_mode_property_->addOptionStd("Single", 0);
  display_mode_property_->addOptionStd("Loop", 1);
  display_mode_property_->addOptionStd("Trail", 2);

  time_scale_property_ = new rviz::FloatProperty("Time Scale",
                                                 1,
                                                 "A time scale factor applied during play back of trajectory",
                                                 main_property_,
                                                 SLOT(changedTimeScale()),
                                                 this);
  time_scale_property_->setMin(1e-8f);

  trail_step_size_property_ = new rviz::IntProperty("Trail Step Size",
                                                    1,
                                                    "Specifies the step size of the samples "
                                                    "shown in the trajectory trail.",
                                                    main_property_,
                                                    SLOT(changedTrailStepSize()),
                                                    this);
  trail_step_size_property_->setMin(1);

  interrupt_display_property_ = new rviz::BoolProperty("Interrupt Display",
                                                       false,
                                                       "Immediately show newly planned trajectory, "
                                                       "interrupting the currently displayed one.",
                                                       main_property_);
}

TrajectoryMonitorWidget::~TrajectoryMonitorWidget()
{
  clearTrajectoryTrail();
  displaying_instruction_ = tesseract_planning::NullInstruction();
  trajectory_to_display_instruction_ = tesseract_planning::NullInstruction();

  delete trajectory_slider_dock_panel_;
}

void TrajectoryMonitorWidget::onInitialize(VisualizationWidget::Ptr visualization,
                                           tesseract::Tesseract::Ptr tesseract,
                                           rviz::DisplayContext* context,
                                           const ros::NodeHandle& update_nh)
{
  // Save pointers for later use
  visualization_ = std::move(visualization);
  tesseract_ = std::move(tesseract);
  context_ = context;
  nh_ = update_nh;

  previous_display_mode_ = display_mode_property_->getOptionInt();

  rviz::WindowManagerInterface* window_context = context_->getWindowManager();
  if (window_context)
  {
    trajectory_slider_panel_ = new TrajectoryPanel(window_context->getParentWindow());
    trajectory_slider_dock_panel_ =
        window_context->addPane(display_->getName() + " - Slider", trajectory_slider_panel_);
    trajectory_slider_dock_panel_->setIcon(display_->getIcon());
    connect(trajectory_slider_dock_panel_,
            SIGNAL(visibilityChanged(bool)),
            this,
            SLOT(trajectorySliderPanelVisibilityChange(bool)));
    trajectory_slider_panel_->onInitialize();
  }
}

void TrajectoryMonitorWidget::onEnable()
{
  visualization_->setTrajectoryVisible(cached_visible_);
  changedTrajectoryTopic();  // load topic at startup if default used
}

void TrajectoryMonitorWidget::onDisable()
{
  cached_visible_ = visualization_->isTrajectoryVisible();
  visualization_->setTrajectoryVisible(false);
  displaying_instruction_ = tesseract_planning::NullInstruction();
  animating_path_ = false;

  if (trajectory_slider_panel_)
    trajectory_slider_panel_->onDisable();
}

void TrajectoryMonitorWidget::onReset()
{
  clearTrajectoryTrail();
  displaying_instruction_ = tesseract_planning::NullInstruction();
  trajectory_to_display_instruction_ = tesseract_planning::NullInstruction();
  animating_path_ = false;
}

void TrajectoryMonitorWidget::onNameChange(const QString& name)
{
  if (trajectory_slider_dock_panel_)
    trajectory_slider_dock_panel_->setWindowTitle(name + " - Slider");
}

void TrajectoryMonitorWidget::clearTrajectoryTrail()
{
  for (auto& link_pair : visualization_->getLinks())
    link_pair.second->clearTrajectory();
}

void TrajectoryMonitorWidget::createTrajectoryTrail()
{
  clearTrajectoryTrail();

  long stepsize = trail_step_size_property_->getInt();
  // always include last trajectory point
  long num_waypoints = trajectory_player_.size();
  num_trail_waypoints_ =
      static_cast<size_t>(std::ceil(static_cast<float>(num_waypoints + stepsize - 1) / static_cast<float>(stepsize)));
  std::vector<tesseract_environment::EnvState::Ptr> states_data;
  states_data.reserve(num_trail_waypoints_);
  for (std::size_t i = 0; i < num_trail_waypoints_; i++)
  {
    // limit to last trajectory point
    auto waypoint_i = static_cast<long>(std::min(static_cast<long>(i) * stepsize, num_waypoints - 1));
    std::unordered_map<std::string, double> joints;
    tesseract_planning::MoveInstruction mi = trajectory_player_.getByIndex(waypoint_i);
    const Eigen::VectorXd& joint_values = tesseract_planning::getJointPosition(mi.getWaypoint());
    const std::vector<std::string>& joint_names = tesseract_planning::getJointNames(mi.getWaypoint());
    states_data.push_back(tesseract_->getEnvironment()->getState(joint_names, joint_values));
  }

  // If current state is not visible must set trajectory for all links for a single state so static
  // objects will be visible
  for (const auto& tf : states_data[0]->link_transforms)
  {
    LinkWidget* lw = visualization_->getLink(tf.first);
    lw->clearTrajectory();

    if (!visualization_->isCurrentStateVisible() && !visualization_->isStartStateVisible())
      lw->setTrajectory({ tf.second });
  }

  // Set Trajectory for active links
  for (const auto& link_name : tesseract_->getEnvironment()->getActiveLinkNames())
  {
    std::vector<Eigen::Isometry3d> link_trajectory;
    link_trajectory.reserve(states_data.size());
    for (auto& state : states_data)
    {
      link_trajectory.push_back(state->link_transforms[link_name]);
    }
    visualization_->getLink(link_name)->setTrajectory(link_trajectory);
  }
}

void TrajectoryMonitorWidget::changedDisplayMode()
{
  if (display_mode_property_->getOptionInt() != 2)
  {
    if (display_->isEnabled() && !tesseract_planning::isNullInstruction(displaying_instruction_) && animating_path_)
      return;

    visualization_->setStartStateVisible(true);
    visualization_->setTrajectoryVisible(false);
    trajectory_player_.enableLoop(display_mode_property_->getOptionInt() == 1);

    clearTrajectoryTrail();

    if (trajectory_slider_panel_)
      trajectory_slider_panel_->pauseButton(false);
  }
  else
  {
    visualization_->setStartStateVisible(false);
    visualization_->setTrajectoryVisible(true);
    if (trajectory_slider_panel_)
      trajectory_slider_panel_->pauseButton(true);
  }
}

void TrajectoryMonitorWidget::changedTrailStepSize()
{
  if (display_mode_property_->getOptionInt() == 2)
    createTrajectoryTrail();
}

void TrajectoryMonitorWidget::changedTrajectoryTopic()
{
  trajectory_topic_sub_.shutdown();
  if (!trajectory_topic_property_->getStdString().empty())
  {
    trajectory_topic_sub_ = nh_.subscribe(
        trajectory_topic_property_->getStdString(), 5, &TrajectoryMonitorWidget::incomingDisplayTrajectory, this);
  }
}

void TrajectoryMonitorWidget::changedTimeScale()
{
  trajectory_player_.setScale(static_cast<double>(time_scale_property_->getFloat()));
}

void TrajectoryMonitorWidget::interruptCurrentDisplay()
{
  // update() starts a new trajectory as soon as it is available
  // interrupting may cause the newly received trajectory to interrupt
  // hence, only interrupt when current_state_ already advanced past first
  if (trajectory_player_.currentDuration() > 0)
    animating_path_ = false;
}

void TrajectoryMonitorWidget::dropTrajectory() { drop_displaying_trajectory_ = true; }
void TrajectoryMonitorWidget::onUpdate(float /*wall_dt*/)
{
  if (!tesseract_->isInitialized() || !visualization_)
    return;

  if (drop_displaying_trajectory_)
  {
    animating_path_ = false;
    displaying_instruction_ = tesseract_planning::NullInstruction();
    trajectory_slider_panel_->update(0);
    drop_displaying_trajectory_ = false;
    trajectory_player_.reset();
  }

  if (!animating_path_)
  {  // finished last animation?

    boost::mutex::scoped_lock lock(update_trajectory_message_);
    // new trajectory available to display?
    if (tesseract_planning::isCompositeInstruction(trajectory_to_display_instruction_))
    {
      animating_path_ = true;

      if (display_mode_property_->getOptionInt() == 2)
        animating_path_ = false;

      trajectory_player_.setProgram(
          *trajectory_to_display_instruction_.cast_const<tesseract_planning::CompositeInstruction>());
      slider_count_ = static_cast<int>(std::ceil(trajectory_player_.trajectoryDuration() / SLIDER_RESOLUTION));
      createTrajectoryTrail();
      if (trajectory_slider_panel_)
        trajectory_slider_panel_->update(slider_count_);
    }
    else if (tesseract_planning::isCompositeInstruction(displaying_instruction_))
    {
      if (display_mode_property_->getOptionInt() == 1)
      {
        animating_path_ = true;
      }
      else if (display_mode_property_->getOptionInt() == 0)
      {
        if (previous_display_mode_ != display_mode_property_->getOptionInt())
        {
          animating_path_ = true;
        }
        else
        {
          if (trajectory_slider_panel_->getSliderPosition() == (slider_count_ - 1))
            animating_path_ = false;
          else
            animating_path_ = true;
        }
      }
      else
      {
        if (previous_display_mode_ != display_mode_property_->getOptionInt())
        {
          createTrajectoryTrail();
          if (trajectory_slider_panel_)
            trajectory_slider_panel_->update(slider_count_);
        }

        animating_path_ = false;
      }
      previous_display_mode_ = display_mode_property_->getOptionInt();
    }
    trajectory_to_display_instruction_ = tesseract_planning::NullInstruction();

    if (animating_path_)
    {
      tesseract_planning::MoveInstruction mi = trajectory_player_.setCurrentDuration(0);
      const Eigen::VectorXd& joint_values = tesseract_planning::getJointPosition(mi.getWaypoint());
      const std::vector<std::string>& joint_names = tesseract_planning::getJointNames(mi.getWaypoint());
      tesseract_environment::EnvState::Ptr state = tesseract_->getEnvironment()->getState(joint_names, joint_values);
      visualization_->setStartState(state->link_transforms);

      if (trajectory_slider_panel_)
        trajectory_slider_panel_->setSliderPosition(0);
    }
  }

  if (animating_path_)
  {
    tesseract_environment::EnvState::Ptr state;
    if (trajectory_slider_panel_ != nullptr && trajectory_slider_panel_->isPaused())
    {
      double duration = static_cast<double>(trajectory_slider_panel_->getSliderPosition()) /
                        static_cast<double>(trajectory_slider_panel_->getSliderPosition());
      tesseract_planning::MoveInstruction mi = trajectory_player_.setCurrentDuration(duration);
      const Eigen::VectorXd& joint_values = tesseract_planning::getJointPosition(mi.getWaypoint());
      const std::vector<std::string>& joint_names = tesseract_planning::getJointNames(mi.getWaypoint());
      state = tesseract_->getEnvironment()->getState(joint_names, joint_values);
    }
    else
    {
      tesseract_planning::MoveInstruction mi = trajectory_player_.getNext();
      const Eigen::VectorXd& joint_values = tesseract_planning::getJointPosition(mi.getWaypoint());
      const std::vector<std::string>& joint_names = tesseract_planning::getJointNames(mi.getWaypoint());
      state = tesseract_->getEnvironment()->getState(joint_names, joint_values);

      if (trajectory_slider_panel_ != nullptr)
      {
        int slider_index = static_cast<int>(
            std::ceil((trajectory_player_.currentDuration() / trajectory_player_.trajectoryDuration()) *
                      static_cast<double>(slider_count_)));
        trajectory_slider_panel_->setSliderPosition(slider_index);
      }
    }

    visualization_->setStartState(state->link_transforms);
  }
}

void TrajectoryMonitorWidget::incomingDisplayTrajectory(const tesseract_msgs::Trajectory::ConstPtr& msg)
{
  // Error check
  if (!tesseract_->isInitialized())
  {
    ROS_ERROR_STREAM_NAMED("trajectory_visualization", "No environment");
    return;
  }

  if (visualization_)
  {
    //    if (!visualization_->isTrajectoryVisible())
    //      visualization_->setTrajectoryVisible(true);
    visualization_->setTrajectoryVisible(false);

    if (!visualization_->isStartStateVisible())
      visualization_->setStartStateVisible(true);
  }

  if (!msg->tesseract_state.id.empty() && msg->tesseract_state.id != tesseract_->getEnvironment()->getName())
    ROS_WARN("Received a trajectory to display for model '%s' but model '%s' "
             "was expected",
             msg->tesseract_state.id.c_str(),
             tesseract_->getEnvironment()->getName().c_str());

  if (!msg->instructions.empty())
  {
    tesseract_planning::Instruction program = tesseract_planning::fromXMLString(msg->instructions);
    boost::mutex::scoped_lock lock(update_trajectory_message_);
    trajectory_to_display_instruction_ = program;
    if (interrupt_display_property_->getBool())
      interruptCurrentDisplay();
  }
  else if (!msg->joint_trajectory.points.empty())
  {
    tesseract_planning::CompositeInstruction composite;
    for (unsigned i = 0; i < msg->joint_trajectory.points.size(); ++i)
    {
      tesseract_planning::StateWaypoint swp;
      const trajectory_msgs::JointTrajectoryPoint& p = msg->joint_trajectory.points[i];
      swp.joint_names = msg->joint_trajectory.joint_names;
      swp.position = Eigen::Map<const Eigen::VectorXd>(p.positions.data(), static_cast<long>(p.positions.size()));
      swp.velocity = Eigen::Map<const Eigen::VectorXd>(p.velocities.data(), static_cast<long>(p.velocities.size()));
      swp.acceleration =
          Eigen::Map<const Eigen::VectorXd>(p.accelerations.data(), static_cast<long>(p.accelerations.size()));
      swp.time = p.time_from_start.toSec();
      composite.push_back(tesseract_planning::MoveInstruction(swp, tesseract_planning::MoveInstructionType::FREESPACE));
    }

    boost::mutex::scoped_lock lock(update_trajectory_message_);
    trajectory_to_display_instruction_ = composite;
    if (interrupt_display_property_->getBool())
      interruptCurrentDisplay();
  }
  else
  {
    trajectory_to_display_instruction_ = tesseract_planning::NullInstruction();
  }
}

void TrajectoryMonitorWidget::trajectorySliderPanelVisibilityChange(bool enable)
{
  if (!trajectory_slider_panel_)
    return;

  if (enable)
    trajectory_slider_panel_->onEnable();
  else
    trajectory_slider_panel_->onDisable();
}

}  // namespace tesseract_rviz
