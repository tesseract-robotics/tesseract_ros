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
#include <tesseract_command_language/core/serialization.h>
#include <tesseract_rosutils/utils.h>
TESSERACT_COMMON_IGNORE_WARNINGS_POP

#include "tesseract_rviz/render_tools/visualize_trajectory_widget.h"
#include "tesseract_rviz/render_tools/visualization_widget.h"
#include "tesseract_rviz/render_tools/link_widget.h"

namespace tesseract_rviz
{
const double SLIDER_RESOLUTION = 0.001;

VisualizeTrajectoryWidget::VisualizeTrajectoryWidget(rviz::Property* widget, rviz::Display* display)
  : widget_(widget)
  , display_(display)
  , visualization_(nullptr)
  , env_(nullptr)
  , cached_visible_(false)
  , animating_path_(false)
  , drop_displaying_trajectory_(false)
  , trajectory_slider_panel_(nullptr)
  , trajectory_slider_dock_panel_(nullptr)
{
  main_property_ = new rviz::Property("Trajectory Visualization", "", "Displays a trajectory", widget_, nullptr, this);

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

VisualizeTrajectoryWidget::~VisualizeTrajectoryWidget()
{
  clearTrajectoryTrail();
  displaying_trajectory_.clear();
  trajectory_to_display_.clear();

  delete trajectory_slider_dock_panel_;
}

void VisualizeTrajectoryWidget::onInitialize(VisualizationWidget::Ptr visualization,
                                             tesseract_environment::Environment::Ptr env,
                                             rviz::DisplayContext* context)
{
  // Save pointers for later use
  visualization_ = std::move(visualization);
  env_ = std::move(env);
  context_ = context;

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

void VisualizeTrajectoryWidget::onEnable() { visualization_->setTrajectoryVisible(cached_visible_); }

void VisualizeTrajectoryWidget::onDisable()
{
  cached_visible_ = visualization_->isTrajectoryVisible();
  visualization_->setTrajectoryVisible(false);
  displaying_trajectory_.clear();
  animating_path_ = false;

  if (trajectory_slider_panel_)
    trajectory_slider_panel_->onDisable();
}

void VisualizeTrajectoryWidget::onReset()
{
  clearTrajectoryTrail();
  displaying_trajectory_.clear();
  trajectory_to_display_.clear();
  animating_path_ = false;
}

void VisualizeTrajectoryWidget::onNameChange(const QString& name)
{
  if (trajectory_slider_dock_panel_)
    trajectory_slider_dock_panel_->setWindowTitle(name + " - Slider");
}

void VisualizeTrajectoryWidget::clearTrajectoryTrail()
{
  for (auto& link_pair : visualization_->getLinks())
    link_pair.second->clearTrajectory();
}

void VisualizeTrajectoryWidget::createTrajectoryTrail()
{
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
    tesseract_common::JointState joint_state = trajectory_player_.getByIndex(waypoint_i);
    states_data.push_back(env_->getState(joint_state.joint_names, joint_state.position));
  }

  std::vector<std::string> active_link_names = env_->getActiveLinkNames();

  // If current state is not visible must set trajectory for all links for a single state so static
  // objects will be visible
  for (const auto& tf : states_data[0]->link_transforms)
  {
    // Active links get set in the next stage below do not set them here
    if (std::find(active_link_names.begin(), active_link_names.end(), tf.first) != active_link_names.end())
      continue;

    LinkWidget* lw = visualization_->getLink(tf.first);
    lw->hideTrajectory();

    if (!visualization_->isCurrentStateVisible() && !visualization_->isStartStateVisible())
    {
      lw->setTrajectory({ tf.second });
      lw->showTrajectory();
    }
  }

  // Set Trajectory for active links
  for (const auto& link_name : env_->getActiveLinkNames())
  {
    tesseract_common::VectorIsometry3d link_trajectory;
    link_trajectory.reserve(states_data.size());
    for (auto& state : states_data)
    {
      link_trajectory.push_back(state->link_transforms[link_name]);
    }
    LinkWidget* l = visualization_->getLink(link_name);
    l->setTrajectory(link_trajectory);
    l->showTrajectory();
  }
}

void VisualizeTrajectoryWidget::changedDisplayMode()
{
  if (display_mode_property_->getOptionInt() != 2)
  {
    if (display_->isEnabled() && !displaying_trajectory_.empty() && animating_path_)
      return;

    visualization_->setStartStateVisible(true);
    visualization_->setTrajectoryVisible(false);

    clearTrajectoryTrail();

    if (trajectory_slider_panel_)
      trajectory_slider_panel_->pauseButton(false);
  }
  else
  {
    animating_path_ = false;

    visualization_->setStartStateVisible(false);
    visualization_->setTrajectoryVisible(false);

    if (trajectory_slider_panel_)
      trajectory_slider_panel_->pauseButton(true);
  }
}

void VisualizeTrajectoryWidget::changedTrailStepSize()
{
  if (display_mode_property_->getOptionInt() == 2)
    createTrajectoryTrail();
}

void VisualizeTrajectoryWidget::changedTimeScale()
{
  trajectory_player_.setScale(static_cast<double>(time_scale_property_->getFloat()));
}

void VisualizeTrajectoryWidget::interruptCurrentDisplay()
{
  // update() starts a new trajectory as soon as it is available
  // interrupting may cause the newly received trajectory to interrupt
  // hence, only interrupt when current_state_ already advanced past first
  if (trajectory_player_.currentDuration() > 0)
    animating_path_ = false;
}

void VisualizeTrajectoryWidget::dropTrajectory() { drop_displaying_trajectory_ = true; }
void VisualizeTrajectoryWidget::onUpdate(float /*wall_dt*/)
{
  if (!env_->isInitialized() || !visualization_)
    return;

  if (drop_displaying_trajectory_)
  {
    animating_path_ = false;
    displaying_trajectory_.clear();
    trajectory_slider_panel_->update(0);
    drop_displaying_trajectory_ = false;
    trajectory_player_.reset();
  }

  if (!animating_path_)
  {  // finished last animation?

    boost::mutex::scoped_lock lock(update_trajectory_message_);
    // new trajectory available to display?
    if (!trajectory_to_display_.empty())
    {
      animating_path_ = true;

      if (display_mode_property_->getOptionInt() == 2)
        animating_path_ = false;

      displaying_trajectory_ = trajectory_to_display_;
      trajectory_player_.setTrajectory(displaying_trajectory_);

      if (trajectory_env_commands_.empty())
      {
        trajectory_state_solver_ = env_->getStateSolver();
      }
      else
      {
        auto env_cloned = env_->clone();
        env_cloned->applyCommands(trajectory_env_commands_);
        trajectory_state_solver_ = env_cloned->getStateSolver();
      }

      slider_count_ = static_cast<int>(std::ceil(trajectory_player_.trajectoryDuration() / SLIDER_RESOLUTION)) + 1;

      if (display_mode_property_->getOptionInt() == 2)
      {
        createTrajectoryTrail();
        visualization_->setTrajectoryVisible(true);
      }

      if (trajectory_slider_panel_)
        trajectory_slider_panel_->update(slider_count_);
    }
    else if (!displaying_trajectory_.empty())
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
          if (trajectory_player_.isFinished())
            animating_path_ = false;
          else
            animating_path_ = true;
        }
      }
      else
      {
        if (previous_display_mode_ != display_mode_property_->getOptionInt())
        {
          if (display_mode_property_->getOptionInt() == 2)
          {
            createTrajectoryTrail();
            visualization_->setTrajectoryVisible(true);
          }

          if (trajectory_slider_panel_)
            trajectory_slider_panel_->update(slider_count_);
        }

        animating_path_ = false;
      }
      previous_display_mode_ = display_mode_property_->getOptionInt();
    }
    trajectory_to_display_.clear();
    trajectory_env_commands_.clear();

    if (animating_path_)
    {
      trajectory_player_.reset();
      tesseract_common::JointState joint_state = trajectory_player_.setCurrentDuration(0);
      tesseract_environment::EnvState::Ptr state =
          trajectory_state_solver_->getState(joint_state.joint_names, joint_state.position);
      visualization_->setStartState(state->link_transforms);

      if (trajectory_slider_panel_)
        trajectory_slider_panel_->setSliderPosition(0);
    }
  }

  if (animating_path_)
  {
    if (trajectory_slider_panel_ != nullptr && trajectory_slider_panel_->isVisible() &&
        trajectory_slider_panel_->isPaused())
    {
      double duration = static_cast<double>(trajectory_slider_panel_->getSliderPosition()) * SLIDER_RESOLUTION;
      tesseract_common::JointState joint_state = trajectory_player_.setCurrentDuration(duration);
      tesseract_environment::EnvState::Ptr state =
          trajectory_state_solver_->getState(joint_state.joint_names, joint_state.position);
      visualization_->setStartState(state->link_transforms);
    }
    else
    {
      if (trajectory_player_.isFinished())
      {
        animating_path_ = false;  // animation finished
        if ((display_mode_property_->getOptionInt() != 1) && trajectory_slider_panel_)
          trajectory_slider_panel_->pauseButton(true);
      }
      else
      {
        tesseract_common::JointState joint_state = trajectory_player_.getNext();
        tesseract_environment::EnvState::Ptr state =
            trajectory_state_solver_->getState(joint_state.joint_names, joint_state.position);

        if (trajectory_slider_panel_ != nullptr)
        {
          int slider_index = static_cast<int>(std::ceil(trajectory_player_.currentDuration() / SLIDER_RESOLUTION));
          trajectory_slider_panel_->setSliderPosition(slider_index);
        }

        visualization_->setStartState(state->link_transforms);
      }
    }
  }
}

void VisualizeTrajectoryWidget::setDisplayTrajectory(const tesseract_msgs::Trajectory::ConstPtr& msg)
{
  // Error check
  if (!env_->isInitialized())
  {
    ROS_ERROR_STREAM_NAMED("VisualizeTrajectoryWidget", "No environment");
    return;
  }

  if (visualization_)
  {
    if (!visualization_->isStartStateVisible())
      visualization_->setStartStateVisible(true);
  }

  if (msg->environment.command_history.size() > 0)
  {
    tesseract_environment::Environment::Ptr env = tesseract_rosutils::fromMsg(msg->environment);
    if (env)
      setEnvironment(env);
  }

  trajectory_env_commands_.clear();
  if (!msg->commands.empty())
    trajectory_env_commands_ = tesseract_rosutils::fromMsg(msg->commands);

  if (!msg->instructions.empty())
  {
    using namespace tesseract_planning;
    Instruction program = Serialization::fromArchiveStringXML<Instruction>(msg->instructions);
    boost::mutex::scoped_lock lock(update_trajectory_message_);
    const auto& ci = program.as<CompositeInstruction>();
    trajectory_to_display_ = toJointTrajectory(ci);
    if (interrupt_display_property_->getBool())
      interruptCurrentDisplay();
  }
  else if (!msg->joint_trajectory.empty())
  {
    boost::mutex::scoped_lock lock(update_trajectory_message_);
    trajectory_to_display_ = tesseract_rosutils::fromMsg(msg->joint_trajectory);
    if (interrupt_display_property_->getBool())
      interruptCurrentDisplay();
  }
  else
  {
    trajectory_to_display_.clear();
  }
}

void VisualizeTrajectoryWidget::setEnvironment(tesseract_environment::Environment::Ptr env)
{
  onDisable();
  visualization_->initialize(true, true, true, true);
  env_ = std::move(env);
  visualization_->addSceneGraph(*(env_->getSceneGraph()));  // required?
  visualization_->setVisible(true);
  onEnable();
}

void VisualizeTrajectoryWidget::trajectorySliderPanelVisibilityChange(bool enable)
{
  if (!trajectory_slider_panel_)
    return;

  if (enable)
    trajectory_slider_panel_->onEnable();
  else
    trajectory_slider_panel_->onDisable();
}

}  // namespace tesseract_rviz
