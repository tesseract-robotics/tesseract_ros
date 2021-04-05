/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2015, University of Colorado, Boulder
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
 *   * Neither the name of the Univ of CO, Boulder nor the names of its
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

/* Author: Dave Coleman
   Desc:   Wraps a trajectory_visualization playback class for Rviz into a stand
   alone display
*/

#include <tesseract_common/macros.h>
TESSERACT_COMMON_IGNORE_WARNINGS_PUSH
#include <ros/package.h>
#include <tesseract_rosutils/utils.h>
#include <tesseract_rosutils/conversions.h>
TESSERACT_COMMON_IGNORE_WARNINGS_POP

#include <tesseract_msgs/PlanningResponseArchive.h>
#include <tesseract_rviz/planning_response_archive_plugin/planning_response_archive_display.h>
#include <tesseract_command_language/core/serialization.h>
#include <tesseract_command_language/command_language.h>
#include <tesseract_command_language/utils/utils.h>
#include <tesseract_msgs/Trajectory.h>

namespace tesseract_rviz
{
const std::string ARCHIVE_TOPIC_NAME = "/planning_response_archive";

PlanningResponseArchiveDisplay::PlanningResponseArchiveDisplay()
{
  env_ = std::make_shared<tesseract_environment::Environment>();
  visualize_trajectory_widget_ = std::make_shared<VisualizeTrajectoryWidget>(this, this);
}

PlanningResponseArchiveDisplay::~PlanningResponseArchiveDisplay() = default;
void PlanningResponseArchiveDisplay::onInitialize()
{
  Display::onInitialize();

  visualization_ = std::make_shared<VisualizationWidget>(scene_node_, context_, "Tesseract State", this);
  visualization_->setCurrentStateVisible(false);

  visualize_trajectory_widget_->onInitialize(visualization_, env_, context_);

  archive_topic_sub_ = nh_.subscribe(ARCHIVE_TOPIC_NAME, 1, &PlanningResponseArchiveDisplay::callback, this);

  visualization_->setVisible(false);
}

void PlanningResponseArchiveDisplay::callback(const tesseract_msgs::PlanningResponseArchiveConstPtr& msg)
{
  using namespace tesseract_rosutils;
  using namespace tesseract_planning;

  // Convert to objects
  tesseract_msgs::PlanningRequestArchive request_archive = msg->planning_request;
  auto env = fromMsg(request_archive.environment);
  tesseract_environment::Commands commands = fromMsg(request_archive.commands);
  env->applyCommands(commands);
  Instruction results = CompositeInstruction();
  if (!msg->results.empty())
    results = Serialization::fromArchiveStringXML<Instruction>(msg->results);

  // Get the current find tcp callbacks
  std::vector<tesseract_environment::FindTCPCallbackFn> env_cb;
  if (env_ != nullptr)
    env_cb = env_->getFindTCPCallbacks();

  // Load Environment
  env_.swap(env);
  for (auto& f : env_cb)
    env_->addFindTCPCallback(f);
  visualize_trajectory_widget_->setEnvironment(env_);

  // Convert TCL to tesseract_msgs::Trajectory
  tesseract_common::JointTrajectory traj =
      tesseract_planning::toJointTrajectory(results.as<tesseract_planning::CompositeInstruction>());
  auto traj_msg = boost::make_shared<tesseract_msgs::Trajectory>();
  tesseract_rosutils::toMsg(traj_msg->joint_trajectory, traj);

  // Manually call the callback
  visualize_trajectory_widget_->setDisplayTrajectory(traj_msg);
}

void PlanningResponseArchiveDisplay::reset()
{
  visualization_->clear();
  Display::reset();

  visualize_trajectory_widget_->onReset();
}

void PlanningResponseArchiveDisplay::onEnable()
{
  Display::onEnable();

  visualize_trajectory_widget_->onEnable();
}

void PlanningResponseArchiveDisplay::onDisable()
{
  visualize_trajectory_widget_->onDisable();

  Display::onDisable();
}

void PlanningResponseArchiveDisplay::update(float wall_dt, float ros_dt)
{
  Display::update(wall_dt, ros_dt);
  visualize_trajectory_widget_->onUpdate(wall_dt);
}

void PlanningResponseArchiveDisplay::setName(const QString& name)
{
  BoolProperty::setName(name);
  visualize_trajectory_widget_->onNameChange(name);
}

}  // namespace tesseract_rviz
