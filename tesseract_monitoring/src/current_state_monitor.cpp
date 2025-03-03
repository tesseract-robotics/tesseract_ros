/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2011, Willow Garage, Inc.
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

/* Author: Ioan Sucan */

#include <tesseract_common/macros.h>
TESSERACT_COMMON_IGNORE_WARNINGS_PUSH
#include <ros/console.h>
#include <ros/node_handle.h>

#include <limits>
#include <utility>
#include <mutex>
#include <condition_variable>
#include <memory>
#include <functional>
#include <unordered_map>
#include <map>

#include <sensor_msgs/JointState.h>
#include <tf2_ros/transform_broadcaster.h>
#include <geometry_msgs/TransformStamped.h>
#include <tf2_eigen/tf2_eigen.h>
TESSERACT_COMMON_IGNORE_WARNINGS_POP

#include <tesseract_monitoring/current_state_monitor.h>

#include <tesseract_kinematics/core/joint_group.h>
#include <tesseract_scene_graph/scene_state.h>
#include <tesseract_environment/environment.h>

namespace tesseract_monitoring
{
struct CurrentStateMonitor::Implementation
{
  ros::NodeHandle nh;
  tesseract_environment::Environment::ConstPtr env;
  tesseract_scene_graph::SceneState env_state;
  int last_environment_revision;
  std::map<std::string, ros::Time> joint_time;
  bool state_monitor_started;
  bool copy_dynamics;  // Copy velocity and effort from joint_state
  ros::Time monitor_start_time;
  double error;
  ros::Subscriber joint_state_subscriber;
  tf2_ros::TransformBroadcaster tf_broadcaster;
  ros::Time current_state_time;
  ros::Time last_tf_update;
  bool publish_tf{ false };

  mutable std::mutex state_update_lock;
  mutable std::condition_variable state_update_condition;
  std::vector<JointStateUpdateCallback> update_callbacks;

  Implementation(const tesseract_environment::Environment::ConstPtr& env_, const ros::NodeHandle& nh_)
    : nh(nh_)
    , env(env_)
    , env_state(env_->getState())
    , last_environment_revision(this->env->getRevision())
    , state_monitor_started(false)
    , copy_dynamics(false)
    , error(std::numeric_limits<double>::epsilon())
  {
  }

  tesseract_scene_graph::SceneState getCurrentState() const
  {
    std::scoped_lock slock(state_update_lock);
    return env_state;
  }

  ros::Time getCurrentStateTime() const
  {
    std::scoped_lock slock(state_update_lock);
    return current_state_time;
  }

  std::pair<tesseract_scene_graph::SceneState, ros::Time> getCurrentStateAndTime() const
  {
    std::scoped_lock slock(state_update_lock);
    return std::make_pair(env_state, current_state_time);
  }

  std::unordered_map<std::string, double> getCurrentStateValues() const
  {
    std::scoped_lock slock(state_update_lock);
    return env_state.joints;
  }

  void startStateMonitor(const std::string& joint_states_topic, bool enable_publish_tf)
  {
    publish_tf = enable_publish_tf;
    if (!state_monitor_started && env)
    {
      joint_time.clear();
      if (joint_states_topic.empty())
        ROS_ERROR("The joint states topic cannot be an empty string");
      else
        joint_state_subscriber =
            nh.subscribe(joint_states_topic, 25, &CurrentStateMonitor::Implementation::jointStateCallback, this);
      state_monitor_started = true;
      monitor_start_time = ros::Time::now();
      ROS_DEBUG("Listening to joint states on topic '%s'", nh.resolveName(joint_states_topic).c_str());
    }
  }

  void stopStateMonitor()
  {
    if (state_monitor_started)
    {
      joint_state_subscriber.shutdown();
      ROS_DEBUG("No longer listening o joint states");
      state_monitor_started = false;
    }
  }

  std::string getMonitoredTopic() const
  {
    if (joint_state_subscriber)
      return joint_state_subscriber.getTopic();

    return "";
  }

  bool haveCompleteState() const
  {
    bool result = true;
    std::scoped_lock slock(state_update_lock);
    for (const auto& joint : env_state.joints)
      if (joint_time.find(joint.first) == joint_time.end())
      {
        if (!isPassiveOrMimicDOF(joint.first))
        {
          ROS_DEBUG("Joint variable '%s' has never been updated", joint.first.c_str());
          result = false;
        }
      }
    return result;
  }

  bool haveCompleteState(std::vector<std::string>& missing_joints) const
  {
    bool result = true;
    std::scoped_lock slock(state_update_lock);
    for (const auto& joint : env_state.joints)
    {
      if (joint_time.find(joint.first) == joint_time.end())
      {
        if (!isPassiveOrMimicDOF(joint.first))
        {
          ROS_DEBUG("Joint variable '%s' has never been updated", joint.first.c_str());
          missing_joints.push_back(joint.first);
          result = false;
        }
      }
    }
    return result;
  }

  bool haveCompleteState(const ros::Duration& age) const
  {
    bool result = true;
    ros::Time now = ros::Time::now();
    ros::Time old = now - age;
    std::scoped_lock slock(state_update_lock);
    for (const auto& joint : env_state.joints)
    {
      if (isPassiveOrMimicDOF(joint.first))
        continue;
      auto it = joint_time.find(joint.first);
      if (it == joint_time.end())
      {
        ROS_DEBUG("Joint variable '%s' has never been updated", joint.first.c_str());
        result = false;
      }
      else if (it->second < old)
      {
        ROS_DEBUG("Joint variable '%s' was last updated %0.3lf seconds ago "
                  "(older than the allowed %0.3lf seconds)",
                  joint.first.c_str(),
                  (now - it->second).toSec(),
                  age.toSec());
        result = false;
      }
    }
    return result;
  }

  bool haveCompleteState(const ros::Duration& age, std::vector<std::string>& missing_states) const
  {
    bool result = true;
    ros::Time now = ros::Time::now();
    ros::Time old = now - age;
    std::scoped_lock slock(state_update_lock);

    for (const auto& joint : env_state.joints)
    {
      if (isPassiveOrMimicDOF(joint.first))
        continue;
      auto it = joint_time.find(joint.first);
      if (it == joint_time.end())
      {
        ROS_DEBUG("Joint variable '%s' has never been updated", joint.first.c_str());
        missing_states.push_back(joint.first);
        result = false;
      }
      else if (it->second < old)
      {
        ROS_DEBUG("Joint variable '%s' was last updated %0.3lf seconds ago "
                  "(older than the allowed %0.3lf seconds)",
                  joint.first.c_str(),
                  (now - it->second).toSec(),
                  age.toSec());
        missing_states.push_back(joint.first);
        result = false;
      }
    }
    return result;
  }

  bool waitForCurrentState(ros::Time t, double wait_time) const
  {
    ros::WallTime start = ros::WallTime::now();
    ros::WallDuration elapsed(0, 0);
    ros::WallDuration timeout(wait_time);

    std::unique_lock slock(state_update_lock);
    while (current_state_time < t)
    {
      state_update_condition.wait_for(slock, std::chrono::nanoseconds((timeout - elapsed).toNSec()));
      elapsed = ros::WallTime::now() - start;
      if (elapsed > timeout)
        return false;
    }
    return true;
  }

  bool waitForCompleteState(double wait_time) const
  {
    double slept_time = 0.0;
    double sleep_step_s = std::min(0.05, wait_time / 10.0);
    ros::Duration sleep_step(sleep_step_s);
    while (!haveCompleteState() && slept_time < wait_time)
    {
      sleep_step.sleep();
      slept_time += sleep_step_s;
    }
    return haveCompleteState();
  }

  bool waitForCompleteState(const std::string& manip, double wait_time) const
  {
    if (waitForCompleteState(wait_time))
      return true;
    bool ok = true;

    // check to see if we have a fully known state for the joints we want to
    // record
    std::vector<std::string> missing_joints;
    if (!haveCompleteState(missing_joints))
    {
      tesseract_kinematics::JointGroup::ConstPtr jmg = env->getJointGroup(manip);
      if (jmg)
      {
        std::set<std::string> mj;
        mj.insert(missing_joints.begin(), missing_joints.end());
        const std::vector<std::string>& names = jmg->getJointNames();
        for (std::size_t i = 0; ok && i < names.size(); ++i)
          if (mj.find(names[i]) != mj.end())
            ok = false;
      }
      else
        ok = false;
    }
    return ok;
  }

  void jointStateCallback(const sensor_msgs::JointStateConstPtr& joint_state)
  {
    if (!env->isInitialized())
      return;

    if (joint_state->name.size() != joint_state->position.size())
    {
      ROS_ERROR_THROTTLE(1,
                         "State monitor received invalid joint state (number "
                         "of joint names does not match number of "
                         "positions)");
      return;
    }
    bool update = false;

    {
      std::scoped_lock slock(state_update_lock);
      auto lock = env->lockRead();
      // read the received values, and update their time stamps
      current_state_time = joint_state->header.stamp;
      if (last_environment_revision != env->getRevision())
      {
        env_state = env->getState();
        last_environment_revision = env->getRevision();
      }

      for (unsigned i = 0; i < joint_state->name.size(); ++i)
      {
        if (env_state.joints.find(joint_state->name[i]) != env_state.joints.end())
        {
          double diff = env_state.joints[joint_state->name[i]] - joint_state->position[i];
          if (std::fabs(diff) > std::numeric_limits<double>::epsilon())
          {
            env_state.joints[joint_state->name[i]] = joint_state->position[i];
            update = true;
          }
          joint_time[joint_state->name[i]] = joint_state->header.stamp;
        }
      }

      if (update)
        env_state = env->getState(env_state.joints);

      if (publish_tf)
      {
        std::string base_link = env->getRootLinkName();
        std::vector<geometry_msgs::TransformStamped> transforms;
        transforms.reserve(env_state.joints.size());
        for (const auto& pose : env_state.link_transforms)
        {
          if (pose.first != base_link)
          {
            geometry_msgs::TransformStamped tf = tf2::eigenToTransform(pose.second);
            tf.header.stamp = current_state_time;
            tf.header.frame_id = base_link;
            tf.child_frame_id = pose.first;
            transforms.push_back(tf);
          }
        }
        tf_broadcaster.sendTransform(transforms);
      }
    }

    // callbacks, if needed
    if (update)
      for (auto& update_callback : update_callbacks)
        update_callback(joint_state);

    // notify waitForCurrentState() *after* potential update callbacks
    state_update_condition.notify_all();
  }

  bool isPassiveOrMimicDOF(const std::string& /*dof*/) const
  {
    const auto& active_joints = env_->getActiveJointNames();
    auto passive = (std::find(active_joints.begin(), active_joints.end(), dof) == active_joints.end());
    auto mimic = env_->getJoint(dof)->mimic != nullptr;

    return false;
  }
};

CurrentStateMonitor::CurrentStateMonitor(const std::shared_ptr<const tesseract_environment::Environment>& env)
  : CurrentStateMonitor(env, ros::NodeHandle())
{
}

CurrentStateMonitor::CurrentStateMonitor(const std::shared_ptr<const tesseract_environment::Environment>& env,
                                         const ros::NodeHandle& nh)
  : impl_(std::make_unique<Implementation>(env, nh))
{
}

const tesseract_environment::Environment& CurrentStateMonitor::getEnvironment() const
{
  return *std::as_const(*impl_).env;
}

CurrentStateMonitor::~CurrentStateMonitor() { stopStateMonitor(); }
tesseract_scene_graph::SceneState CurrentStateMonitor::getCurrentState() const
{
  return std::as_const(*impl_).getCurrentState();
}
ros::Time CurrentStateMonitor::getCurrentStateTime() const { return std::as_const(*impl_).getCurrentStateTime(); }
std::pair<tesseract_scene_graph::SceneState, ros::Time> CurrentStateMonitor::getCurrentStateAndTime() const
{
  return std::as_const(*impl_).getCurrentStateAndTime();
}
std::unordered_map<std::string, double> CurrentStateMonitor::getCurrentStateValues() const
{
  return std::as_const(*impl_).getCurrentStateValues();
}

void CurrentStateMonitor::addUpdateCallback(const JointStateUpdateCallback& fn)
{
  if (fn)
    impl_->update_callbacks.push_back(fn);
}

const ros::Time& CurrentStateMonitor::getMonitorStartTime() const { return std::as_const(*impl_).monitor_start_time; }

void CurrentStateMonitor::clearUpdateCallbacks() { impl_->update_callbacks.clear(); }
void CurrentStateMonitor::startStateMonitor(const std::string& joint_states_topic, bool publish_tf)
{
  impl_->startStateMonitor(joint_states_topic, publish_tf);
}

bool CurrentStateMonitor::isActive() const { return std::as_const(*impl_).state_monitor_started; }
void CurrentStateMonitor::stopStateMonitor() { impl_->stopStateMonitor(); }

std::string CurrentStateMonitor::getMonitoredTopic() const { return std::as_const(*impl_).getMonitoredTopic(); }

bool CurrentStateMonitor::haveCompleteState() const { return std::as_const(*impl_).haveCompleteState(); }

bool CurrentStateMonitor::haveCompleteState(std::vector<std::string>& missing_joints) const
{
  return std::as_const(*impl_).haveCompleteState(missing_joints);
}

bool CurrentStateMonitor::haveCompleteState(const ros::Duration& age) const
{
  return std::as_const(*impl_).haveCompleteState(age);
}

bool CurrentStateMonitor::haveCompleteState(const ros::Duration& age, std::vector<std::string>& missing_states) const
{
  return std::as_const(*impl_).haveCompleteState(age, missing_states);
}

bool CurrentStateMonitor::waitForCurrentState(ros::Time t, double wait_time) const
{
  return std::as_const(*impl_).waitForCurrentState(t, wait_time);
}

bool CurrentStateMonitor::waitForCompleteState(double wait_time) const
{
  return std::as_const(*impl_).waitForCompleteState(wait_time);
}

bool CurrentStateMonitor::waitForCompleteState(const std::string& manip, double wait_time) const
{
  return std::as_const(*impl_).waitForCompleteState(manip, wait_time);
}

void CurrentStateMonitor::setBoundsError(double error) { impl_->error = (error > 0) ? error : -error; }

double CurrentStateMonitor::getBoundsError() const { return std::as_const(*impl_).error; }

void CurrentStateMonitor::enableCopyDynamics(bool enabled) { impl_->copy_dynamics = enabled; }

}  // namespace tesseract_monitoring
