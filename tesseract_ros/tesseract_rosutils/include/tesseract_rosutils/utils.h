/**
 * @file utils.h
 * @brief Tesseract ROS utility functions.
 *
 * @author Levi Armstrong
 * @date April 15, 2018
 * @version TODO
 * @bug No known bugs
 *
 * @copyright Copyright (c) 2018, Southwest Research Institute
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
#ifndef TESSERACT_ROSUTILS_UTILS_H
#define TESSERACT_ROSUTILS_UTILS_H

#include <tesseract_common/macros.h>
TESSERACT_COMMON_IGNORE_WARNINGS_PUSH
#include <trajectory_msgs/JointTrajectory.h>
#include <tesseract_msgs/TesseractState.h>
#include <tesseract_msgs/EnvironmentCommand.h>
#include <tesseract_msgs/ContactResultVector.h>
#include <tesseract_msgs/Mesh.h>
#include <tesseract_msgs/Link.h>
#include <tesseract_msgs/Geometry.h>
#include <tesseract_msgs/Material.h>
#include <tesseract_msgs/Inertial.h>
#include <tesseract_msgs/VisualGeometry.h>
#include <tesseract_msgs/CollisionGeometry.h>
#include <tesseract_msgs/Joint.h>
#include <tesseract_msgs/JointCalibration.h>
#include <tesseract_msgs/JointDynamics.h>
#include <tesseract_msgs/JointLimits.h>
#include <tesseract_msgs/JointMimic.h>
#include <tesseract_msgs/JointSafety.h>
#include <tesseract_msgs/ProcessPlan.h>
#include <tesseract_msgs/AllowedCollisionEntry.h>

#include <geometry_msgs/Pose.h>
#include <geometry_msgs/PoseArray.h>

#include <tesseract_environment/core/environment.h>
#include <tesseract_scene_graph/resource_locator.h>

#include <Eigen/Geometry>
TESSERACT_COMMON_IGNORE_WARNINGS_POP

#include <tesseract_scene_graph/link.h>
#include <tesseract_geometry/geometries.h>
#include <tesseract_collision/core/common.h>
#include <tesseract_common/types.h>
#include <tesseract_scene_graph/resource_locator.h>

namespace tesseract_rosutils
{
std::string locateResource(const std::string& url);

class ROSResourceLocator : public tesseract_scene_graph::SimpleResourceLocator
{
public:
  ROSResourceLocator();
};

bool isMsgEmpty(const sensor_msgs::JointState& msg);

bool isMsgEmpty(const sensor_msgs::MultiDOFJointState& msg);

bool isIdentical(const tesseract_geometry::Geometry& shape1, const tesseract_geometry::Geometry& shape2);

bool isIdentical(const tesseract_scene_graph::Visual& visual1, const tesseract_scene_graph::Visual& visual2);

bool isIdentical(const tesseract_scene_graph::Collision& collision1,
                 const tesseract_scene_graph::Collision& collision2);

bool isIdentical(const tesseract_scene_graph::Link& link1, const tesseract_scene_graph::Link& link2);

/** \brief Construct the message that corresponds to the shape. Return false on failure. */
bool toMsg(tesseract_msgs::Geometry& geometry_msgs, const tesseract_geometry::Geometry& geometry);

bool fromMsg(tesseract_geometry::Geometry::Ptr& geometry, const tesseract_msgs::Geometry& geometry_msg);

bool toMsg(tesseract_msgs::Material& material_msg, const tesseract_scene_graph::Material::Ptr& material);

bool fromMsg(tesseract_scene_graph::Material::Ptr& material, const tesseract_msgs::Material& material_msg);

bool toMsg(tesseract_msgs::Inertial& inertial_msg, const tesseract_scene_graph::Inertial::Ptr& inertial);

bool fromMsg(tesseract_scene_graph::Inertial::Ptr& inertial, const tesseract_msgs::Inertial& inertial_msg);

bool toMsg(tesseract_msgs::VisualGeometry& visual_msg, const tesseract_scene_graph::Visual& visual);

bool fromMsg(tesseract_scene_graph::Visual::Ptr& visual, const tesseract_msgs::VisualGeometry& visual_msg);

bool toMsg(tesseract_msgs::CollisionGeometry& collision_msg, const tesseract_scene_graph::Collision& collision);

bool fromMsg(tesseract_scene_graph::Collision::Ptr& collision, const tesseract_msgs::CollisionGeometry& collision_msg);

bool toMsg(tesseract_msgs::Link& link_msg, const tesseract_scene_graph::Link& link);

tesseract_scene_graph::Link fromMsg(const tesseract_msgs::Link& link_msg);

bool toMsg(tesseract_msgs::JointCalibration& joint_calibration_msg,
           const tesseract_scene_graph::JointCalibration::Ptr& joint_calibration);

bool fromMsg(tesseract_scene_graph::JointCalibration::Ptr& joint_calibration,
             const tesseract_msgs::JointCalibration& joint_calibration_msg);

bool toMsg(tesseract_msgs::JointDynamics& joint_dynamics_msg,
           const tesseract_scene_graph::JointDynamics::Ptr& joint_dynamics);

bool fromMsg(tesseract_scene_graph::JointDynamics::Ptr& joint_dynamics,
             const tesseract_msgs::JointDynamics& joint_dynamics_msg);

bool toMsg(tesseract_msgs::JointLimits& joint_limits_msg, const tesseract_scene_graph::JointLimits::Ptr& joint_limits);

bool fromMsg(tesseract_scene_graph::JointLimits::Ptr& joint_limits,
             const tesseract_msgs::JointLimits& joint_limits_msg);

bool toMsg(tesseract_msgs::JointMimic& joint_mimic_msg, const tesseract_scene_graph::JointMimic::Ptr& joint_mimic);

bool fromMsg(tesseract_scene_graph::JointMimic::Ptr& joint_mimic, const tesseract_msgs::JointMimic& joint_mimic_msg);

bool toMsg(tesseract_msgs::JointSafety& joint_safety_msg, const tesseract_scene_graph::JointSafety::Ptr& joint_safety);

bool fromMsg(tesseract_scene_graph::JointSafety::Ptr& joint_safety,
             const tesseract_msgs::JointSafety& joint_safety_msg);

bool toMsg(tesseract_msgs::Joint& joint_msg, const tesseract_scene_graph::Joint& joint);

tesseract_scene_graph::Joint fromMsg(const tesseract_msgs::Joint& joint_msg);

/**
 * @brief Convert allowed collision matrix to a vector of allowed collision entry messages
 * @param acm_msg Vector of allowed collision entries to populate
 * @param acm Allowed collision matrix to convert to message
 * @return True if successful, otherwise false
 */
bool toMsg(std::vector<tesseract_msgs::AllowedCollisionEntry>& acm_msg,
           const tesseract_scene_graph::AllowedCollisionMatrix& acm);

void toMsg(tesseract_msgs::SceneGraph& scene_graph_msg, const tesseract_scene_graph::SceneGraph& scene_graph);

tesseract_scene_graph::SceneGraph fromMsg(const tesseract_msgs::SceneGraph& scene_graph_msg);

void toMsg(sensor_msgs::JointState& joint_state, const tesseract_environment::EnvState& state);

bool toMsg(tesseract_msgs::EnvironmentCommand& command_msg, const tesseract_environment::Command& command);

bool toMsg(std::vector<tesseract_msgs::EnvironmentCommand>& commands_msg,
           const tesseract_environment::Commands& commands,
           const unsigned long past_revision);

void toMsg(const sensor_msgs::JointStatePtr& joint_state, const tesseract_environment::EnvState& state);

void toMsg(tesseract_msgs::TesseractState& state_msg, const tesseract_environment::Environment& env);

void toMsg(const tesseract_msgs::TesseractStatePtr& state_msg, const tesseract_environment::Environment& env);

/**
 * @brief Generate a JointTrajectory Message that contains all joints in the environment
 * @param traj_msg The output JointTrajectory Message
 * @param start_state The Environment start/current state
 * @param joint_names The joint names corresponding to the trajectory
 * @param traj The joint trajectory
 */
void toMsg(trajectory_msgs::JointTrajectory& traj_msg,
           const tesseract_environment::EnvState& start_state,
           const std::vector<std::string>& joint_names,
           const Eigen::Ref<const tesseract_common::TrajArray>& traj);

/**
 * @brief Generate a JointTrajectory Message that contains all joints in the environment
 * @param traj_msg The output JointTrajectory Message
 * @param start_state The Environment start/current state
 * @param joint_names The joint names corresponding to the trajectory
 * @param traj The joint trajectory
 */
void toMsg(const trajectory_msgs::JointTrajectoryPtr& traj_msg,
           const tesseract_environment::EnvState& start_state,
           const std::vector<std::string>& joint_names,
           const Eigen::Ref<const tesseract_common::TrajArray>& traj);

/**
 * @brief Generate a JointTrajectory Message that contains only trajectory joints
 * @param traj_msg The output JointTrajectory Message
 * @param joint_names The joint names corresponding to the trajectory
 * @param traj The joint trajectory
 */
void toMsg(trajectory_msgs::JointTrajectory& traj_msg, const tesseract_common::JointTrajectory& traj);

/**
 * @brief Generate a JointTrajectory Message that contains only trajectory joints
 * @param traj_msg The output JointTrajectory Message
 * @param joint_names The joint names corresponding to the trajectory
 * @param traj The joint trajectory
 */
void toMsg(const trajectory_msgs::JointTrajectoryPtr& traj_msg, const tesseract_common::JointTrajectory& traj);

bool processMsg(tesseract_environment::Environment& env, const sensor_msgs::JointState& joint_state_msg);

bool processMsg(tesseract_environment::Environment& env,
                const std::vector<tesseract_msgs::EnvironmentCommand>& env_command_msg);

bool processMsg(const tesseract_environment::Environment::Ptr& env, const sensor_msgs::JointState& joint_state_msg);

bool processMsg(tesseract_environment::Environment& env, const tesseract_msgs::TesseractState& state_msg);

bool processMsg(const tesseract_environment::Environment::Ptr& env, const tesseract_msgs::TesseractState& state_msg);

/**
 * @brief Convert Geometry Pose Message to Eigen
 * @param pose Eigen type to filled out
 * @param pose_msg The message to be converted
 * @return True if successful, otherwise false
 */
bool fromMsg(Eigen::Isometry3d& pose, const geometry_msgs::Pose& pose_msg);

/**
 * @brief Convert Eigen to Geometry Pose Message
 * @param pose_msg Geometry Pose Message to filled out
 * @param pose The Eigen type to be converted
 * @return True if successful, otherwise false
 */
bool toMsg(geometry_msgs::Pose& pose_msg, const Eigen::Isometry3d& pose);

void toMsg(tesseract_msgs::ContactResult& contact_result_msg,
           const tesseract_collision::ContactResult& contact_result,
           const ros::Time& stamp = ros::Time::now());

void toMsg(const tesseract_msgs::ContactResultPtr& contact_result_msg,
           const tesseract_collision::ContactResult& contact_result,
           const ros::Time& stamp = ros::Time::now());

/**
 * @brief Convert a vector of Eigen::Isometry3d into a pose array
 * @param Pose Array
 * @param transforms A vector of transforms
 * @return True if successful, otherwise false
 */
bool toMsg(geometry_msgs::PoseArray& pose_array, const tesseract_common::VectorIsometry3d& transforms);

}  // namespace tesseract_rosutils

#endif
