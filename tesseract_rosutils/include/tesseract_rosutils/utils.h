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
#include <Eigen/Geometry>
TESSERACT_COMMON_IGNORE_WARNINGS_POP

#include <tesseract_common/resource_locator.h>
#include <tesseract_common/eigen_types.h>
#include <tesseract_common/collision_margin_data.h>
#include <tesseract_srdf/kinematics_information.h>

#include <tesseract_common/fwd.h>
#include <tesseract_geometry/fwd.h>
#include <tesseract_collision/core/fwd.h>
#include <tesseract_scene_graph/fwd.h>
#include <tesseract_environment/fwd.h>
#include <tesseract_collision/core/fwd.h>
#include <tesseract_motion_planners/core/fwd.h>
#include <tesseract_task_composer/core/fwd.h>
#include <tesseract_msgs/TransformMap.h>

#include <ros/message_forward.h>
namespace tesseract_msgs
{
ROS_DECLARE_MESSAGE(AllowedCollisionEntry)
ROS_DECLARE_MESSAGE(ChainGroup)
ROS_DECLARE_MESSAGE(CollisionGeometry)
ROS_DECLARE_MESSAGE(ContactMarginPair)
ROS_DECLARE_MESSAGE(CollisionMarginData)
ROS_DECLARE_MESSAGE(CollisionMarginPairOverrideType)
ROS_DECLARE_MESSAGE(ContactResult)
ROS_DECLARE_MESSAGE(ContactResultVector)
ROS_DECLARE_MESSAGE(ContactManagersPluginInfo)
ROS_DECLARE_MESSAGE(EnvironmentCommand)
ROS_DECLARE_MESSAGE(Geometry)
ROS_DECLARE_MESSAGE(GroupsJointState)
ROS_DECLARE_MESSAGE(GroupsJointStates)
ROS_DECLARE_MESSAGE(GroupsKinematicPlugins)
ROS_DECLARE_MESSAGE(GroupsTCP)
ROS_DECLARE_MESSAGE(GroupsTCPs)
ROS_DECLARE_MESSAGE(Inertial)
ROS_DECLARE_MESSAGE(Joint)
ROS_DECLARE_MESSAGE(JointCalibration)
ROS_DECLARE_MESSAGE(JointDynamics)
ROS_DECLARE_MESSAGE(JointGroup)
ROS_DECLARE_MESSAGE(JointLimits)
ROS_DECLARE_MESSAGE(JointMimic)
ROS_DECLARE_MESSAGE(JointSafety)
ROS_DECLARE_MESSAGE(JointState)
ROS_DECLARE_MESSAGE(KinematicsInformation)
ROS_DECLARE_MESSAGE(Link)
ROS_DECLARE_MESSAGE(LinkGroup)
ROS_DECLARE_MESSAGE(Material)
ROS_DECLARE_MESSAGE(Mesh)
ROS_DECLARE_MESSAGE(SceneGraph)
ROS_DECLARE_MESSAGE(StringDoublePair)
ROS_DECLARE_MESSAGE(StringPair)
ROS_DECLARE_MESSAGE(Environment)
ROS_DECLARE_MESSAGE(EnvironmentState)
ROS_DECLARE_MESSAGE(Trajectory)
ROS_DECLARE_MESSAGE(VisualGeometry)
ROS_DECLARE_MESSAGE(JointTrajectory)
ROS_DECLARE_MESSAGE(PluginInfo)
ROS_DECLARE_MESSAGE(KinematicsPluginInfo)
ROS_DECLARE_MESSAGE(StringPluginInfoPair)
ROS_DECLARE_MESSAGE(TaskComposerNodeInfo)
}  // namespace tesseract_msgs

namespace trajectory_msgs
{
ROS_DECLARE_MESSAGE(JointTrajectory)
}

namespace geometry_msgs
{
ROS_DECLARE_MESSAGE(Pose)
ROS_DECLARE_MESSAGE(PoseArray)
}  // namespace geometry_msgs

namespace sensor_msgs
{
ROS_DECLARE_MESSAGE(JointState)
}

namespace ros
{
class Time;
}

namespace tesseract_rosutils
{
bool isMsgEmpty(const sensor_msgs::JointState& msg);
bool isMsgEmpty(const tesseract_msgs::TransformMap& msg);

bool isIdentical(const tesseract_geometry::Geometry& shape1, const tesseract_geometry::Geometry& shape2);

bool isIdentical(const tesseract_scene_graph::Visual& visual1, const tesseract_scene_graph::Visual& visual2);

bool isIdentical(const tesseract_scene_graph::Collision& collision1,
                 const tesseract_scene_graph::Collision& collision2);

bool isIdentical(const tesseract_scene_graph::Link& link1, const tesseract_scene_graph::Link& link2);

/** \brief Construct the message that corresponds to the shape. Return false on failure. */
bool toMsg(tesseract_msgs::Geometry& geometry_msgs, const tesseract_geometry::Geometry& geometry);

bool fromMsg(std::shared_ptr<tesseract_geometry::Geometry>& geometry, const tesseract_msgs::Geometry& geometry_msg);

bool toMsg(tesseract_msgs::Material& material_msg, const std::shared_ptr<tesseract_scene_graph::Material>& material);

bool fromMsg(std::shared_ptr<tesseract_scene_graph::Material>& material, const tesseract_msgs::Material& material_msg);

bool toMsg(tesseract_msgs::Inertial& inertial_msg, const std::shared_ptr<tesseract_scene_graph::Inertial>& inertial);

bool fromMsg(std::shared_ptr<tesseract_scene_graph::Inertial>& inertial, const tesseract_msgs::Inertial& inertial_msg);

bool toMsg(tesseract_msgs::VisualGeometry& visual_msg, const tesseract_scene_graph::Visual& visual);

bool fromMsg(std::shared_ptr<tesseract_scene_graph::Visual>& visual, const tesseract_msgs::VisualGeometry& visual_msg);

bool toMsg(tesseract_msgs::CollisionGeometry& collision_msg, const tesseract_scene_graph::Collision& collision);

bool fromMsg(std::shared_ptr<tesseract_scene_graph::Collision>& collision,
             const tesseract_msgs::CollisionGeometry& collision_msg);

bool toMsg(tesseract_msgs::Link& link_msg, const tesseract_scene_graph::Link& link);

tesseract_scene_graph::Link fromMsg(const tesseract_msgs::Link& link_msg);

bool toMsg(tesseract_msgs::JointCalibration& joint_calibration_msg,
           const std::shared_ptr<tesseract_scene_graph::JointCalibration>& joint_calibration);

bool fromMsg(std::shared_ptr<tesseract_scene_graph::JointCalibration>& joint_calibration,
             const tesseract_msgs::JointCalibration& joint_calibration_msg);

bool toMsg(tesseract_msgs::JointDynamics& joint_dynamics_msg,
           const std::shared_ptr<tesseract_scene_graph::JointDynamics>& joint_dynamics);

bool fromMsg(std::shared_ptr<tesseract_scene_graph::JointDynamics>& joint_dynamics,
             const tesseract_msgs::JointDynamics& joint_dynamics_msg);

bool toMsg(tesseract_msgs::JointLimits& joint_limits_msg,
           const std::shared_ptr<tesseract_scene_graph::JointLimits>& joint_limits);

bool fromMsg(std::shared_ptr<tesseract_scene_graph::JointLimits>& joint_limits,
             const tesseract_msgs::JointLimits& joint_limits_msg);

bool toMsg(tesseract_msgs::JointMimic& joint_mimic_msg,
           const std::shared_ptr<tesseract_scene_graph::JointMimic>& joint_mimic);

bool fromMsg(std::shared_ptr<tesseract_scene_graph::JointMimic>& joint_mimic,
             const tesseract_msgs::JointMimic& joint_mimic_msg);

bool toMsg(tesseract_msgs::JointSafety& joint_safety_msg,
           const std::shared_ptr<tesseract_scene_graph::JointSafety>& joint_safety);

bool fromMsg(std::shared_ptr<tesseract_scene_graph::JointSafety>& joint_safety,
             const tesseract_msgs::JointSafety& joint_safety_msg);

bool toMsg(tesseract_msgs::Joint& joint_msg, const tesseract_scene_graph::Joint& joint);

tesseract_common::PairsCollisionMarginData
fromMsg(const std::vector<tesseract_msgs::ContactMarginPair>& contact_margin_pairs_msg);
std::vector<tesseract_msgs::ContactMarginPair>
toMsg(const tesseract_common::PairsCollisionMarginData& contact_margin_pairs);

tesseract_common::CollisionMarginData fromMsg(const tesseract_msgs::CollisionMarginData& contact_margin_data_msg);
tesseract_msgs::CollisionMarginData toMsg(const tesseract_common::CollisionMarginData& contact_margin_data);

tesseract_common::CollisionMarginPairOverrideType
fromMsg(const tesseract_msgs::CollisionMarginPairOverrideType& contact_margin_pair_override_type_msg);
tesseract_msgs::CollisionMarginPairOverrideType
toMsg(const tesseract_common::CollisionMarginPairOverrideType& contact_margin_pair_override_type);

tesseract_scene_graph::Joint fromMsg(const tesseract_msgs::Joint& joint_msg);

/**
 * @brief Convert allowed collision matrix to a vector of allowed collision entry messages
 * @param acm_msg Vector of allowed collision entries to populate
 * @param acm Allowed collision matrix to convert to message
 * @return True if successful, otherwise false
 */
bool toMsg(std::vector<tesseract_msgs::AllowedCollisionEntry>& acm_msg,
           const tesseract_common::AllowedCollisionMatrix& acm);

void toMsg(tesseract_msgs::SceneGraph& scene_graph_msg, const tesseract_scene_graph::SceneGraph& scene_graph);

tesseract_scene_graph::SceneGraph fromMsg(const tesseract_msgs::SceneGraph& scene_graph_msg);

bool toMsg(tesseract_msgs::EnvironmentCommand& command_msg, const tesseract_environment::Command& command);

std::vector<std::shared_ptr<const tesseract_environment::Command>>
fromMsg(const std::vector<tesseract_msgs::EnvironmentCommand>& commands_msg);

std::shared_ptr<tesseract_environment::Command> fromMsg(const tesseract_msgs::EnvironmentCommand& command_msg);

bool toMsg(std::vector<tesseract_msgs::EnvironmentCommand>& commands_msg,
           const std::vector<std::shared_ptr<const tesseract_environment::Command>>& commands,
           unsigned long past_revision);

void toMsg(tesseract_msgs::EnvironmentState& state_msg,
           const tesseract_environment::Environment& env,
           bool include_joint_states = true);

void toMsg(const tesseract_msgs::EnvironmentStatePtr& state_msg, const tesseract_environment::Environment& env);

/**
 * @brief Generate a JointTrajectory Message that contains only trajectory joints
 * @param traj_msg The output JointTrajectory Message
 * @param joint_names The joint names corresponding to the trajectory
 * @param traj The joint trajectory
 */
void toMsg(tesseract_msgs::JointTrajectory& traj_msg, const tesseract_common::JointTrajectory& traj);

/**
 * @brief Generate a JointTrajectory from message
 * @param traj_msg The trajectory message to convert
 * @param traj The joint trajectory
 */
tesseract_common::JointTrajectory fromMsg(const tesseract_msgs::JointTrajectory& traj_msg);

bool processMsg(tesseract_environment::Environment& env,
                const sensor_msgs::JointState& joint_state_msg,
                const tesseract_msgs::TransformMap& floating_joint_state_msg = {});

/**
 * @brief Apply the provided commands to the environment
 * @param env The environment to apply the commands
 * @param env_command_msg The commands to apply
 * @return True if successful, otherwise false
 */
bool processMsg(tesseract_environment::Environment& env,
                const std::vector<tesseract_msgs::EnvironmentCommand>& env_command_msg);

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

void toMsg(tesseract_msgs::ContactResult& contact_result_msg, const tesseract_collision::ContactResult& contact_result);

void toMsg(tesseract_msgs::ContactResult& contact_result_msg,
           const tesseract_collision::ContactResult& contact_result,
           const ros::Time& stamp);

void toMsg(const tesseract_msgs::ContactResultPtr& contact_result_msg,
           const tesseract_collision::ContactResult& contact_result);

void toMsg(const tesseract_msgs::ContactResultPtr& contact_result_msg,
           const tesseract_collision::ContactResult& contact_result,
           const ros::Time& stamp);

/**
 * @brief Convert kinematics plugin info to message
 * @param info Kinematics plugin info
 * @return Kinematics plugin info
 */
tesseract_msgs::KinematicsPluginInfo toMsg(const tesseract_common::KinematicsPluginInfo& info);

/**
 * @brief Convert contact managers plugin info to message
 * @param info contact managers plugin info
 * @return contact managers plugin info
 */
tesseract_msgs::ContactManagersPluginInfo toMsg(const tesseract_common::ContactManagersPluginInfo& info);

/**
 * @brief Convert plugin info map to message
 * @param info_map plugin info map
 * @return plugin info map
 */
std::vector<tesseract_msgs::StringPluginInfoPair>
toMsg(const std::map<std::string, tesseract_common::PluginInfo>& info_map);

/**
 * @brief Convert plugin info to message
 * @param info plugin info
 * @return plugin info
 */
tesseract_msgs::PluginInfo toMsg(const tesseract_common::PluginInfo& info);

/**
 * @brief Convert a vector of Eigen::Isometry3d into a pose array
 * @param Pose Array
 * @param transforms A vector of transforms
 * @return True if successful, otherwise false
 */
bool toMsg(geometry_msgs::PoseArray& pose_array, const tesseract_common::VectorIsometry3d& transforms);

/**
 * @brief Convert a chain group to message
 * @param group Chain group
 * @return Chain group message
 */
tesseract_msgs::ChainGroup toMsg(tesseract_srdf::ChainGroups::const_reference group);

/**
 * @brief Convert a group's joint state to message
 * @param group Group's joint states
 * @return Group's joint states message
 */
tesseract_msgs::GroupsJointStates toMsg(tesseract_srdf::GroupJointStates::const_reference group);

/**
 * @brief Convert a group's tool center points to message
 * @param group Group's tool center points
 * @return Group's tool center points message
 */
tesseract_msgs::GroupsTCPs toMsg(tesseract_srdf::GroupTCPs::const_reference group);

/**
 * @brief Convert manipulator managers data to message
 * @param kin_info Kinematics information message to populate
 * @param manager The Manipulator manager to convert to message
 * @return True if successful, otherwise false
 */
bool toMsg(tesseract_msgs::KinematicsInformation& kin_info_msg, const tesseract_srdf::KinematicsInformation& kin_info);

/**
 * @brief This will populate the kinematics information from the kinematics information message
 * @param kin_info The kinematics information data structure to populate
 * @param kin_info_msg The kinematics information message
 * @return True if successful, otherwise false
 */
bool fromMsg(tesseract_srdf::KinematicsInformation& kin_info,
             const tesseract_msgs::KinematicsInformation& kin_info_msg);

/**
 * @brief Convert kinematics plugin info from message
 * @param info_msg Kinematics plugin info message
 * @return Kinematics plugin info
 */
tesseract_common::KinematicsPluginInfo fromMsg(const tesseract_msgs::KinematicsPluginInfo& info_msg);

/**
 * @brief Convert contact managers plugin info from message
 * @param info_msg Contact managers plugin info message
 * @return Contact managers plugin info
 */
tesseract_common::ContactManagersPluginInfo fromMsg(const tesseract_msgs::ContactManagersPluginInfo& info_msg);

/**
 * @brief Convert plugin info map from message
 * @param info_map plugin info map message
 * @return plugin info map
 */
std::map<std::string, tesseract_common::PluginInfo>
fromMsg(const std::vector<tesseract_msgs::StringPluginInfoPair>& info_map_msg);

/**
 * @brief Convert plugin info from message
 * @param info plugin info message
 * @return plugin info
 */
tesseract_common::PluginInfo fromMsg(const tesseract_msgs::PluginInfo& info_msg);

/**
 * @brief This will populate a transform map message
 * @param transform_map_msg The transform map message
 * @param transform_map The transform map
 * @return True if successful, otherwise false
 */
bool toMsg(tesseract_msgs::TransformMap& transform_map_msg, const tesseract_common::TransformMap& transform_map);

/**
 * @brief This will populate a transform map given a message
 * @param transform_map The transform map
 * @param transform_map_msg The transform map message
 * @return True if successful, otherwise false
 */
bool fromMsg(tesseract_common::TransformMap& transform_map, const tesseract_msgs::TransformMap& transform_map_msg);

/**
 * @brief This will populate a joint states map message
 * @param joint_state_msg The joint states map message
 * @param joint_state The joint state map
 * @return True if successful, otherwise false
 */
bool toMsg(sensor_msgs::JointState& joint_state_msg, const std::unordered_map<std::string, double>& joint_state);

/**
 * @brief This will populate a joint states from message
 * @param joint_state The joint state map
 * @param joint_state_msg The joint states map message
 * @return True if successful, otherwise false
 */
bool fromMsg(std::unordered_map<std::string, double>& joint_state, const sensor_msgs::JointState& joint_state_msg);

/**
 * @brief This will populate a joint states map message
 * @param joint_state_msg The joint states map message
 * @param joint_state The joint state map
 * @return True if successful, otherwise false
 */
bool toMsg(std::vector<tesseract_msgs::StringDoublePair>& joint_state_map_msg,
           const std::unordered_map<std::string, double>& joint_state);

/**
 * @brief This will populate a joint states from message
 * @param joint_state The joint state map
 * @param joint_state_map_msg The joint states map message
 * @return True if successful, otherwise false
 */
bool fromMsg(std::unordered_map<std::string, double>& joint_state,
             const std::vector<tesseract_msgs::StringDoublePair>& joint_state_map_msg);

/**
 * @brief Converts a Environment object to a Tesseract msg
 * @param tesseract_msg Resulting Message
 * @param env Input Environment object
 * @param include_joint_states If true, the joint_states element will be populated with the current state
 * @return True if successful, otherwise false
 */
bool toMsg(tesseract_msgs::Environment& environment_msg,
           const tesseract_environment::Environment& env,
           bool include_joint_states = true);

/**
 * @brief Converts a Environment object to a Tesseract msg
 * @param tesseract_msg Resulting Message
 * @param env Input Environment object
 * @param include_joint_states If true, the joint_states element will be populated with the current state
 * @return True if successful, otherwise false
 */
bool toMsg(tesseract_msgs::Environment& environment_msg,
           const std::shared_ptr<const tesseract_environment::Environment>& env,
           bool include_joint_states = true);

/**
 * @brief Converts a Tesseract msg to a Environment object
 * @param tesseract_msg Input Tesseract msg
 * @return Resulting Tesseract Object if successful, nullptr otherwise
 */
std::unique_ptr<tesseract_environment::Environment> fromMsg(const tesseract_msgs::Environment& environment_msg);

/**
 * @brief Converts a TaskInfo object to a TaskInfo msg
 * @param task_info_msg Resulting message
 * @param task_info TaskInfo object
 * @return True if successful, otherwise false
 */
bool toMsg(tesseract_msgs::TaskComposerNodeInfo& node_info_msg, tesseract_planning::TaskComposerNodeInfo& node_info);

/**
 * @brief Converts a TaskInfo msg to a TaskInfo object
 * @param task_info_msg Input TaskInfo msg
 * @return Resulting Tesseract Object if successful, nullptr otherwise
 */
std::unique_ptr<tesseract_planning::TaskComposerNodeInfo>
fromMsg(const tesseract_msgs::TaskComposerNodeInfo& node_info_msg);

/**
 * @brief Converts a tesseract_common::JointTrajectory msg to a trajectory_msgs::JointTrajectory object
 * @param joint_trajectory Input JointTrajectory msg
 * @return Resulting Tesseract
 */
trajectory_msgs::JointTrajectory toMsg(const tesseract_common::JointTrajectory& joint_trajectory,
                                       const tesseract_scene_graph::SceneState& initial_state);

/**
 * @brief Convert trajectory_msgs::JointTrajectory to Tesseract tesseract_common::JointTrajectory
 * @param joint_trajectory The trajectory to convert
 * @return A tesseract joint trajectory
 */
tesseract_common::JointTrajectory fromMsg(const trajectory_msgs::JointTrajectory& joint_trajectory_msg);

}  // namespace tesseract_rosutils

#endif
