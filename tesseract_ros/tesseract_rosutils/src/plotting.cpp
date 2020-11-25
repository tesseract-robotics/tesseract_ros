/**
 * @file plotting.cpp
 * @brief Tesseract ROS plotting functions.
 *
 * @author Levi Armstrong
 * @date April 15, 2018
 * @version TODO
 * @bug No known bugs
 *
 * @copyright Copyright (c) 2013, Southwest Research Institute
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
#include <tesseract_common/macros.h>
TESSERACT_COMMON_IGNORE_WARNINGS_PUSH
#include <ros/console.h>
#include <Eigen/Geometry>
#include <ros/ros.h>
#include <geometry_msgs/PoseArray.h>
TESSERACT_COMMON_IGNORE_WARNINGS_POP

#include <tesseract_rosutils/plotting.h>
#include <tesseract_rosutils/utils.h>
#include <tesseract_rosutils/conversions.h>

#include <tesseract_command_language/utils/flatten_utils.h>
#include <tesseract_environment/core/environment.h>
#include <tesseract_command_language/command_language.h>

namespace tesseract_rosutils
{
ROSPlotting::ROSPlotting(std::string root_link, std::string topic_namespace)
  : root_link_(root_link), topic_namespace_(topic_namespace)
{
  ros::NodeHandle nh;

  trajectory_pub_ =
      nh.advertise<tesseract_msgs::Trajectory>(topic_namespace + "/display_tesseract_trajectory", 1, true);
  collisions_pub_ = nh.advertise<visualization_msgs::MarkerArray>(topic_namespace + "/display_collisions", 1, true);
  arrows_pub_ = nh.advertise<visualization_msgs::MarkerArray>(topic_namespace + "/display_arrows", 1, true);
  axes_pub_ = nh.advertise<visualization_msgs::MarkerArray>(topic_namespace + "/display_axes", 1, true);
  tool_path_pub_ = nh.advertise<geometry_msgs::PoseArray>(topic_namespace + "/display_tool_path", 1, true);
}

bool ROSPlotting::init(tesseract::Tesseract::ConstPtr thor)
{
  tesseract_ = thor;
  return true;
}

bool ROSPlotting::isConnected() const { return true; }

void ROSPlotting::waitForConnection(long seconds) const
{
  const ros::WallTime start_time = ros::WallTime::now();
  const ros::WallDuration wall_timeout{ static_cast<double>(seconds) };
  while (ros::ok())
  {
    if (isConnected())
      return;

    if (wall_timeout >= ros::WallDuration(0))
    {
      const ros::WallTime current_time = ros::WallTime::now();
      if ((current_time - start_time) >= wall_timeout)
        return;
    }

    ros::WallDuration(0.02).sleep();
    ros::spinOnce();
  }

  return;
}

void ROSPlotting::plotEnvironment(tesseract_environment::Environment::ConstPtr /*env*/) {}

void ROSPlotting::plotEnvironmentState(tesseract_environment::EnvState::ConstPtr /*state*/) {}

void ROSPlotting::plotTrajectory(const tesseract_msgs::Trajectory& traj)
{
  trajectory_pub_.publish(traj);
  ros::spinOnce();
}

void ROSPlotting::plotTrajectory(const std::vector<std::string>& joint_names,
                                 const Eigen::Ref<const tesseract_common::TrajArray>& traj)
{
  tesseract_msgs::Trajectory msg;
  tesseract_common::JointTrajectory trajectory;
  trajectory.joint_names = joint_names;
  trajectory.trajectory = traj;

  // Set the joint trajectory message
  toMsg(msg.joint_trajectory, trajectory);

  plotTrajectory(msg);
}

void ROSPlotting::plotTrajectory(const tesseract_common::JointTrajectory& traj)
{
  tesseract_msgs::Trajectory msg;

  // Set the joint trajectory message
  toMsg(msg.joint_trajectory, traj);

  plotTrajectory(msg);
}

void ROSPlotting::plotTrajectory(const tesseract_planning::Instruction& instruction)
{
  tesseract_msgs::Trajectory msg;

  // Set tesseract state information
  if (tesseract_ != nullptr)
    toMsg(msg.tesseract_state, *(tesseract_->getEnvironment()));

  // Set the joint trajectory message
  toJointTrajectory(msg.joint_trajectory, instruction);

  plotTrajectory(msg);
}

void ROSPlotting::plotTrajectory(const std::vector<std::string>& joint_names,
                                 const Eigen::Ref<const tesseract_common::TrajArray>& traj,
                                 const tesseract_environment::Environment::ConstPtr& env)
{
  tesseract_msgs::Trajectory msg;

  // Set tesseract state information
  toMsg(msg.tesseract_state, *env);

  // Set the joint trajectory message
  toMsg(msg.joint_trajectory, *(env->getCurrentState()), joint_names, traj);

  plotTrajectory(msg);
}

void ROSPlotting::plotToolPath(const tesseract_planning::Instruction& instruction)
{
  using namespace tesseract_planning;

  if (tesseract_ == nullptr)
  {
    ROS_ERROR("plotToolPath: requires that the plotter be initialized with a tesseract!");
    return;
  }

  geometry_msgs::PoseArray tool_path;
  tool_path.header.frame_id = tesseract_->getEnvironment()->getRootLinkName();
  tesseract_environment::StateSolver::Ptr state_solver = tesseract_->getEnvironment()->getStateSolver();
  if (isCompositeInstruction(instruction))
  {
    const auto* ci = instruction.cast_const<CompositeInstruction>();

    // Assume all the plan instructions have the same manipulator as the composite
    assert(!ci->getManipulatorInfo().empty());
    const ManipulatorInfo& composite_mi = ci->getManipulatorInfo();

    auto composite_mi_fwd_kin =
        tesseract_->getEnvironment()->getManipulatorManager()->getFwdKinematicSolver(composite_mi.manipulator);
    if (composite_mi_fwd_kin == nullptr)
    {
      ROS_ERROR_STREAM("plotToolPath: Manipulator: " << composite_mi.manipulator << " does not exist!");
      return;
    }
    const std::string& tip_link = composite_mi_fwd_kin->getTipLinkName();

    std::vector<std::reference_wrapper<const Instruction>> fi = tesseract_planning::flatten(*ci, planFilter);
    if (fi.empty())
      fi = tesseract_planning::flatten(*ci, moveFilter);

    for (const auto& i : fi)
    {
      ManipulatorInfo manip_info;

      // Check for updated manipulator information and get waypoint
      Waypoint wp = NullWaypoint();
      if (isPlanInstruction(i.get()))
      {
        const auto* pi = i.get().cast_const<PlanInstruction>();
        manip_info = composite_mi.getCombined(pi->getManipulatorInfo());
        wp = pi->getWaypoint();
      }
      else if (isMoveInstruction(i.get()))
      {
        const auto* mi = i.get().cast_const<MoveInstruction>();
        manip_info = composite_mi.getCombined(mi->getManipulatorInfo());
        wp = mi->getWaypoint();
      }

      // Extract TCP
      Eigen::Isometry3d tcp = tesseract_->findTCP(manip_info);

      if (isStateWaypoint(wp))
      {
        const auto* swp = wp.cast_const<StateWaypoint>();
        assert(static_cast<long>(swp->joint_names.size()) == swp->position.size());
        tesseract_environment::EnvState::Ptr state = state_solver->getState(swp->joint_names, swp->position);
        geometry_msgs::Pose p;
        tesseract_rosutils::toMsg(p, state->link_transforms[tip_link] * tcp);
        tool_path.poses.push_back(p);
      }
      else if (isJointWaypoint(wp))
      {
        const auto* jwp = wp.cast_const<JointWaypoint>();
        assert(static_cast<long>(jwp->joint_names.size()) == jwp->size());
        tesseract_environment::EnvState::Ptr state = state_solver->getState(jwp->joint_names, *jwp);
        geometry_msgs::Pose p;
        tesseract_rosutils::toMsg(p, state->link_transforms[tip_link] * tcp);
        tool_path.poses.push_back(p);
      }
      else if (isCartesianWaypoint(wp))
      {
        const auto* cwp = wp.cast_const<CartesianWaypoint>();
        if (manip_info.working_frame.empty())
        {
          geometry_msgs::Pose p;
          tesseract_rosutils::toMsg(p, (*cwp));
          tool_path.poses.push_back(p);
        }
        else
        {
          tesseract_environment::EnvState::ConstPtr state = tesseract_->getEnvironment()->getCurrentState();
          geometry_msgs::Pose p;
          tesseract_rosutils::toMsg(p, state->link_transforms.at(manip_info.working_frame) * (*cwp));
          tool_path.poses.push_back(p);
        }
      }
      else
      {
        ROS_ERROR("plotTrajectoy: Unsupported Waypoint Type!");
      }
    }
  }
  else if (isPlanInstruction(instruction))
  {
    assert(isPlanInstruction(instruction));
    const auto* pi = instruction.cast_const<PlanInstruction>();

    // Assume all the plan instructions have the same manipulator as the composite
    assert(!pi->getManipulatorInfo().empty());
    const ManipulatorInfo& composite_mi = pi->getManipulatorInfo();
    ManipulatorInfo manip_info = composite_mi.getCombined(pi->getManipulatorInfo());

    auto composite_mi_fwd_kin =
        tesseract_->getEnvironment()->getManipulatorManager()->getFwdKinematicSolver(manip_info.manipulator);
    if (composite_mi_fwd_kin == nullptr)
    {
      ROS_ERROR_STREAM("plotToolPath: Manipulator: " << manip_info.manipulator << " does not exist!");
      return;
    }
    const std::string& tip_link = composite_mi_fwd_kin->getTipLinkName();

    // Extract TCP
    Eigen::Isometry3d tcp = tesseract_->findTCP(manip_info);

    if (isStateWaypoint(pi->getWaypoint()))
    {
      const auto* swp = pi->getWaypoint().cast_const<StateWaypoint>();
      assert(static_cast<long>(swp->joint_names.size()) == swp->position.size());
      tesseract_environment::EnvState::Ptr state = state_solver->getState(swp->joint_names, swp->position);
      geometry_msgs::Pose p;
      tesseract_rosutils::toMsg(p, state->link_transforms[tip_link] * tcp);
      tool_path.poses.push_back(p);
    }
    else if (isJointWaypoint(pi->getWaypoint()))
    {
      const auto* jwp = pi->getWaypoint().cast_const<JointWaypoint>();
      assert(static_cast<long>(jwp->joint_names.size()) == jwp->size());
      tesseract_environment::EnvState::Ptr state = state_solver->getState(jwp->joint_names, *jwp);
      geometry_msgs::Pose p;
      tesseract_rosutils::toMsg(p, state->link_transforms[tip_link] * tcp);
      tool_path.poses.push_back(p);
    }
    else if (isCartesianWaypoint(pi->getWaypoint()))
    {
      const auto* cwp = pi->getWaypoint().cast_const<CartesianWaypoint>();
      if (manip_info.working_frame.empty())
      {
        geometry_msgs::Pose p;
        tesseract_rosutils::toMsg(p, (*cwp));
        tool_path.poses.push_back(p);
      }
      else
      {
        tesseract_environment::EnvState::ConstPtr state = tesseract_->getEnvironment()->getCurrentState();
        geometry_msgs::Pose p;
        tesseract_rosutils::toMsg(p, state->link_transforms.at(manip_info.working_frame) * (*cwp));
        tool_path.poses.push_back(p);
      }
    }
    else
    {
      ROS_ERROR("plotTrajectoy: Unsupported Waypoint Type!");
    }
  }
  else
  {
    ROS_ERROR("plotTrajectoy: Unsupported Instruction Type!");
  }

  tool_path_pub_.publish(tool_path);
  ros::spinOnce();
}

void ROSPlotting::plotContactResults(const std::vector<std::string>& link_names,
                                     const tesseract_collision::ContactResultVector& dist_results,
                                     const Eigen::Ref<const Eigen::VectorXd>& safety_distances)
{
  visualization_msgs::MarkerArray msg = getContactResultsMarkerArrayMsg(
      marker_counter_, root_link_, topic_namespace_, ros::Time::now(), link_names, dist_results, safety_distances);
  if (!dist_results.empty())
  {
    collisions_pub_.publish(msg);
    ros::spinOnce();
  }
}

void ROSPlotting::plotArrow(const Eigen::Ref<const Eigen::Vector3d>& pt1,
                            const Eigen::Ref<const Eigen::Vector3d>& pt2,
                            const Eigen::Ref<const Eigen::Vector4d>& rgba,
                            double scale)
{
  visualization_msgs::MarkerArray msg;
  auto marker =
      getMarkerArrowMsg(marker_counter_, root_link_, topic_namespace_, ros::Time::now(), pt1, pt2, rgba, scale);
  msg.markers.push_back(marker);
  arrows_pub_.publish(msg);
  ros::spinOnce();
}

void ROSPlotting::plotAxis(const Eigen::Isometry3d& axis, double scale)
{
  visualization_msgs::MarkerArray msg;
  Eigen::Vector3d x_axis = axis.matrix().block<3, 1>(0, 0);
  Eigen::Vector3d y_axis = axis.matrix().block<3, 1>(0, 1);
  Eigen::Vector3d z_axis = axis.matrix().block<3, 1>(0, 2);
  Eigen::Vector3d position = axis.matrix().block<3, 1>(0, 3);

  auto marker = getMarkerCylinderMsg(marker_counter_,
                                     root_link_,
                                     topic_namespace_,
                                     ros::Time::now(),
                                     position,
                                     position + x_axis,
                                     Eigen::Vector4d(1, 0, 0, 1),
                                     scale);
  msg.markers.push_back(marker);

  marker = getMarkerCylinderMsg(marker_counter_,
                                root_link_,
                                topic_namespace_,
                                ros::Time::now(),
                                position,
                                position + y_axis,
                                Eigen::Vector4d(0, 1, 0, 1),
                                scale);
  msg.markers.push_back(marker);

  marker = getMarkerCylinderMsg(marker_counter_,
                                root_link_,
                                topic_namespace_,
                                ros::Time::now(),
                                position,
                                position + z_axis,
                                Eigen::Vector4d(0, 0, 1, 1),
                                scale);
  msg.markers.push_back(marker);

  axes_pub_.publish(msg);
}

void ROSPlotting::clear()
{
  // Remove old arrows
  marker_counter_ = 0;
  visualization_msgs::MarkerArray msg;
  visualization_msgs::Marker marker;
  marker.header.frame_id = root_link_;
  marker.header.stamp = ros::Time();
  marker.ns = topic_namespace_;
  marker.id = 0;
  marker.type = visualization_msgs::Marker::ARROW;
  marker.action = visualization_msgs::Marker::DELETEALL;
  msg.markers.push_back(marker);
  collisions_pub_.publish(msg);
  arrows_pub_.publish(msg);
  axes_pub_.publish(msg);
  ros::spinOnce();
}

void ROSPlotting::waitForInput()
{
  ROS_ERROR("Hit enter key to step optimization!");
  std::cin.ignore(std::numeric_limits<std::streamsize>::max(), '\n');
}

const std::string& ROSPlotting::getRootLink() const { return root_link_; }

visualization_msgs::Marker ROSPlotting::getMarkerArrowMsg(int& id_counter,
                                                          const std::string& frame_id,
                                                          const std::string& ns,
                                                          const ros::Time& time_stamp,
                                                          const Eigen::Ref<const Eigen::Vector3d>& pt1,
                                                          const Eigen::Ref<const Eigen::Vector3d>& pt2,
                                                          const Eigen::Ref<const Eigen::Vector4d>& rgba,
                                                          double scale)
{
  visualization_msgs::Marker marker;
  marker.header.frame_id = frame_id;
  marker.header.stamp = time_stamp;
  marker.ns = ns;
  marker.id = ++id_counter;
  marker.type = visualization_msgs::Marker::ARROW;
  marker.action = visualization_msgs::Marker::ADD;

  Eigen::Vector3d x, y, z;
  x = (pt2 - pt1).normalized();
  marker.pose.position.x = pt1(0);
  marker.pose.position.y = pt1(1);
  marker.pose.position.z = pt1(2);

  y = x.unitOrthogonal();
  z = (x.cross(y)).normalized();
  Eigen::Matrix3d rot;
  rot.col(0) = x;
  rot.col(1) = y;
  rot.col(2) = z;
  Eigen::Quaterniond q(rot);
  q.normalize();
  marker.pose.orientation.x = q.x();
  marker.pose.orientation.y = q.y();
  marker.pose.orientation.z = q.z();
  marker.pose.orientation.w = q.w();

  marker.scale.x = std::abs((pt2 - pt1).norm());
  marker.scale.y = scale;
  marker.scale.z = scale;

  marker.color.r = static_cast<float>(rgba(0));
  marker.color.g = static_cast<float>(rgba(1));
  marker.color.b = static_cast<float>(rgba(2));
  marker.color.a = static_cast<float>(rgba(3));

  return marker;
}

visualization_msgs::Marker ROSPlotting::getMarkerCylinderMsg(int& id_counter,
                                                             const std::string& frame_id,
                                                             const std::string& ns,
                                                             const ros::Time& time_stamp,
                                                             const Eigen::Ref<const Eigen::Vector3d>& pt1,
                                                             const Eigen::Ref<const Eigen::Vector3d>& pt2,
                                                             const Eigen::Ref<const Eigen::Vector4d>& rgba,
                                                             double scale)
{
  visualization_msgs::Marker marker;
  marker.header.frame_id = frame_id;
  marker.header.stamp = time_stamp;
  marker.ns = ns;
  marker.id = ++id_counter;
  marker.type = visualization_msgs::Marker::CYLINDER;
  marker.action = visualization_msgs::Marker::ADD;

  double length = scale * std::abs((pt2 - pt1).norm());
  Eigen::Vector3d x, y, z, center;
  z = (pt2 - pt1).normalized();
  center = pt1 + (length / 2.0) * z;
  marker.pose.position.x = center(0);
  marker.pose.position.y = center(1);
  marker.pose.position.z = center(2);

  y = z.unitOrthogonal();
  x = (y.cross(z)).normalized();
  Eigen::Matrix3d rot;
  rot.col(0) = x;
  rot.col(1) = y;
  rot.col(2) = z;
  Eigen::Quaterniond q(rot);
  q.normalize();
  marker.pose.orientation.x = q.x();
  marker.pose.orientation.y = q.y();
  marker.pose.orientation.z = q.z();
  marker.pose.orientation.w = q.w();

  marker.scale.x = length / 20.0;
  marker.scale.y = length / 20.0;
  marker.scale.z = length;

  marker.color.r = static_cast<float>(rgba(0));
  marker.color.g = static_cast<float>(rgba(1));
  marker.color.b = static_cast<float>(rgba(2));
  marker.color.a = static_cast<float>(rgba(3));

  return marker;
}

visualization_msgs::MarkerArray
ROSPlotting::getContactResultsMarkerArrayMsg(int& id_counter,
                                             const std::string& frame_id,
                                             const std::string& ns,
                                             const ros::Time& time_stamp,
                                             const std::vector<std::string>& link_names,
                                             const tesseract_collision::ContactResultVector& dist_results,
                                             const Eigen::Ref<const Eigen::VectorXd>& safety_distances)
{
  visualization_msgs::MarkerArray msg;
  for (unsigned i = 0; i < dist_results.size(); ++i)
  {
    const tesseract_collision::ContactResult& dist = dist_results[i];
    const double& safety_distance = safety_distances[i];

    Eigen::Vector4d rgba;
    if (dist.distance < 0)
    {
      rgba << 1.0, 0.0, 0.0, 1.0;
    }
    else if (dist.distance < safety_distance)
    {
      rgba << 1.0, 1.0, 0.0, 1.0;
    }
    else
    {
      rgba << 0.0, 1.0, 0.0, 1.0;
    }

    if (dist.cc_type[0] == tesseract_collision::ContinuousCollisionType::CCType_Between)
    {
      Eigen::Vector4d cc_rgba;
      cc_rgba << 0.0, 0.0, 1.0, 1.0;
      auto marker = getMarkerArrowMsg(id_counter,
                                      frame_id,
                                      ns,
                                      time_stamp,
                                      dist.transform[0] * dist.nearest_points_local[0],
                                      dist.cc_transform[0] * dist.nearest_points_local[0],
                                      cc_rgba,
                                      0.01);
      msg.markers.push_back(marker);
    }

    if (dist.cc_type[1] == tesseract_collision::ContinuousCollisionType::CCType_Between)
    {
      Eigen::Vector4d cc_rgba;
      cc_rgba << 0.0, 0.0, 0.5, 1.0;
      auto marker = getMarkerArrowMsg(id_counter,
                                      frame_id,
                                      ns,
                                      time_stamp,
                                      dist.transform[1] * dist.nearest_points_local[1],
                                      dist.cc_transform[1] * dist.nearest_points_local[1],
                                      cc_rgba,
                                      0.01);
      msg.markers.push_back(marker);
    }

    auto it0 = std::find(link_names.begin(), link_names.end(), dist.link_names[0]);
    auto it1 = std::find(link_names.begin(), link_names.end(), dist.link_names[1]);

    if (it0 != link_names.end() && it1 != link_names.end())
    {
      auto marker0 = getMarkerArrowMsg(
          id_counter, frame_id, ns, time_stamp, dist.nearest_points[0], dist.nearest_points[1], rgba, 0.01);
      msg.markers.push_back(marker0);

      auto marker1 = getMarkerArrowMsg(
          id_counter, frame_id, ns, time_stamp, dist.nearest_points[1], dist.nearest_points[0], rgba, 0.01);
      msg.markers.push_back(marker1);
    }
    else if (it0 != link_names.end())
    {
      auto marker = getMarkerArrowMsg(
          id_counter, frame_id, ns, time_stamp, dist.nearest_points[1], dist.nearest_points[0], rgba, 0.01);
      msg.markers.push_back(marker);
    }
    else
    {
      auto marker = getMarkerArrowMsg(
          id_counter, frame_id, ns, time_stamp, dist.nearest_points[0], dist.nearest_points[1], rgba, 0.01);
      msg.markers.push_back(marker);
    }
  }

  return msg;
}

}  // namespace tesseract_rosutils
