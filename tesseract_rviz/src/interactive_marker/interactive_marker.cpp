/*
 * Copyright (c) 2011, Willow Garage, Inc.
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the distribution.
 *     * Neither the name of the Willow Garage, Inc. nor the names of its
 *       contributors may be used to endorse or promote products derived from
 *       this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */

#include <boost/make_shared.hpp>

#include <QMenu>

#include <OgreSceneNode.h>
#include <OgreSceneManager.h>
#include <OgreMaterialManager.h>
#include <OgreResourceGroupManager.h>
#include <OgreSubEntity.h>
#include <OgreMath.h>
#include <OgreRenderWindow.h>

#include <ros/ros.h>

#include "rviz/frame_manager.h"
#include "rviz/display_context.h"
#include "rviz/selection/selection_manager.h"
#include "rviz/frame_manager.h"
#include "rviz/render_panel.h"
#include "rviz/geometry.h"
#include "rviz/validate_quaternions.h"

#include <tesseract_rviz/interactive_marker/integer_action.h>
#include <tesseract_rviz/interactive_marker/interactive_marker.h>
#include <tesseract_rviz/markers/utils.h>
#include <tesseract_rviz/conversions.h>

namespace tesseract_rviz
{
InteractiveMarker::InteractiveMarker(std::string name,
                                     std::string description,
                                     Ogre::SceneNode* scene_node,
                                     rviz::DisplayContext* context,
                                     float scale)
  : visible_(true)
  , context_(context)
  , reference_node_(scene_node->createChildSceneNode())
  , position_(scene_node->getPosition())
  , orientation_(scene_node->getOrientation())
  , pose_changed_(false)
  , time_since_last_feedback_(0)
  , name_(std::move(name))
  , description_(std::move(description))
  , dragging_(false)
  , pose_update_requested_(false)
  , scale_(scale)
  , show_visual_aids_(false)
  , show_axes_(false)
  , show_description_(false)
{
  axes_ = new rviz::Axes(context->getSceneManager(), reference_node_, 1, 0.05f);

  axes_->setPosition(position_);
  axes_->setOrientation(orientation_);
  axes_->set(scale_, scale_ * 0.05f);
  setShowAxes(show_axes_);

  //  has_menu_ = message.menu_entries.size() > 0;

  description_control_ = boost::make_shared<InteractiveMarkerControl>(name_,
                                                                      description_,
                                                                      context_,
                                                                      reference_node_,
                                                                      this,
                                                                      InteractiveMode::NONE,
                                                                      OrientationMode::VIEW_FACING,
                                                                      true,
                                                                      Ogre::Quaternion());
  makeTitle(*description_control_, description_);

  // create menu
  menu_entries_.clear();
  menu_.reset();
  if (has_menu_)
  {
    menu_.reset(new QMenu());
    top_level_menu_ids_.clear();

    // Put all menu entries into the menu_entries_ map and create the
    // tree of menu entry ids.
    //    for ( unsigned m=0; m < message.menu_entries.size(); m++ )
    //    {
    //      const visualization_msgs::MenuEntry& entry = message.menu_entries[ m ];
    //      MenuNode node;
    //      node.entry = entry;
    //      menu_entries_[ entry.id ] = node;
    //      if( entry.parent_id == 0 )
    //      {
    //        top_level_menu_ids_.push_back( entry.id );
    //      }
    //      else
    //      {
    //        // Find the parent node and add this entry to the parent's list of children.
    //        std::map< uint32_t, MenuNode >::iterator parent_it = menu_entries_.find( entry.parent_id );
    //        if( parent_it == menu_entries_.end() ) {
    //          ROS_ERROR("interactive marker menu entry %u found before its parent id %u.  Ignoring.", entry.id,
    //          entry.parent_id);
    //        }
    //        else
    //        {
    //          (*parent_it).second.child_ids.push_back( entry.id );
    //        }
    //      }
    //    }
    populateMenu(menu_.get(), top_level_menu_ids_);
  }
}

InteractiveMarker::~InteractiveMarker()
{
  delete axes_;
  context_->getSceneManager()->destroySceneNode(reference_node_);
}

InteractiveMarkerControl::Ptr InteractiveMarker::createInteractiveControl(const std::string& name,
                                                                          const std::string& description,
                                                                          const InteractiveMode interactive_mode,
                                                                          const OrientationMode orientation_mode,
                                                                          const bool always_visible,
                                                                          const Ogre::Quaternion& orientation)
{
  auto search_iter = controls_.find(name);

  // If message->name in map,
  if (search_iter != controls_.end())
  {
    // Use existing control
    return search_iter->second;
  }

  // Else make new control
  auto control = boost::make_shared<InteractiveMarkerControl>(name,
                                                              description,
                                                              context_,
                                                              reference_node_,
                                                              this,
                                                              interactive_mode,
                                                              orientation_mode,
                                                              always_visible,
                                                              orientation);
  controls_[name] = control;
  return control;
}

// Recursively append menu and submenu entries to menu, based on a
// vector of menu entry id numbers describing the menu entries at the
// current level.
void InteractiveMarker::populateMenu(QMenu* /*menu*/, std::vector<uint32_t>& ids)
{
  for (unsigned int id : ids)
  {
    auto node_it = menu_entries_.find(id);
    ROS_ASSERT_MSG(
        node_it != menu_entries_.end(), "interactive marker menu entry %u not found during populateMenu().", id);
    MenuNode node = (*node_it).second;

    //    if ( node.child_ids.empty() )
    //    {
    //      IntegerAction* action = new IntegerAction( makeMenuString( node.entry.title ),
    //                                                 menu,
    //                                                 (int) node.entry.id );
    //      connect( action, SIGNAL( triggered( int )), this, SLOT( handleMenuSelect( int )));
    //      menu->addAction( action );
    //    }
    //    else
    //    {
    //      // make sub-menu
    //      QMenu* sub_menu = menu->addMenu( makeMenuString( node.entry.title ));
    //      populateMenu( sub_menu, node.child_ids );
    //    }
  }
}

QString InteractiveMarker::makeMenuString(const std::string& entry)
{
  QString menu_entry;
  if (entry.find("[x]") == 0)
  {
    menu_entry = QChar(0x2611) + QString::fromStdString(entry.substr(3));
  }
  else if (entry.find("[ ]") == 0)
  {
    menu_entry = QChar(0x2610) + QString::fromStdString(entry.substr(3));
  }
  else
  {
    menu_entry = QChar(0x3000) + QString::fromStdString(entry);
  }
  return menu_entry;
}

void InteractiveMarker::setVisible(bool visible)
{
  visible_ = visible;
  reference_node_->setVisible(visible_);

  for (auto& control_pair : controls_)
    control_pair.second->setVisible(visible_);

  updateDescriptionVisibility();
  updateAxesVisibility();
  updateVisualAidsVisibility();

  //  if (!show_menu_ && visible)
}

void InteractiveMarker::update(float wall_dt)
{
  boost::recursive_mutex::scoped_lock lock(mutex_);
  time_since_last_feedback_ += wall_dt;

  for (auto& control : controls_)
  {
    control.second->update();
  }
  if (description_control_)
  {
    description_control_->update();
  }

  if (dragging_)
  {
    if (pose_changed_)
    {
      publishPose();
    }
    else if (time_since_last_feedback_ > 0.25)
    {
      publishFeedback();
    }
  }
}

void InteractiveMarker::publishPose()
{
  boost::recursive_mutex::scoped_lock lock(mutex_);
  publishFeedback();
  pose_changed_ = false;
}

void InteractiveMarker::requestPoseUpdate(Ogre::Vector3 position, Ogre::Quaternion orientation)
{
  boost::recursive_mutex::scoped_lock lock(mutex_);
  if (dragging_)
  {
    pose_update_requested_ = true;
    requested_position_ = position;
    requested_orientation_ = orientation;
  }
  else
  {
    setPose(position, orientation, "");
  }
}

void InteractiveMarker::setPose(Ogre::Vector3 position, Ogre::Quaternion orientation, const std::string& control_name)
{
  boost::recursive_mutex::scoped_lock lock(mutex_);
  position_ = position;
  orientation_ = orientation;
  pose_changed_ = true;
  last_control_name_ = control_name;

  axes_->setPosition(position_);
  axes_->setOrientation(orientation_);

  for (auto& control : controls_)
  {
    control.second->interactiveMarkerPoseChanged(position_, orientation_);
  }
  if (description_control_)
  {
    description_control_->interactiveMarkerPoseChanged(position_, orientation_);
  }
}

void InteractiveMarker::setSize(float scale)
{
  scale_ = scale;
  for (auto& control : controls_)
    control.second->updateSize();
}

void InteractiveMarker::setShowDescription(bool show)
{
  show_description_ = show;
  updateDescriptionVisibility();
}

void InteractiveMarker::setShowAxes(bool show)
{
  show_axes_ = show;
  updateAxesVisibility();
}

void InteractiveMarker::setShowVisualAids(bool show)
{
  show_visual_aids_ = show;
  updateVisualAidsVisibility();
}

void InteractiveMarker::updateDescriptionVisibility()
{
  boost::recursive_mutex::scoped_lock lock(mutex_);
  if (description_control_.get())
  {
    description_control_->setVisible(visible_ && show_description_);
  }
}

void InteractiveMarker::updateAxesVisibility()
{
  boost::recursive_mutex::scoped_lock lock(mutex_);
  axes_->getSceneNode()->setVisible(visible_ && show_axes_);
}

void InteractiveMarker::updateVisualAidsVisibility()
{
  boost::recursive_mutex::scoped_lock lock(mutex_);
  for (auto& control : controls_)
  {
    control.second->setShowVisualAids(visible_ && show_visual_aids_);
  }
}

void InteractiveMarker::translate(Ogre::Vector3 delta_position, const std::string& control_name)
{
  boost::recursive_mutex::scoped_lock lock(mutex_);
  setPose(position_ + delta_position, orientation_, control_name);
}

void InteractiveMarker::rotate(Ogre::Quaternion delta_orientation, const std::string& control_name)
{
  boost::recursive_mutex::scoped_lock lock(mutex_);
  setPose(position_, delta_orientation * orientation_, control_name);
}

void InteractiveMarker::startDragging()
{
  boost::recursive_mutex::scoped_lock lock(mutex_);
  dragging_ = true;
  pose_changed_ = false;
}

void InteractiveMarker::stopDragging()
{
  boost::recursive_mutex::scoped_lock lock(mutex_);
  dragging_ = false;
  if (pose_update_requested_)
  {
    setPose(requested_position_, requested_orientation_, "");
    pose_update_requested_ = false;
  }
}

bool InteractiveMarker::handle3DCursorEvent(rviz::ViewportMouseEvent& event,
                                            const Ogre::Vector3& cursor_pos,
                                            const Ogre::Quaternion& /*cursor_rot*/,
                                            const std::string& control_name)
{
  boost::recursive_mutex::scoped_lock lock(mutex_);

  if (event.acting_button == Qt::LeftButton)
  {
    Ogre::Vector3 point_rel_world = cursor_pos;
    bool got_3D_point = true;

    publishFeedback(got_3D_point, point_rel_world);
  }

  if (!dragging_ && menu_.get())
  {
    // Event.right() will be false during a right-button-up event.  We
    // want to swallow (with the "return true") all other
    // right-button-related mouse events.
    if (event.right())
    {
      return true;
    }
    if (event.rightUp() && event.buttons_down == Qt::NoButton)
    {
      // Save the 3D mouse point to send with the menu feedback, if any.
      Ogre::Vector3 three_d_point = cursor_pos;
      bool valid_point = true;
      Ogre::Vector2 mouse_pos = rviz::project3DPointToViewportXY(event.viewport, cursor_pos);
      QCursor::setPos(event.panel->mapToGlobal(QPoint(static_cast<int>(mouse_pos.x), static_cast<int>(mouse_pos.y))));
      showMenu(event, control_name, three_d_point, valid_point);
      return true;
    }
  }

  return false;
}

bool InteractiveMarker::handleMouseEvent(rviz::ViewportMouseEvent& event, const std::string& control_name)
{
  boost::recursive_mutex::scoped_lock lock(mutex_);

  if (event.acting_button == Qt::LeftButton)
  {
    Ogre::Vector3 point_rel_world;
    bool got_3D_point = context_->getSelectionManager()->get3DPoint(event.viewport, event.x, event.y, point_rel_world);

    publishFeedback(got_3D_point, point_rel_world);
  }

  if (!dragging_ && menu_.get())
  {
    // Event.right() will be false during a right-button-up event.  We
    // want to swallow (with the "return true") all other
    // right-button-related mouse events.
    if (event.right())
    {
      return true;
    }
    if (event.rightUp() && event.buttons_down == Qt::NoButton)
    {
      // Save the 3D mouse point to send with the menu feedback, if any.
      Ogre::Vector3 three_d_point;
      bool valid_point = context_->getSelectionManager()->get3DPoint(event.viewport, event.x, event.y, three_d_point);
      showMenu(event, control_name, three_d_point, valid_point);
      return true;
    }
  }

  return false;
}

void InteractiveMarker::showMenu(rviz::ViewportMouseEvent& event,
                                 const std::string& control_name,
                                 const Ogre::Vector3& three_d_point,
                                 bool valid_point)
{
  // Save the 3D mouse point to send with the menu feedback, if any.
  got_3d_point_for_menu_ = valid_point;
  three_d_point_for_menu_ = three_d_point;

  event.panel->showContextMenu(menu_);
  last_control_name_ = control_name;
}

void InteractiveMarker::handleMenuSelect(int menu_item_id)
{
  boost::recursive_mutex::scoped_lock lock(mutex_);

  auto it = menu_entries_.find(static_cast<unsigned>(menu_item_id));

  if (it != menu_entries_.end())
  {
    //    visualization_msgs::MenuEntry& entry = it->second.entry;

    //    std::string command = entry.command;
    //    uint8_t command_type = entry.command_type;

    //    if ( command_type == visualization_msgs::MenuEntry::FEEDBACK )
    //    {
    //      visualization_msgs::InteractiveMarkerFeedback feedback;
    //      feedback.event_type = visualization_msgs::InteractiveMarkerFeedback::MENU_SELECT;
    //      feedback.menu_entry_id = entry.id;
    //      feedback.control_name = last_control_name_;
    //      publishFeedback( feedback, got_3d_point_for_menu_, three_d_point_for_menu_ );
    //    }
    //    else if ( command_type == visualization_msgs::MenuEntry::ROSRUN )
    //    {
    //      std::string sys_cmd = "rosrun " + command;
    //      ROS_INFO_STREAM( "Running system command: " << sys_cmd );
    //      sys_thread_ = boost::shared_ptr<boost::thread>( new boost::thread( boost::bind( &system, sys_cmd.c_str() ) )
    //      );
    //      //system( sys_cmd.c_str() );
    //    }
    //    else if ( command_type == visualization_msgs::MenuEntry::ROSLAUNCH )
    //    {
    //      std::string sys_cmd = "roslaunch " + command;
    //      ROS_INFO_STREAM( "Running system command: " << sys_cmd );
    //      sys_thread_ = boost::shared_ptr<boost::thread>( new boost::thread( boost::bind( &system, sys_cmd.c_str() ) )
    //      );
    //      //system( sys_cmd.c_str() );
    //    }
  }
}

void InteractiveMarker::publishFeedback(bool mouse_point_valid, const Ogre::Vector3& mouse_point_rel_world)
{
  boost::recursive_mutex::scoped_lock lock(mutex_);
  Eigen::Isometry3d transform;
  Eigen::Vector3d mouse_point;

  std::string frame_name = context_->getFixedFrame().toStdString();

  Ogre::Vector3 world_position = reference_node_->convertLocalToWorldPosition(position_);
  Ogre::Quaternion world_orientation = reference_node_->convertLocalToWorldOrientation(orientation_);

  toEigen(transform, world_position, world_orientation);

  if (mouse_point_valid)
  {
    mouse_point = Eigen::Vector3d(mouse_point_rel_world.x, mouse_point_rel_world.y, mouse_point_rel_world.z);
  }

  Q_EMIT userFeedback(frame_name, transform, mouse_point, mouse_point_valid);

  time_since_last_feedback_ = 0;
}

}  // namespace tesseract_rviz
