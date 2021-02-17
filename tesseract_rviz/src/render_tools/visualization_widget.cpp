/*
 * Copyright (c) 2008, Willow Garage, Inc.
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

#include "tesseract_rviz/render_tools/visualization_widget.h"
#include "tesseract_rviz/render_tools/joint_widget.h"
#include "tesseract_rviz/render_tools/link_widget.h"
#include "tesseract_scene_graph/allowed_collision_matrix.h"

#include <tesseract_common/macros.h>
TESSERACT_COMMON_IGNORE_WARNINGS_PUSH
#include <rviz/display_context.h>
#include <rviz/properties/bool_property.h>
#include <rviz/properties/enum_property.h>
#include <rviz/properties/property.h>

#include <rviz/ogre_helpers/axes.h>
#include <rviz/ogre_helpers/object.h>
#include <rviz/ogre_helpers/shape.h>

#include <OgreEntity.h>
#include <OgreMaterial.h>
#include <OgreMaterialManager.h>
#include <OgreResourceGroupManager.h>
#include <OgreSceneManager.h>
#include <OgreSceneNode.h>

#include <ros/assert.h>
#include <ros/console.h>
#include <ros/package.h>
TESSERACT_COMMON_IGNORE_WARNINGS_POP

#include <tesseract_rviz/conversions.h>

namespace tesseract_rviz
{
VisualizationWidget::VisualizationWidget(Ogre::SceneNode* root_node,
                                         rviz::DisplayContext* context,
                                         std::string name,
                                         rviz::Property* parent_property)
  : scene_manager_(context->getSceneManager())
  , scene_graph_(std::make_shared<tesseract_scene_graph::SceneGraph>())
  , visible_(true)
  , visual_visible_(true)
  , collision_visible_(false)
  , current_state_visible_(true)
  , start_state_visible_(false)
  , end_state_visible_(false)
  , trajectory_visible_(false)
  , context_(context)
  , link_tree_(nullptr)
  , doing_set_checkbox_(false)
  , inChangedEnableAllLinks(false)
  , name_(std::move(name))
{
  // Add tesseract resources to ogre
  std::string tesseract_rviz_path = ros::package::getPath("tesseract_rviz");
  Ogre::ResourceGroupManager::getSingleton().addResourceLocation(
      tesseract_rviz_path + "/ogre_media/models", "FileSystem", "tesseract_rviz");

  root_visual_node_ = root_node->createChildSceneNode();
  root_collision_node_ = root_node->createChildSceneNode();
  root_other_node_ = root_node->createChildSceneNode();

  link_factory_ = new LinkFactory();

  setVisualVisible(visual_visible_);
  setCollisionVisible(collision_visible_);
  setAlpha(1.0f);

  link_tree_ = new rviz::Property("Links", QVariant(), "", parent_property);
  link_tree_->hide();  // hide until loaded

  link_tree_style_ = new rviz::EnumProperty(
      "Link Tree Style", "", "How the list of links is displayed", link_tree_, SLOT(changedLinkTreeStyle()), this);
  initLinkTreeStyle();
  expand_tree_ = new rviz::BoolProperty(
      "Expand Tree", false, "Expand or collapse link tree", link_tree_, SLOT(changedExpandTree()), this);
  expand_link_details_ = new rviz::BoolProperty("Expand Link Details",
                                                false,
                                                "Expand link details (sub properties) to see all info for all links.",
                                                link_tree_,
                                                SLOT(changedExpandLinkDetails()),
                                                this);
  expand_joint_details_ = new rviz::BoolProperty("Expand Joint Details",
                                                 false,
                                                 "Expand joint details (sub properties) "
                                                 "to see all info for all joints.",
                                                 link_tree_,
                                                 SLOT(changedExpandJointDetails()),
                                                 this);
  enable_all_links_ = new rviz::BoolProperty(
      "All Links Enabled", true, "Turn all links on or off.", link_tree_, SLOT(changedEnableAllLinks()), this);
}

VisualizationWidget::~VisualizationWidget()
{
  clear();

  scene_manager_->destroySceneNode(root_visual_node_->getName());
  scene_manager_->destroySceneNode(root_collision_node_->getName());
  scene_manager_->destroySceneNode(root_other_node_->getName());
  delete link_factory_;
  delete link_tree_;
}

void VisualizationWidget::setLinkFactory(LinkFactory* link_factory)
{
  if (link_factory)
  {
    delete link_factory_;
    link_factory_ = link_factory;
  }
}

void VisualizationWidget::setVisible(bool visible)
{
  visible_ = visible;
  if (visible)
  {
    root_visual_node_->setVisible(visual_visible_);
    root_collision_node_->setVisible(collision_visible_);
    updateLinkVisibilities();
  }
  else
  {
    root_visual_node_->setVisible(false);
    root_collision_node_->setVisible(false);
    updateLinkVisibilities();
  }
}

void VisualizationWidget::setVisualVisible(bool visible)
{
  visual_visible_ = visible;
  updateLinkVisibilities();
}

void VisualizationWidget::setCollisionVisible(bool visible)
{
  collision_visible_ = visible;
  updateLinkVisibilities();
}

void VisualizationWidget::setCurrentStateVisible(bool visible)
{
  current_state_visible_ = visible;
  updateLinkVisibilities();
}

void VisualizationWidget::setStartStateVisible(bool visible)
{
  start_state_visible_ = visible;
  updateLinkVisibilities();
}

void VisualizationWidget::setEndStateVisible(bool visible)
{
  end_state_visible_ = visible;
  updateLinkVisibilities();
}

void VisualizationWidget::setTrajectoryVisible(bool visible)
{
  trajectory_visible_ = visible;
  updateLinkVisibilities();
}

void VisualizationWidget::updateLinkVisibilities()
{
  for (auto& link : links_)
    link.second->updateVisibility();
}

bool VisualizationWidget::isVisible() { return visible_; }
bool VisualizationWidget::isVisualVisible() { return visual_visible_; }
bool VisualizationWidget::isCollisionVisible() { return collision_visible_; }
bool VisualizationWidget::isCurrentStateVisible() { return current_state_visible_; }
bool VisualizationWidget::isStartStateVisible() { return start_state_visible_; }
bool VisualizationWidget::isEndStateVisible() { return end_state_visible_; }
bool VisualizationWidget::isTrajectoryVisible() { return trajectory_visible_; }
void VisualizationWidget::setAlpha(float a)
{
  alpha_ = a;

  for (auto& link : links_)
    link.second->setAlpha(alpha_);
}

void VisualizationWidget::clear()
{
  // unparent all link and joint properties so they can be deleted in arbitrary
  // order without being delete by their parent propeties (which vary based on
  // style)
  unparentLinkProperties();

  for (auto& link : links_)
  {
    LinkWidget* l = link.second;
    delete l;
  }

  for (auto& joint : joints_)
  {
    JointWidget* j = joint.second;
    delete j;
  }

  links_.clear();
  joints_.clear();
  active_links_.clear();
  root_visual_node_->removeAndDestroyAllChildren();
  root_collision_node_->removeAndDestroyAllChildren();
  root_other_node_->removeAndDestroyAllChildren();
  scene_graph_->clear();

  root_link_ = nullptr;
  link_tree_->hide();
}

bool VisualizationWidget::isInitialized() const { return initialized_; }

LinkWidget* VisualizationWidget::LinkFactory::createLink(VisualizationWidget* env,
                                                         const tesseract_scene_graph::Link& link,
                                                         bool visual,
                                                         bool collision)
{
  return new LinkWidget(env, link, visual, collision);
}

JointWidget* VisualizationWidget::LinkFactory::createJoint(VisualizationWidget* robot,
                                                           const tesseract_scene_graph::Joint& joint)
{
  return new JointWidget(robot, joint);
}

void VisualizationWidget::initialize(bool visual, bool collision, bool show_active, bool show_static)
{
  link_tree_->hide();  // hide until loaded
  initialized_ = false;
  load_visual_ = visual;
  load_collision_ = collision;
  load_active_ = show_active;
  load_static_ = show_static;

  // clear out any data (properties, shapes, etc) from a previously loaded robot.
  clear();

  // Populate the list of active links
  //  tesseract::tesseract_ros::getActiveLinkNamesRecursive(active_links_, urdf->getRoot(), false);

  // Show Tree
  link_tree_->show();

  // set the link tree style and add link/joint properties to rviz pane.
  setLinkTreeStyle(LinkTreeStyle(link_tree_style_->getOptionInt()));

  // at startup the link tree is collapsed since it is large and not often
  // needed.
  link_tree_->collapse();

  setVisualVisible(isVisualVisible());
  setCollisionVisible(isCollisionVisible());

  initialized_ = true;
}

bool VisualizationWidget::addSceneGraph(const tesseract_scene_graph::SceneGraph& scene_graph, const std::string& prefix)
{
  unparentLinkProperties();

  // Create properties for each link.
  // Properties are not added to display until changedLinkTreeStyle() is called
  // (below).
  {
    bool is_scene_empty = scene_graph_->isEmpty();
    std::vector<tesseract_scene_graph::Link::ConstPtr> links = scene_graph.getLinks();
    for (const tesseract_scene_graph::Link::ConstPtr& tlink : links)
    {
      if (prefix.empty())
        addLink(tlink->clone());
      else
        addLink(tlink->clone(prefix));
    }

    if (is_scene_empty)
    {
      scene_graph_->setRoot(scene_graph_->getRoot());
      root_link_ = links_[scene_graph_->getRoot()];
    }
  }

  // Create properties for each joint.
  // Properties are not added to display until changedLinkTreeStyle() is called
  // (below).
  {
    std::vector<tesseract_scene_graph::Joint::ConstPtr> joints = scene_graph.getJoints();
    for (const tesseract_scene_graph::Joint::ConstPtr& tjoint : joints)
    {
      if (prefix.empty())
        addJoint(tjoint->clone());
      else
        addJoint(tjoint->clone(prefix));
    }
  }

  // Add allowed collision matrix
  for (const auto& acm_pair : scene_graph.getAllowedCollisionMatrix()->getAllAllowedCollisions())
    addAllowedCollision(acm_pair.first.first, acm_pair.first.second, acm_pair.second);

  return true;
}

bool VisualizationWidget::addSceneGraph(const tesseract_scene_graph::SceneGraph& scene_graph,
                                        const tesseract_scene_graph::Joint& joint,
                                        const std::string& prefix)
{
  unparentLinkProperties();

  // Create properties for each link.
  // Properties are not added to display until changedLinkTreeStyle() is called
  // (below).
  {
    assert(!scene_graph_->isEmpty());
    std::vector<tesseract_scene_graph::Link::ConstPtr> links = scene_graph.getLinks();
    for (const tesseract_scene_graph::Link::ConstPtr& tlink : links)
    {
      if (prefix.empty())
        addLink(tlink->clone());
      else
        addLink(tlink->clone(prefix));
    }
  }

  // Create properties for each joint.
  // Properties are not added to display until changedLinkTreeStyle() is called
  // (below).
  {
    // Add connecting joint
    addJoint(joint);

    // Add all other joints
    std::vector<tesseract_scene_graph::Joint::ConstPtr> joints = scene_graph.getJoints();
    for (const tesseract_scene_graph::Joint::ConstPtr& tjoint : joints)
    {
      if (prefix.empty())
        addJoint(tjoint->clone());
      else
        addJoint(tjoint->clone(prefix));
    }
  }

  // Add allowed collision matrix
  for (const auto& acm_pair : scene_graph.getAllowedCollisionMatrix()->getAllAllowedCollisions())
    addAllowedCollision(acm_pair.first.first, acm_pair.first.second, acm_pair.second);

  return true;
}

bool VisualizationWidget::addLink(const tesseract_scene_graph::Link& link, bool replace_allowed)
{
  unparentLinkProperties();

  bool link_exists = (links_.find(link.getName()) != links_.end());
  if (link_exists && !replace_allowed)
  {
    ROS_WARN("Tried to add link (%s) with same name as an existing link.", link.getName().c_str());
    return false;
  }

  bool show_geom = true;
  bool is_active = std::find(active_links_.begin(), active_links_.end(), link.getName()) != active_links_.end();
  if ((!load_active_ && is_active) || (!load_static_ && !is_active))
    show_geom = false;

  // Add or replace link
  if (!scene_graph_->addLink(link, link_exists))
    return false;

  // Remove current link widget if exists
  if (link_exists)
  {
    ROS_DEBUG("Visualization Widget: Replacing link %s", link.getName().c_str());
    LinkWidget* current_link = links_[link.getName()];
    links_.erase(link.getName());
    current_link->setParentProperty(nullptr);
    delete current_link;
  }

  LinkWidget* tlink = link_factory_->createLink(this, link, load_visual_ & show_geom, load_collision_ & show_geom);
  tlink->setAlpha(alpha_);
  tlink->updateVisibility();

  links_[link.getName()] = tlink;

  return true;
}

bool VisualizationWidget::removeLink(const std::string& name)
{
  unparentLinkProperties();
  auto it = links_.find(name);
  if (it == links_.end())
  {
    ROS_WARN("Tried to remove link (%s) that does not exist", name.c_str());
    return false;
  }

  std::vector<tesseract_scene_graph::Joint::ConstPtr> joints = scene_graph_->getInboundJoints(name);
  assert(joints.size() <= 1);

  // get child link names to remove
  std::vector<std::string> child_link_names = scene_graph_->getLinkChildrenNames(name);

  LinkWidget* link = it->second;
  link->setParentProperty(nullptr);
  delete link;
  links_.erase(name);
  scene_graph_->removeLink(name);

  for (const auto& link_name : child_link_names)
  {
    auto it2 = links_.find(link_name);
    LinkWidget* link = it2->second;
    link->setParentProperty(nullptr);
    delete link;
    links_.erase(link_name);

    std::vector<tesseract_scene_graph::Joint::ConstPtr> joints = scene_graph_->getInboundJoints(link_name);
    if (joints.size() == 1)
    {
      auto it3 = joints_.find(name);
      if (it3 != joints_.end())
      {
        JointWidget* joint = it3->second;
        joint->setParentProperty(nullptr);
        delete joint;
        joints_.erase(joints[0]->getName());
      }
    }
    scene_graph_->removeLink(link_name);
  }

  return true;
}

bool VisualizationWidget::moveLink(const tesseract_scene_graph::Joint& joint)
{
  unparentLinkProperties();

  std::vector<tesseract_scene_graph::Joint::ConstPtr> joints = scene_graph_->getInboundJoints(joint.child_link_name);
  assert(joints.size() == 1);

  auto it = joints_.find(joints[0]->getName());
  if (it != joints_.end())
  {
    JointWidget* joint = it->second;
    joint->setParentProperty(nullptr);
    delete joint;
    joints_.erase(joints[0]->getName());
  }

  if (!addJoint(joint.clone()))
    return false;

  scene_graph_->removeJoint(joints[0]->getName());

  return true;
}

bool VisualizationWidget::addJoint(const tesseract_scene_graph::Joint& joint, bool replace)
{
  unparentLinkProperties();

  bool joint_exists = (joints_.find(joint.getName()) != joints_.end());
  if (joint_exists && !replace)
  {
    ROS_WARN("Tried to add joint (%s) with same name as an existing joint.", joint.getName().c_str());
    return false;
  }

  if (!joint_exists && replace)
  {
    ROS_WARN("Tried to replace joint (%s) which does not exist.", joint.getName().c_str());
    return false;
  }

  if (joint_exists)
  {
    // Add link to scene_graph
    if (!scene_graph_->removeJoint(joint.getName()))
      return false;

    // Add link to scene_graph
    if (!scene_graph_->addJoint(joint.clone()))
      return false;

    // Remove current joint widget
    JointWidget* current_joint = joints_[joint.getName()];
    current_joint->setParentProperty(nullptr);
    delete current_joint;
    joints_.erase(joint.getName());
  }
  else
  {
    // Add link to scene_graph
    if (!scene_graph_->addJoint(joint.clone()))
      return false;
  }

  // Add new joint widget
  JointWidget* tjoint = link_factory_->createJoint(this, joint);
  tjoint->setAlpha(alpha_);

  joints_[joint.getName()] = tjoint;

  return true;
}

bool VisualizationWidget::removeJoint(const std::string& name)
{
  auto it = joints_.find(name);
  if (it == joints_.end())
  {
    ROS_WARN("Tried to remove Joint (%s) that does not exist", name.c_str());
    return false;
  }

  JointWidget* joint = it->second;
  return removeLink(joint->getChildLinkName());
}

bool VisualizationWidget::moveJoint(const std::string& joint_name, const std::string& parent_link)
{
  unparentLinkProperties();

  auto it = joints_.find(joint_name);
  if (it == joints_.end())
  {
    ROS_WARN("Tried to move Joint (%s) that does not exist", joint_name.c_str());
    return false;
  }

  JointWidget* joint = it->second;
  joint->setParentLinkName(parent_link);

  // Move joint in scene_graph
  tesseract_scene_graph::Joint::ConstPtr cj = scene_graph_->getJoint(joint_name);
  tesseract_scene_graph::Joint nj = cj->clone(joint_name);
  nj.parent_link_name = parent_link;
  std::vector<tesseract_scene_graph::Joint::ConstPtr> joints = scene_graph_->getInboundJoints(cj->child_link_name);
  assert(joints.size() == 1);
  if (!scene_graph_->removeJoint(joints[0]->getName()))
    return false;

  if (!scene_graph_->addJoint(std::move(nj)))
    return false;

  return true;
}

bool VisualizationWidget::changeJointOrigin(const std::string& name, const Eigen::Isometry3d& new_origin)
{
  auto found = joints_.find(name);

  if (found == joints_.end())
  {
    ROS_WARN("Tried to change origin of Joint (%s) that does not exist", name.c_str());
    return false;
  }

  // Update transform associated with the joint
  JointWidget* joint = found->second;
  Ogre::Vector3 pos;
  Ogre::Quaternion quat;
  toOgre(pos, quat, new_origin);
  joint->setTransforms(pos, quat);

  scene_graph_->changeJointOrigin(name, new_origin);
  return true;
}

void VisualizationWidget::addAllowedCollision(const std::string& link_name1,
                                              const std::string& link_name2,
                                              const std::string& reason)
{
  unparentLinkProperties();

  auto it1 = links_.find(link_name1);
  if (it1 == links_.end())
  {
    ROS_WARN("Tried to add allowed collision for Link (%s) that does not exist", link_name1.c_str());
    return;
  }

  auto it2 = links_.find(link_name2);
  if (it2 == links_.end())
  {
    ROS_WARN("Tried to add allowed collision for Link (%s) that does not exist", link_name2.c_str());
    return;
  }

  LinkWidget* link1 = it1->second;
  link1->addAllowedCollision(link_name2, reason);

  LinkWidget* link2 = it2->second;
  link2->addAllowedCollision(link_name1, reason);

  // Add to scene graph
  scene_graph_->addAllowedCollision(link_name1, link_name2, reason);
}

void VisualizationWidget::removeAllowedCollision(const std::string& link_name1, const std::string& link_name2)
{
  unparentLinkProperties();

  auto it1 = links_.find(link_name1);
  if (it1 == links_.end())
  {
    ROS_WARN("Tried to remove allowed collision for Link (%s) that does not exist", link_name1.c_str());
    return;
  }

  auto it2 = links_.find(link_name2);
  if (it2 == links_.end())
  {
    ROS_WARN("Tried to remove allowed collision for Link (%s) that does not exist", link_name2.c_str());
    return;
  }

  LinkWidget* link1 = it1->second;
  link1->removeAllowedCollision(link_name2);

  LinkWidget* link2 = it2->second;
  link2->removeAllowedCollision(link_name1);

  // Remove from scene graph
  scene_graph_->removeAllowedCollision(link_name1, link_name2);
}

void VisualizationWidget::removeAllowedCollision(const std::string& link_name)
{
  unparentLinkProperties();

  auto it = links_.find(link_name);
  if (it == links_.end())
  {
    ROS_WARN("Tried to remove all allowed collision for Link (%s) that does not exist", link_name.c_str());
    return;
  }

  it->second->clearAllowedCollisions();

  for (auto link_pair : links_)
  {
    LinkWidget* link = link_pair.second;
    link->removeAllowedCollision(link_name);
  }

  // Remove from scene graph
  scene_graph_->removeAllowedCollision(link_name);
}

void VisualizationWidget::setLinkCollisionEnabled(const std::string& name, bool enabled)
{
  auto it = links_.find(name);
  if (it == links_.end())
  {
    ROS_WARN("Tried to change link (%s) collision enabled, which does not exist", name.c_str());
    return;
  }

  LinkWidget* link = it->second;
  link->setCollisionEnabled(enabled);
  scene_graph_->setLinkCollisionEnabled(name, enabled);
}

void VisualizationWidget::setLinkVisibleEnabled(const std::string& name, bool enabled)
{
  auto it = links_.find(name);
  if (it == links_.end())
  {
    ROS_WARN("Tried to change link (%s) collision enabled, which does not exist", name.c_str());
    return;
  }

  LinkWidget* link = it->second;
  link->setVisibleEnabled(enabled);
  scene_graph_->setLinkCollisionEnabled(name, enabled);
}

void VisualizationWidget::unparentLinkProperties()
{
  // remove link properties from their parents
  for (auto& link : links_)
    link.second->setParentProperty(nullptr);

  // remove joint properties from their parents
  for (auto& joint : joints_)
    joint.second->setParentProperty(nullptr);
}

void VisualizationWidget::useDetailProperty(bool use_detail)
{
  // remove sub properties and add them to detail
  for (auto& link : links_)
    link.second->useDetailProperty(use_detail);

  // remove joint properties from their parents
  for (auto& joint : joints_)
    joint.second->useDetailProperty(use_detail);
}

void VisualizationWidget::changedExpandTree()
{
  bool expand = expand_tree_->getBool();

  for (auto& link : links_)
  {
    if (expand)
      link.second->getLinkProperty()->expand();
    else
      link.second->getLinkProperty()->collapse();
  }

  for (auto& joint : joints_)
  {
    if (expand)
      joint.second->getJointProperty()->expand();
    else
      joint.second->getJointProperty()->collapse();
  }
}

void VisualizationWidget::changedHideSubProperties()
{
  bool hide = /* !show_details_->getBool(); */ false;

  for (auto& link : links_)
    link.second->hideSubProperties(hide);

  for (auto& joint : joints_)
    joint.second->hideSubProperties(hide);
}

void VisualizationWidget::changedExpandLinkDetails()
{
  bool expand = expand_link_details_->getBool();

  for (auto& link : links_)
    link.second->expandDetails(expand);
}

void VisualizationWidget::changedExpandJointDetails()
{
  bool expand = expand_joint_details_->getBool();

  for (auto& joint : joints_)
    joint.second->expandDetails(expand);
}

void VisualizationWidget::changedEnableAllLinks()
{
  if (doing_set_checkbox_)
    return;

  bool enable = enable_all_links_->getBool();

  inChangedEnableAllLinks = true;

  for (auto& link : links_)
    if (link.second->hasGeometry())
      link.second->getLinkProperty()->setValue(enable);

  for (auto& joint : joints_)
    if (joint.second->hasDescendentLinksWithGeometry())
      joint.second->getJointProperty()->setValue(enable);

  inChangedEnableAllLinks = false;
}

void VisualizationWidget::setEnableAllLinksCheckbox(const QVariant& val)
{
  // doing_set_checkbox_ prevents changedEnableAllLinks from turning all
  // links off when we modify the enable_all_links_ property.
  doing_set_checkbox_ = true;
  enable_all_links_->setValue(val);
  doing_set_checkbox_ = false;
}

void VisualizationWidget::initLinkTreeStyle()
{
  style_name_map_.clear();
  style_name_map_[STYLE_LINK_LIST] = "Links in Alphabetic Order";
  style_name_map_[STYLE_JOINT_LIST] = "Joints in Alphabetic Order";
  style_name_map_[STYLE_LINK_TREE] = "Tree of links";
  style_name_map_[STYLE_JOINT_LINK_TREE] = "Tree of links and joints";

  link_tree_style_->clearOptions();
  std::map<LinkTreeStyle, std::string>::const_iterator style_it = style_name_map_.begin();
  std::map<LinkTreeStyle, std::string>::const_iterator style_end = style_name_map_.end();
  for (; style_it != style_end; ++style_it)
  {
    link_tree_style_->addOptionStd(style_it->second, style_it->first);
  }
}

bool VisualizationWidget::styleShowLink(LinkTreeStyle style)
{
  return style == STYLE_LINK_LIST || style == STYLE_LINK_TREE || style == STYLE_JOINT_LINK_TREE;
}

bool VisualizationWidget::styleShowJoint(LinkTreeStyle style)
{
  return style == STYLE_JOINT_LIST || style == STYLE_JOINT_LINK_TREE;
}
bool VisualizationWidget::styleIsTree(LinkTreeStyle style)
{
  return style == STYLE_LINK_TREE || style == STYLE_JOINT_LINK_TREE;
}
void VisualizationWidget::setLinkTreeStyle(LinkTreeStyle style)
{
  std::map<LinkTreeStyle, std::string>::const_iterator style_it = style_name_map_.find(style);
  if (style_it == style_name_map_.end())
    link_tree_style_->setValue(style_name_map_[STYLE_DEFAULT].c_str());
  else
    link_tree_style_->setValue(style_it->second.c_str());
}

JointWidget* VisualizationWidget::findParentJoint(JointWidget* joint)
{
  const std::string& parent_link = joint->getParentLinkName();
  for (auto& joint_pair : joints_)
    if (parent_link == joint_pair.second->getChildLinkName())
      return joint_pair.second;

  return nullptr;
}

JointWidget* VisualizationWidget::findParentJoint(LinkWidget* link)
{
  for (auto& joint_pair : joints_)
    if (link->getName() == joint_pair.second->getChildLinkName())
      return joint_pair.second;

  return nullptr;
}

JointWidget* VisualizationWidget::findChildJoint(LinkWidget* link)
{
  for (auto& joint_pair : joints_)
    if (link->getName() == joint_pair.second->getParentLinkName())
      return joint_pair.second;

  return nullptr;
}

// insert properties into link_tree_ according to style
void VisualizationWidget::changedLinkTreeStyle()
{
  auto style = LinkTreeStyle(link_tree_style_->getOptionInt());

  unparentLinkProperties();

  // expand_tree_->setValue(false);

  switch (style)
  {
    case STYLE_LINK_TREE:
    case STYLE_JOINT_LINK_TREE:
      useDetailProperty(true);
      if (root_link_)
      {
        if (styleShowLink(style))
        {
          for (auto& link : links_)
          {
            link.second->setLinkPropertyDescription();
            JointWidget* parent_joint = findParentJoint(link.second);

            if (parent_joint == nullptr)
            {
              link.second->setParentProperty(link_tree_);
            }
            else
            {
              if (styleShowJoint(style))
              {
                link.second->setParentProperty(parent_joint->getJointProperty());
              }
              else
              {
                auto it = links_.find(parent_joint->getParentLinkName());
                if (it != links_.end())
                  link.second->setParentProperty(it->second->getLinkProperty());
                else
                  link.second->setParentProperty(link_tree_);
              }
            }
          }
        }

        if (styleShowJoint(style))
        {
          for (auto& joint_pair : joints_)
          {
            JointWidget* joint = joint_pair.second;
            joint->setJointPropertyDescription();

            auto parent_it = links_.find(joint->getParentLinkName());

            if (parent_it == links_.end())
            {
              joint->setParentProperty(link_tree_);
            }
            else
            {
              if (styleShowLink(style))
              {
                joint->setParentProperty(parent_it->second->getLinkProperty());
              }
              else
              {
                JointWidget* parent_joint = findParentJoint(joint);
                if (parent_joint != nullptr)
                  joint->setParentProperty(parent_joint->getJointProperty());
                else
                  joint->setParentProperty(link_tree_);
              }
            }
          }
        }
      }
      break;

    case STYLE_JOINT_LIST:
    {
      useDetailProperty(false);
      auto joint_it = joints_.begin();
      auto joint_end = joints_.end();
      for (; joint_it != joint_end; ++joint_it)
      {
        joint_it->second->setParentProperty(link_tree_);
        joint_it->second->setJointPropertyDescription();
      }
      break;
    }

    case STYLE_LINK_LIST:
    default:
      useDetailProperty(false);
      for (auto& link : links_)
      {
        link.second->setParentProperty(link_tree_);
        link.second->setLinkPropertyDescription();
      }
      break;
  }

  switch (style)
  {
    case STYLE_LINK_TREE:
      link_tree_->setName("Link Tree");
      link_tree_->setDescription("A tree of all links in the robot.  Uncheck a "
                                 "link to hide its geometry.");
      expand_tree_->show();
      expand_link_details_->show();
      expand_joint_details_->hide();
      break;
    case STYLE_JOINT_LINK_TREE:
      link_tree_->setName("Link/Joint Tree");
      link_tree_->setDescription("A tree of all joints and links in the robot.  "
                                 "Uncheck a link to hide its geometry.");
      expand_tree_->show();
      expand_link_details_->show();
      expand_joint_details_->show();
      break;
    case STYLE_JOINT_LIST:
      link_tree_->setName("Joints");
      link_tree_->setDescription("All joints in the robot in alphabetic order.");
      expand_tree_->hide();
      expand_link_details_->hide();
      expand_joint_details_->show();
      break;
    case STYLE_LINK_LIST:
    default:
      link_tree_->setName("Links");
      link_tree_->setDescription("All links in the robot in alphabetic order.  "
                                 "Uncheck a link to hide its geometry.");
      expand_tree_->hide();
      expand_link_details_->show();
      expand_joint_details_->hide();
      break;
  }

  if (link_tree_->getHidden())
    link_tree_->show();

  expand_link_details_->setValue(false);
  expand_joint_details_->setValue(false);
  expand_tree_->setValue(false);
  calculateJointCheckboxes();
}

LinkWidget* VisualizationWidget::getLink(const std::string& name)
{
  auto it = links_.find(name);
  if (it == links_.end())
  {
    ROS_DEBUG("Link [%s] does not exist", name.c_str());
    return nullptr;
  }

  return it->second;
}

JointWidget* VisualizationWidget::getJoint(const std::string& name)
{
  auto it = joints_.find(name);
  if (it == joints_.end())
  {
    ROS_DEBUG("Joint [%s] does not exist", name.c_str());
    return nullptr;
  }

  return it->second;
}

void VisualizationWidget::calculateJointCheckboxes()
{
  if (inChangedEnableAllLinks)
    return;

  int links_with_geom_checked = 0;
  int links_with_geom_unchecked = 0;

  // check root link
  LinkWidget* link = root_link_;

  if (!link)
  {
    setEnableAllLinksCheckbox(QVariant());
    return;
  }

  if (link->hasGeometry())
  {
    bool checked = link->getLinkProperty()->getValue().toBool();
    links_with_geom_checked += checked ? 1 : 0;
    links_with_geom_unchecked += checked ? 0 : 1;
  }
  int links_with_geom = links_with_geom_checked + links_with_geom_unchecked;

  // check all child links and joints recursively
  JointWidget* child_joint = findChildJoint(link);
  while (child_joint != nullptr)
  {
    int child_links_with_geom;
    int child_links_with_geom_checked;
    int child_links_with_geom_unchecked;
    child_joint->calculateJointCheckboxesRecursive(
        child_links_with_geom, child_links_with_geom_checked, child_links_with_geom_unchecked);
    links_with_geom_checked += child_links_with_geom_checked;
    links_with_geom_unchecked += child_links_with_geom_unchecked;

    link = getLink(child_joint->getChildLinkName());
    child_joint = findChildJoint(link);
  }
  links_with_geom = links_with_geom_checked + links_with_geom_unchecked;

  if (!links_with_geom)
  {
    setEnableAllLinksCheckbox(QVariant());
  }
  else
  {
    setEnableAllLinksCheckbox(links_with_geom_unchecked == 0);
  }
}

void VisualizationWidget::update() { changedLinkTreeStyle(); }

void VisualizationWidget::update(const TransformMap& transforms)
{
  for (auto& link_pair : links_)
  {
    LinkWidget* link = link_pair.second;

    link->setToNormalMaterial();
    auto it = transforms.find(link->getName());
    if (it != transforms.end())
    {
      link->setCurrentTransform(it->second);
      link->setStartTransform(it->second);
      link->setEndTransform(it->second);
    }
    else
    {
      link->setToErrorMaterial();
    }
  }

  // Update joint transformations
  for (auto& joint_pair : joints_)
  {
    JointWidget* joint = joint_pair.second;

    LinkWidget* p_link = links_[joint->getParentLinkName()];
    joint->setTransforms(p_link->getPosition(), p_link->getOrientation());
  }
}

void VisualizationWidget::setStartState(const TransformMap& transforms)
{
  for (auto& link_pair : links_)
  {
    LinkWidget* link = link_pair.second;

    link->setToNormalMaterial();
    auto it = transforms.find(link->getName());
    if (it != transforms.end())
    {
      link->setStartTransform(it->second);
    }
    else
    {
      link->setToErrorMaterial();
    }
  }
}

void VisualizationWidget::setEndState(const TransformMap& transforms)
{
  for (auto& link_pair : links_)
  {
    LinkWidget* link = link_pair.second;

    link->setToNormalMaterial();
    auto it = transforms.find(link->getName());
    if (it != transforms.end())
    {
      link->setEndTransform(it->second);
    }
    else
    {
      link->setToErrorMaterial();
    }
  }
}

void VisualizationWidget::setPosition(const Ogre::Vector3& position)
{
  root_visual_node_->setPosition(position);
  root_collision_node_->setPosition(position);
}

void VisualizationWidget::setOrientation(const Ogre::Quaternion& orientation)
{
  root_visual_node_->setOrientation(orientation);
  root_collision_node_->setOrientation(orientation);
}

void VisualizationWidget::setScale(const Ogre::Vector3& scale)
{
  root_visual_node_->setScale(scale);
  root_collision_node_->setScale(scale);
}

const Ogre::Vector3& VisualizationWidget::getPosition() { return root_visual_node_->getPosition(); }
const Ogre::Quaternion& VisualizationWidget::getOrientation() { return root_visual_node_->getOrientation(); }
}  // namespace tesseract_rviz
