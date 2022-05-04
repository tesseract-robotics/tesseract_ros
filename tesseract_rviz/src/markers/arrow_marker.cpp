/*
 * Copyright (c) 2009, Willow Garage, Inc.
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

#include <OgreVector3.h>
#include <OgreQuaternion.h>
#include <OgreSceneNode.h>
#include <OgreSceneManager.h>
#include <OgreEntity.h>

#include "rviz/display_context.h"
#include "rviz/ogre_helpers/arrow.h"
#include "rviz/ogre_helpers/shape.h"
#include "rviz/selection/selection_manager.h"

#include <tesseract_rviz/markers/arrow_marker.h>
#include <tesseract_rviz/markers/marker_selection_handler.h>

namespace tesseract_rviz
{
ArrowMarker::ArrowMarker(const std::string& ns,
                         const int id,
                         Ogre::SceneManager* scene_manager,
                         Ogre::SceneNode* parent_node)
  : MarkerBase(ns, id, scene_manager, parent_node), arrow_(nullptr), location_(Ogre::Vector3(0, 0, 0))
{
  ctor(Ogre::Vector3(0, 0, 0), Ogre::Vector3(1, 0, 0), getDefaultProportions());
}

ArrowMarker::ArrowMarker(const std::string& ns,
                         const int id,
                         Ogre::Vector3 point1,
                         Ogre::Vector3 point2,
                         Ogre::SceneManager* scene_manager,
                         Ogre::SceneNode* parent_node)
  : MarkerBase(ns, id, scene_manager, parent_node), arrow_(nullptr)
{
  Ogre::Vector3 direction = point2 - point1;
  float distance = direction.length();
  direction.normalise();

  float head_length = distance * 0.23f;
  float shaft_diameter = distance * 0.35f;
  float head_diameter = distance * 0.55f;
  float shaft_length = distance - head_length;
  std::array<float, 4> proportions = { shaft_length, shaft_diameter, head_length, head_diameter };

  ctor(point1, direction, proportions);
}

ArrowMarker::ArrowMarker(const std::string& ns,
                         const int id,
                         Ogre::Vector3 location,
                         Ogre::Vector3 direction,
                         std::array<float, 4> proportions,
                         Ogre::SceneManager* scene_manager,
                         Ogre::SceneNode* parent_node)
  : MarkerBase(ns, id, scene_manager, parent_node), arrow_(nullptr)
{
  ctor(location, direction, proportions);
}

void ArrowMarker::ctor(Ogre::Vector3 location, Ogre::Vector3 direction, std::array<float, 4> proportions)
{
  child_scene_node_ = scene_node_->createChildSceneNode();
  location_ = location;
  arrow_ = new rviz::Arrow(scene_manager_, child_scene_node_);

  arrow_->set(proportions[0], proportions[1], proportions[2], proportions[3]);

  direction.normalise();

  // for some reason the arrow goes into the y direction by default
  Ogre::Quaternion orient = Ogre::Vector3::NEGATIVE_UNIT_Z.getRotationTo(direction);

  arrow_->setPosition(location_);
  arrow_->setOrientation(orient);
}

ArrowMarker::~ArrowMarker()
{
  delete arrow_;
  scene_manager_->destroySceneNode(child_scene_node_);
}

std::array<float, 4> ArrowMarker::getDefaultProportions() { return { 0.77f, 1.0f, 0.23f, 2.0f }; }

void ArrowMarker::setScale(Ogre::Vector3 scale)
{
  arrow_->getSceneNode()->setScale(scale);
  arrow_->setPosition(scale.x * location_);
}

Ogre::Vector3 ArrowMarker::getScale() const { return arrow_->getSceneNode()->getScale(); }

void ArrowMarker::setColor(Ogre::ColourValue color) { arrow_->setColor(color.r, color.g, color.b, color.a); }

std::set<Ogre::MaterialPtr> ArrowMarker::getMaterials()
{
  std::set<Ogre::MaterialPtr> materials;
  extractMaterials(arrow_->getHead()->getEntity(), materials);
  extractMaterials(arrow_->getShaft()->getEntity(), materials);
  return materials;
}

void ArrowMarker::createMarkerSelectionHandler(rviz::DisplayContext* context)
{
  handler_.reset(new MarkerSelectionHandler(this, getID(), context));
  handler_->addTrackedObjects(arrow_->getSceneNode());
}

}  // namespace tesseract_rviz
