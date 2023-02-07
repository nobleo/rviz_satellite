/*
 * Copyright (c) 2012, Willow Garage, Inc.
 * Copyright (c) 2018, Bosch Software Innovations GmbH.
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
 *     * Neither the name of the copyright holder nor the names of its contributors
 *       may be used to endorse or promote products derived from
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

#include "tile_object.hpp"

#include <algorithm>
#include <string>
#include <vector>

#include <OgreManualObject.h>
#include <OgreMaterialManager.h>
#include <OgreRenderable.h>
#include <OgreSceneManager.h>
#include <OgreSceneNode.h>
#include <OgreSharedPtr.h>
#include <OgreTechnique.h>
#include <OgreTextureManager.h>

#include "rviz_rendering/material_manager.hpp"
#include "rviz_rendering/custom_parameter_indices.hpp"

namespace rviz_satellite
{

TileObject::TileObject(
  Ogre::SceneManager * scene_manager,
  Ogre::SceneNode * parent_scene_node,
  std::string unique_id,
  Ogre::Real tile_size, Ogre::Real x, Ogre::Real y, bool draw_under)
: scene_manager_(scene_manager),
  parent_scene_node_(parent_scene_node),
  unique_id_(unique_id),
  manual_object_(nullptr)
{
  setupMaterial();
  setupSceneNodeWithManualObject();

  scene_node_->setPosition(x, y, -1.0);
  scene_node_->setScale(tile_size, tile_size, 1.0);

  if (draw_under) {
    manual_object_->setRenderQueueGroup(Ogre::RENDER_QUEUE_4);
  }
}

TileObject::~TileObject()
{
  if (texture_) {
    Ogre::TextureManager::getSingleton().remove(texture_);
  }
  if (material_) {
    Ogre::MaterialManager::getSingleton().remove(material_);
  }
  if (manual_object_) {
    scene_node_->detachObject(manual_object_);
    scene_manager_->destroyManualObject(manual_object_);
  }
  if (scene_node_) {
    scene_manager_->destroySceneNode(scene_node_);
  }
}

void TileObject::updateAlpha(float alpha)
{
  if (alpha > 0.998f) {
    material_->setDepthWriteEnabled(true);
    material_->setSceneBlending(Ogre::SBT_REPLACE);
  } else {
    material_->setDepthWriteEnabled(false);
    material_->setSceneBlending(Ogre::SBT_TRANSPARENT_ALPHA);
  }
  auto texture_unit = material_->getTechnique(0)->getPass(0)->getTextureUnitState(0);
  texture_unit->setAlphaOperation(Ogre::LBX_SOURCE1, Ogre::LBS_MANUAL, Ogre::LBS_CURRENT, alpha);
}

void TileObject::updateData(QImage & image)
{
  auto ogre_format = Ogre::PF_B8G8R8;
  if (QImage::Format_Grayscale8 == image.format()) {
    ogre_format = Ogre::PF_L8;
  } else {
    image = image.convertToFormat(QImage::Format_RGB888);
  }
  Ogre::DataStreamPtr pixel_stream;
  pixel_stream.reset(
    new Ogre::MemoryDataStream(
      reinterpret_cast<void *>(image.bits()),
      image.sizeInBytes()));

  if (texture_) {
    Ogre::TextureManager::getSingleton().remove(texture_);
    texture_ = nullptr;
  }

  // TODO(ZeilingerM) check whether returned image is grayscale, and use reduced texture space Ogre::PF_L8
  texture_ = Ogre::TextureManager::getSingleton().loadRawData(
    objectId(),
    "rviz_rendering",
    pixel_stream,
    image.width(),
    image.height(),
    ogre_format,
    Ogre::TEX_TYPE_2D,
    2);
  material_->getTechnique(0)->getPass(0)->getTextureUnitState(0)->setTextureName(
    texture_->getName());
}

std::string TileObject::objectId() const
{
  return unique_id_;
}

Ogre::Real TileObject::tileSize() const
{
  return scene_node_->getScale().x;
}

void TileObject::setVisible(bool visible)
{
  scene_node_->setVisible(visible);
}

void TileObject::translate(Ogre::Vector3 d)
{
  scene_node_->translate(d);
}

void TileObject::setRenderQueueGroup(uint8_t group)
{
  if (manual_object_) {
    manual_object_->setRenderQueueGroup(group);
  }
}

void TileObject::setDepthWriteEnabled(bool depth_write_enabled)
{
  if (material_) {
    material_->setDepthWriteEnabled(depth_write_enabled);
  }
}

Ogre::Pass * TileObject::getTechniquePass()
{
  if (material_) {
    return material_->getTechnique(0)->getPass(0);
  }
  return nullptr;
}

void TileObject::setupMaterial()
{
  material_ = rviz_rendering::MaterialManager::createMaterialWithNoLighting(objectId());
  material_->setDepthBias(-16.0f, 0.0f);
  material_->setCullingMode(Ogre::CULL_NONE);
  material_->setDepthWriteEnabled(false);
  auto texture_unit_state = material_->getTechnique(0)->getPass(0)->createTextureUnitState();
  texture_unit_state->setTextureFiltering(Ogre::TFO_BILINEAR);
}

void TileObject::setupSceneNodeWithManualObject()
{
  std::stringstream ss;
  ss << unique_id_;
  manual_object_ = scene_manager_->createManualObject("TileObject/" + ss.str());
  scene_node_ = parent_scene_node_->createChildSceneNode(ss.str());
  scene_node_->attachObject(manual_object_);
  setupSquareManualObject();
}

void TileObject::setupSquareManualObject()
{
  manual_object_->begin(
    material_->getName(), Ogre::RenderOperation::OT_TRIANGLE_LIST, "rviz_rendering");

  // first triangle
  addPointWithPlaneCoordinates(0.0f, 0.0f);
  addPointWithPlaneCoordinates(1.0f, 1.0f);
  addPointWithPlaneCoordinates(0.0f, 1.0f);

  // second triangle
  addPointWithPlaneCoordinates(0.0f, 0.0f);
  addPointWithPlaneCoordinates(1.0f, 0.0f);
  addPointWithPlaneCoordinates(1.0f, 1.0f);

  manual_object_->end();
}

void TileObject::addPointWithPlaneCoordinates(float x, float y)
{
  manual_object_->position(x, y, 0.0f);
  manual_object_->textureCoord(x, y);
  manual_object_->normal(0.0f, 0.0f, 1.0f);
}

}  // namespace rviz_satellite
