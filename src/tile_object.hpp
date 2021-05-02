/* Copyright 2021 Austrian Institute of Technology GmbH

Licensed under the Apache License, Version 2.0 (the "License");
you may not use this file except in compliance with the License.
You may obtain a copy of the License at

http://www.apache.org/licenses/LICENSE-2.0

Unless required by applicable law or agreed to in writing, software
distributed under the License is distributed on an "AS IS" BASIS,
WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
See the License for the specific language governing permissions and
limitations under the License. */
#pragma once

#include <cstddef>
#include <string>

#include <OgreSharedPtr.h>
#include <OgrePrerequisites.h>
#include <OgreBlendMode.h>

#include <QImage>
#include "rviz_default_plugins/visibility_control.hpp"

namespace rviz_satellite
{

class TileObject
{
public:
  TileObject(
    Ogre::SceneManager * scene_manager,
    Ogre::SceneNode * parent_scene_node,
    std::string unique_id,
    Ogre::Real tile_size, Ogre::Real x, Ogre::Real y, bool draw_under);

  TileObject() = delete;

  ~TileObject();

  std::string objectId() const;

  Ogre::Real tileSize() const;

  void updateAlpha(float alpha);

  void updateData(QImage & map);

  void setVisible(bool visible);

  void translate(Ogre::Vector3);

  void setRenderQueueGroup(uint8_t group);

  void setDepthWriteEnabled(bool depth_write_enabled);

  Ogre::Pass * getTechniquePass();

private:
  void setupMaterial();
  void setupSceneNodeWithManualObject();
  void setupSquareManualObject();
  void addPointWithPlaneCoordinates(float x, float y);

  Ogre::SceneManager * scene_manager_;
  Ogre::SceneNode * parent_scene_node_;
  std::string unique_id_;
  Ogre::SceneNode * scene_node_;
  Ogre::ManualObject * manual_object_;
  Ogre::TexturePtr texture_;
  Ogre::MaterialPtr material_;
};

}  // namespace rviz_satellite
