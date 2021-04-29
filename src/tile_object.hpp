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

  void setRenderQueueGroup(uint8_t group);

  void setDepthWriteEnabled(bool depth_write_enabled);

  Ogre::Pass * getTechniquePass();

  std::string getTextureName();

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
