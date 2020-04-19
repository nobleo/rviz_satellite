/* Copyright 2014 Gareth Cross, 2018-2019 TomTom N.V.

Licensed under the Apache License, Version 2.0 (the "License");
you may not use this file except in compliance with the License.
You may obtain a copy of the License at

http://www.apache.org/licenses/LICENSE-2.0

Unless required by applicable law or agreed to in writing, software
distributed under the License is distributed on an "AS IS" BASIS,
WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
See the License for the specific language governing permissions and
limitations under the License. */

#include "ogre_tile.h"

#include <string>

namespace
{
/**
 * Convert any QImage to a 24bit RGB QImage
 */
QImage convertImage(QImage image)
{
  return image.convertToFormat(QImage::Format_RGB888).mirrored();
}

/**
 * Generate a different texture name each call
 */
std::string uniqueTextureName()
{
  static int count = 0;
  ++count;
  return "satellite_texture_" + std::to_string(count);
}

/**
 * Create a Ogre::Texture on the GPU using a QImage
 */
Ogre::TexturePtr textureFromImage(QImage image)
{
  Ogre::DataStreamPtr data_stream;
  data_stream.bind(new Ogre::MemoryDataStream((void*)image.constBits(), image.byteCount()));

  Ogre::String const res_group = Ogre::ResourceGroupManager::DEFAULT_RESOURCE_GROUP_NAME;

  Ogre::TextureManager& texture_manager = Ogre::TextureManager::getSingleton();

  // swap byte order when going from QImage to Ogre
  return texture_manager.loadRawData(uniqueTextureName(), res_group, data_stream, image.width(), image.height(),
                                     Ogre::PF_B8G8R8, Ogre::TEX_TYPE_2D, 0);
}
}  // namespace

OgreTile::OgreTile(QImage image_) : texture(textureFromImage(convertImage(std::move(image_))))
{
}
