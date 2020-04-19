/* Copyright 2018-2019 TomTom N.V.

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

#include <OGRE/OgreTextureManager.h>
#include <QImage>
#include <utility>

/**
 * A OgreTile holds a Ogre Texture for a tile in the GPU cache.
 */
class OgreTile
{
public:
  Ogre::TexturePtr texture;

public:
  OgreTile(QImage image_);

  OgreTile(OgreTile&& other) noexcept
  {
    (*this) = std::move(other);
  }

  OgreTile& operator=(OgreTile&& other) noexcept
  {
    texture = other.texture;
    other.texture.setNull();

    return *this;
  }

  ~OgreTile()
  {
    if (!texture.isNull())
    {
      Ogre::TextureManager::getSingleton().remove(texture->getName());
    }
  }
};
