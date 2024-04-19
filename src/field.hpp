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

#include <OgreVector.h>
#include <assert.h>

#include <vector>

namespace rviz_satellite
{

// compute list of offsets on the far end of a field with side length (blocks * 2 + 1), when moving
// in the direction of the given offset
std::vector<Ogre::Vector2i> farEndOffsets(int blocks, Ogre::Vector2i offset);

// compute list of offsets on the near end of a field with side length (blocks * 2 + 1), when moving
// in the direction of the given offset
std::vector<Ogre::Vector2i> nearEndOffsets(int blocks, Ogre::Vector2i offset);

}  // namespace rviz_satellite
