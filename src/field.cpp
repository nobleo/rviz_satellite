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
#include <vector>
#include <assert.h>
#include <algorithm>
#include "field.hpp"

namespace rviz_satellite
{

// https://stackoverflow.com/questions/1903954/is-there-a-standard-sign-function-signum-sgn-in-c-c
template<typename T>
int signum(T val)
{
  return (T(0) < val) - (val < T(0));
}

std::vector<Ogre::Vector2i> farEndOffsets(int blocks, Ogre::Vector2i offset)
{
  assert(blocks > 0);
  std::vector<Ogre::Vector2i> result;
  auto offset_x = offset.data[0];
  auto offset_y = offset.data[1];
  for (int x = 0; x < std::abs(offset_x); ++x) {
    for (int y = -blocks; y <= blocks; ++y) {
      auto curr_offset_x = -signum(offset_x) * blocks - signum(offset_x) * x;
      result.push_back(Ogre::Vector2i(curr_offset_x, y));
    }
  }
  for (int y = 0; y < std::abs(offset_y); ++y) {
    // add offset_x, since those columns were part of the previous loop
    auto min_x = std::max(-blocks, -blocks + offset_x);
    auto max_x = std::min(blocks, blocks + offset_x);
    for (int x = min_x; x <= max_x; ++x) {
      auto curr_offset_y = -signum(offset_y) * blocks - signum(offset_y) * y;
      result.push_back(Ogre::Vector2i(x, curr_offset_y));
    }
  }
  return result;
}

std::vector<Ogre::Vector2i> nearEndOffsets(int blocks, Ogre::Vector2i offset)
{
  assert(blocks > 0);
  std::vector<Ogre::Vector2i> result;
  auto offset_x = offset.data[0];
  auto offset_y = offset.data[1];
  assert(std::abs(offset_x) <= blocks);
  assert(std::abs(offset_y) <= blocks);
  for (int x = 1; x <= std::abs(offset_x); ++x) {
    for (int y = -blocks + offset_y; y <= blocks + offset_y; ++y) {
      auto curr_offset_x = signum(offset_x) * blocks + signum(offset_x) * x;
      result.push_back(Ogre::Vector2i(curr_offset_x, y));
    }
  }
  for (int y = 1; y <= std::abs(offset_y); ++y) {
    auto min_x = -blocks + offset_x;
    auto max_x = blocks + offset_x;
    min_x = std::max(min_x, min_x - offset_x);
    max_x = std::min(max_x, max_x - offset_x);
    for (int x = min_x; x <= max_x; ++x) {
      auto curr_offset_y = signum(offset_y) * blocks + signum(offset_y) * y;
      result.push_back(Ogre::Vector2i(x, curr_offset_y));
    }
  }
  return result;
}

}  // namespace rviz_satellite
