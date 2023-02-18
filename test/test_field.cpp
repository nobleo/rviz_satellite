#include "field.hpp"
#include <vector>
#include <gtest/gtest.h>
#include <algorithm>

bool contains(const std::vector<Ogre::Vector2i> & offsets, Ogre::Vector2i v)
{
  return std::find(offsets.begin(), offsets.end(), v) != offsets.end();
}

TEST(FieldTest, east_far_end_contains_whole_left_side) {
  auto offsets = rviz_satellite::farEndOffsets(1, Ogre::Vector2i(1, 0));
  EXPECT_EQ(3lu, offsets.size());
  EXPECT_TRUE(contains(offsets, Ogre::Vector2i(-1, -1)));
  EXPECT_TRUE(contains(offsets, Ogre::Vector2i(-1, 0)));
  EXPECT_TRUE(contains(offsets, Ogre::Vector2i(-1, 1)));
}

TEST(FieldTest, east_near_end_adds_whole_right_side) {
  auto offsets = rviz_satellite::nearEndOffsets(1, Ogre::Vector2i(1, 0));
  EXPECT_EQ(3lu, offsets.size());
  EXPECT_TRUE(contains(offsets, Ogre::Vector2i(2, -1)));
  EXPECT_TRUE(contains(offsets, Ogre::Vector2i(2, 0)));
  EXPECT_TRUE(contains(offsets, Ogre::Vector2i(2, 1)));
}

TEST(FieldTest, south_far_end_contains_whole_top_side) {
  auto offsets = rviz_satellite::farEndOffsets(1, Ogre::Vector2i(0, 1));
  EXPECT_EQ(3lu, offsets.size());
  EXPECT_TRUE(contains(offsets, Ogre::Vector2i(-1, -1)));
  EXPECT_TRUE(contains(offsets, Ogre::Vector2i(0, -1)));
  EXPECT_TRUE(contains(offsets, Ogre::Vector2i(1, -1)));
}

TEST(FieldTest, south_near_end_adds_whole_bottom_side) {
  auto offsets = rviz_satellite::nearEndOffsets(1, Ogre::Vector2i(0, 1));
  EXPECT_EQ(3lu, offsets.size());
  EXPECT_TRUE(contains(offsets, Ogre::Vector2i(-1, 2)));
  EXPECT_TRUE(contains(offsets, Ogre::Vector2i(0, 2)));
  EXPECT_TRUE(contains(offsets, Ogre::Vector2i(1, 2)));
}

TEST(FieldTest, west_far_end_contains_whole_right_side) {
  auto offsets = rviz_satellite::farEndOffsets(1, Ogre::Vector2i(-1, 0));
  EXPECT_EQ(3lu, offsets.size());
  EXPECT_TRUE(contains(offsets, Ogre::Vector2i(1, -1)));
  EXPECT_TRUE(contains(offsets, Ogre::Vector2i(1, 0)));
  EXPECT_TRUE(contains(offsets, Ogre::Vector2i(1, 1)));
}

TEST(FieldTest, west_near_end_adds_whole_left_side) {
  auto offsets = rviz_satellite::nearEndOffsets(1, Ogre::Vector2i(-1, 0));
  EXPECT_EQ(3lu, offsets.size());
  EXPECT_TRUE(contains(offsets, Ogre::Vector2i(-2, -1)));
  EXPECT_TRUE(contains(offsets, Ogre::Vector2i(-2, 0)));
  EXPECT_TRUE(contains(offsets, Ogre::Vector2i(-2, 1)));
}

TEST(FieldTest, se_diag_far_end_contains_unique_corner_offsets) {
  auto offsets = rviz_satellite::farEndOffsets(1, Ogre::Vector2i(1, 1));
  EXPECT_EQ(5lu, offsets.size());
  EXPECT_TRUE(contains(offsets, Ogre::Vector2i(-1, -1)));
  EXPECT_TRUE(contains(offsets, Ogre::Vector2i(-1, 0)));
  EXPECT_TRUE(contains(offsets, Ogre::Vector2i(-1, 1)));
  EXPECT_TRUE(contains(offsets, Ogre::Vector2i(0, -1)));
  EXPECT_TRUE(contains(offsets, Ogre::Vector2i(1, -1)));
}

TEST(FieldTest, se_diag_near_end_adds_unique_corner_offsets) {
  auto offsets = rviz_satellite::nearEndOffsets(1, Ogre::Vector2i(1, 1));
  EXPECT_EQ(5lu, offsets.size());
  EXPECT_TRUE(contains(offsets, Ogre::Vector2i(2, 0)));
  EXPECT_TRUE(contains(offsets, Ogre::Vector2i(2, 1)));
  EXPECT_TRUE(contains(offsets, Ogre::Vector2i(2, 2)));
  EXPECT_TRUE(contains(offsets, Ogre::Vector2i(1, 2)));
  EXPECT_TRUE(contains(offsets, Ogre::Vector2i(0, 2)));
}

TEST(FieldTest, nw_diag_far_end_contains_unique_corner_offsets) {
  auto offsets = rviz_satellite::farEndOffsets(1, Ogre::Vector2i(-1, -1));
  EXPECT_EQ(5lu, offsets.size());
  EXPECT_TRUE(contains(offsets, Ogre::Vector2i(1, -1)));
  EXPECT_TRUE(contains(offsets, Ogre::Vector2i(1, 0)));
  EXPECT_TRUE(contains(offsets, Ogre::Vector2i(1, 1)));
  EXPECT_TRUE(contains(offsets, Ogre::Vector2i(0, 1)));
  EXPECT_TRUE(contains(offsets, Ogre::Vector2i(-1, 1)));
}

TEST(FieldTest, nw_diag_near_end_adds_unique_corner_offsets) {
  auto offsets = rviz_satellite::nearEndOffsets(1, Ogre::Vector2i(-1, -1));
  EXPECT_EQ(5lu, offsets.size());
  EXPECT_TRUE(contains(offsets, Ogre::Vector2i(-2, 0)));
  EXPECT_TRUE(contains(offsets, Ogre::Vector2i(-2, -1)));
  EXPECT_TRUE(contains(offsets, Ogre::Vector2i(-2, -2)));
  EXPECT_TRUE(contains(offsets, Ogre::Vector2i(-1, -2)));
  EXPECT_TRUE(contains(offsets, Ogre::Vector2i(0, -2)));
}
