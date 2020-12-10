// Copyright 2020 Open Source Robotics Foundation, Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include "map_transformer/test_config.hpp"
#include "map_transformer/transformer.hpp"

#include <stdexcept>
#include <string>

#include <gtest/gtest.h>
#include <yaml-cpp/yaml.h>

using map_transformer::test::TEST_DATA_DIRECTORY;


class TestData : public ::testing::Test {
protected:
  const std::string AlignedMapYamlDoc() {
    // Points for testing:
    // Robot point <->  Ref point
    // Origins
    // 0, 0             0, 0
    // Corners of robot map
    // 694, 0           694, 0
    // 0, 386           0, 386
    // 694, 386         694, 386
    // Correspondence points
    // 262, 138         262, 138
    // 433, 201         433, 138
    // Midpoints between correspondence points
    // 341, 168         341, 138
    // 433, 252         433, 189
    // Triangle edges - move across vertical edge horizontally
    // 433, 108         433, 74
    // 432, 108         432, 74
    // Triangle edges - move across horizontal edge vertically
    // 160, 240         160, 240
    // 160, 241         160, 241
    // Triangle centers
    // 321, 194         321, 172
    // 177, 93          177, 93
    return std::string(
R"(ref_map:
  name: reference
  image_file: )") + TEST_DATA_DIRECTORY + R"(/aligned_map_ref.png
  size: [694, 386]
  correspondence_points:
    - [0, 138]
    - [0, 241]
    - [262, 0]
    - [262, 384]
    - [433, 0]
    - [433, 384]
    - [692, 138]
    - [692, 241]
    - [262, 138]
    - [262, 241]
    - [433, 138]
    - [433, 241]
robot_map:
  name: robot
  image_file: )" + TEST_DATA_DIRECTORY + R"(/aligned_map_robot.png
  size: [694, 386]
  correspondence_points:
    - [0, 138]
    - [0, 241]
    - [262, 0]
    - [262, 384]
    - [433, 0]
    - [433, 384]
    - [692, 138]
    - [692, 241]
    - [262, 138]
    - [262, 241]
    - [433, 201]
    - [433, 304])";
  }

  const std::string OffsetMapYamlDoc() {
    // Points for testing:
    // Robot point <->  Ref point
    // Origins
    // 0, 0             30, 20
    // -30, -20         0, 0
    // Corners of robot map
    // 80, 0            110, 20
    // 0, 110           30, 130
    // 80, 110          110, 130
    // 70, -20          100, 0
    // -30, 60          0, 100
    // 70, 60           100, 100
    // Correspondence points
    // 10, 20           40, 50
    // 40, 55           70, 70
    // Midpoints between correspondence points
    // 30, 0            55, 20
    // 25, 53           55, 70
    // 23, 66           56, 85
    // Triangle edges - move across vertical edge horizontally
    // 9, 10            39, 25
    // 10, 10           40, 35
    // 11, 10           40, 35
    // Triangle edges - move across horizontal edge vertically
    // 29, 19           55, 48
    // 29, 20           55, 50
    // 29, 21           55, 50
    // Triangle centers
    // 23, 13           50, 39
    // 33, 31           60, 56
    // 48, 64           79, 79
    // Outside triangulated area
    // 69, 0            99, 20
    // 0, 79            30, 99
    // 69, 79           99, 99
    // Outside the reference map
    // 79, 109          109, 129
    // 79, 40           109, 60
    return std::string(
R"(ref_map:
  name: reference
  size: [100, 100]
  image_file: )") + TEST_DATA_DIRECTORY + R"(/ref_map_100_100.png
  correspondence_points:
    - [30, 20]
    - [40, 50]
    - [70, 50]
    - [40, 70]
    - [70, 70]
    - [40, 20]
    - [70, 20]
    - [30, 50]
    - [99, 50]
    - [30, 70]
    - [99, 70]
    - [40, 99]
    - [70, 99]
robot_map:
  name: robot
  image_file: )" + TEST_DATA_DIRECTORY + R"(/robot_map_80_110.png
  size: [80, 110]
  transform:
    scale: [1, 1]
    rotation: 0
    translation: [30, 20]
  correspondence_points:
    - [0, 0]
    - [10, 20]
    - [46, 20]
    - [10, 51]
    - [40, 55]
    - [10, 0]
    - [50, 0]
    - [0, 20]
    - [69, 20]
    - [0, 50]
    - [69, 59]
    - [10, 79]
    - [34, 79]
)";
  }
};


TEST_F(TestData, transform_aligned_origins) {
  map_transformer::Transformer transformer(AlignedMapYamlDoc());
  auto transformed = transformer.to_ref(map_transformer::Point2D{0, 0});
  ASSERT_EQ(transformed.first, 0);
  ASSERT_EQ(transformed.second, 0);

  transformed = transformer.to_robot(map_transformer::Point2D{0, 0});
  ASSERT_EQ(transformed.first, 0);
  ASSERT_EQ(transformed.second, 0);
}

TEST_F(TestData, transform_aligned_corners_to_ref) {
  map_transformer::Transformer transformer(AlignedMapYamlDoc());
  auto transformed = transformer.to_ref(
    map_transformer::Point2D{transformer.robot_map_size().first, 0});
  ASSERT_EQ(transformed.first, transformer.ref_map_size().first);
  ASSERT_EQ(transformed.second, 0);

  transformed = transformer.to_ref(
    map_transformer::Point2D{0, transformer.robot_map_size().second});
  ASSERT_EQ(transformed.first, 0);
  ASSERT_EQ(transformed.second, transformer.ref_map_size().second);

  transformed = transformer.to_ref(
    map_transformer::Point2D{
      transformer.robot_map_size().first,
      transformer.robot_map_size().second});
  ASSERT_EQ(transformed.first, transformer.ref_map_size().first);
  ASSERT_EQ(transformed.second, transformer.ref_map_size().second);
}

TEST_F(TestData, transform_aligned_corners_to_robot) {
  map_transformer::Transformer transformer(AlignedMapYamlDoc());
  auto transformed = transformer.to_robot(
    map_transformer::Point2D{transformer.ref_map_size().first, 0});
  ASSERT_EQ(transformed.first, transformer.robot_map_size().first);
  ASSERT_EQ(transformed.second, 0);

  transformed = transformer.to_robot(
    map_transformer::Point2D{0, transformer.ref_map_size().second});
  ASSERT_EQ(transformed.first, 0);
  ASSERT_EQ(transformed.second, transformer.robot_map_size().second);

  transformed = transformer.to_robot(
    map_transformer::Point2D{
      transformer.ref_map_size().first,
      transformer.ref_map_size().second});
  ASSERT_EQ(transformed.first, transformer.robot_map_size().first);
  ASSERT_EQ(transformed.second, transformer.robot_map_size().second);
}

TEST_F(TestData, transform_aligned_corr_points_to_ref) {
  map_transformer::Transformer transformer(AlignedMapYamlDoc());
  auto transformed = transformer.to_ref(transformer.robot_map_corr_points()[8]);
  ASSERT_EQ(transformed, transformer.ref_map_corr_points()[8]);

  transformed = transformer.to_ref(transformer.robot_map_corr_points()[10]);
  ASSERT_EQ(transformed, transformer.ref_map_corr_points()[10]);
}

TEST_F(TestData, transform_aligned_corr_points_to_robot) {
  map_transformer::Transformer transformer(AlignedMapYamlDoc());
  auto transformed = transformer.to_robot(transformer.ref_map_corr_points()[8]);
  ASSERT_EQ(transformed, transformer.robot_map_corr_points()[8]);

  transformed = transformer.to_robot(transformer.ref_map_corr_points()[10]);
  ASSERT_EQ(transformed, transformer.robot_map_corr_points()[10]);
}

TEST_F(TestData, transform_aligned_midpoints_to_ref) {
  map_transformer::Transformer transformer(AlignedMapYamlDoc());
  auto transformed = transformer.to_ref(map_transformer::Point2D{341, 168});
  ASSERT_FLOAT_EQ(transformed.first, 341);
  ASSERT_FLOAT_EQ(transformed.second, 138.8947);

  transformed = transformer.to_ref(map_transformer::Point2D{433, 252});
  ASSERT_FLOAT_EQ(transformed.first, 433);
  ASSERT_FLOAT_EQ(transformed.second, 189);
}

TEST_F(TestData, transform_aligned_midpoints_to_robot) {
  map_transformer::Transformer transformer(AlignedMapYamlDoc());
  auto transformed = transformer.to_robot(map_transformer::Point2D{341, 138});
  ASSERT_FLOAT_EQ(transformed.first, 341);
  ASSERT_FLOAT_EQ(transformed.second, 167.1053);

  transformed = transformer.to_robot(map_transformer::Point2D{433, 189});
  ASSERT_FLOAT_EQ(transformed.first, 433);
  ASSERT_FLOAT_EQ(transformed.second, 252);
}

TEST_F(TestData, transform_aligned_triangle_edges_to_ref) {
  // Horizontal
  // Start on one side of the edge
  map_transformer::Transformer transformer(AlignedMapYamlDoc());
  auto transformed = transformer.to_ref(map_transformer::Point2D{433, 108});
  ASSERT_FLOAT_EQ(transformed.first, 433);
  ASSERT_FLOAT_EQ(transformed.second, 74.14925);
  // Move one pixel across the edge and ensure it still transforms properly
  transformed = transformer.to_ref(map_transformer::Point2D{432, 108});
  ASSERT_FLOAT_EQ(transformed.first, 432);
  ASSERT_FLOAT_EQ(transformed.second, 74.402199);

  // Vertical
  // Start on one side of the edge
  transformed = transformer.to_ref(map_transformer::Point2D{160, 240});
  ASSERT_FLOAT_EQ(transformed.first, 160);
  ASSERT_FLOAT_EQ(transformed.second, 240);

  // Move one pixel across the edge and ensure it still transforms properly
  transformed = transformer.to_ref(map_transformer::Point2D{160, 241});
  ASSERT_FLOAT_EQ(transformed.first, 160);
  ASSERT_FLOAT_EQ(transformed.second, 241);
}

TEST_F(TestData, transform_aligned_triangle_edges_to_robot) {
  // Horizontal
  // Start on one side of the edge
  map_transformer::Transformer transformer(AlignedMapYamlDoc());
  auto transformed = transformer.to_robot(map_transformer::Point2D{433, 74});
  ASSERT_FLOAT_EQ(transformed.first, 433);
  ASSERT_FLOAT_EQ(transformed.second, 107.7826);
  // Move one pixel across the edge and ensure it still transforms properly
  transformed = transformer.to_robot(map_transformer::Point2D{432, 74});
  ASSERT_FLOAT_EQ(transformed.first, 432);
  ASSERT_FLOAT_EQ(transformed.second, 107.41418);

  // Vertical
  // Start on one side of the edge
  transformed = transformer.to_robot(map_transformer::Point2D{160, 240});
  ASSERT_FLOAT_EQ(transformed.first, 160);
  ASSERT_FLOAT_EQ(transformed.second, 240);

  // Move one pixel across the edge and ensure it still transforms properly
  transformed = transformer.to_robot(map_transformer::Point2D{160, 241});
  ASSERT_FLOAT_EQ(transformed.first, 160);
  ASSERT_FLOAT_EQ(transformed.second, 241);
}

TEST_F(TestData, transform_aligned_triangle_centers_to_ref) {
  map_transformer::Transformer transformer(AlignedMapYamlDoc());
  auto transformed = transformer.to_ref(map_transformer::Point2D{321, 194});
  ASSERT_FLOAT_EQ(transformed.first, 321);
  ASSERT_FLOAT_EQ(transformed.second, 172.2632);

  transformed = transformer.to_ref(map_transformer::Point2D{177, 93});
  ASSERT_FLOAT_EQ(transformed.first, 177);
  ASSERT_FLOAT_EQ(transformed.second, 93);
}




TEST_F(TestData, transform_offset_origins) {
  map_transformer::Transformer transformer(OffsetMapYamlDoc());
  auto transformed = transformer.to_ref(map_transformer::Point2D{0, 0});
  ASSERT_EQ(transformed, transformer.robot_map_translation());

  transformed = transformer.to_robot(map_transformer::Point2D{0, 0});
  ASSERT_EQ(transformed.first, -transformer.robot_map_translation().first);
  ASSERT_EQ(transformed.second, -transformer.robot_map_translation().second);
}

TEST_F(TestData, transform_offset_corners_to_ref) {
  map_transformer::Transformer transformer(OffsetMapYamlDoc());
  auto transformed = transformer.to_ref(
    map_transformer::Point2D{transformer.robot_map_size().first, 0});
  ASSERT_EQ(transformed.first, 110);
  ASSERT_EQ(transformed.second, 20);

  transformed = transformer.to_ref(
    map_transformer::Point2D{0, transformer.robot_map_size().second});
  ASSERT_EQ(transformed.first, 30);
  ASSERT_EQ(transformed.second, 130);

  transformed = transformer.to_ref(
    map_transformer::Point2D{
      transformer.robot_map_size().first,
      transformer.robot_map_size().second});
  ASSERT_EQ(transformed.first, 110);
  ASSERT_EQ(transformed.second, 130);

  transformed = transformer.to_ref(map_transformer::Point2D{70, -20});
  ASSERT_EQ(transformed.first, 100);
  ASSERT_EQ(transformed.second, 0);

  transformed = transformer.to_ref(map_transformer::Point2D{-30, 60});
  ASSERT_EQ(transformed.first, 0);
  ASSERT_EQ(transformed.second, 80);

  transformed = transformer.to_ref(map_transformer::Point2D{70, 60});
  ASSERT_EQ(transformed.first, 100);
  ASSERT_EQ(transformed.second, 80);
}

TEST_F(TestData, transform_offset_corners_to_robot) {
  map_transformer::Transformer transformer(OffsetMapYamlDoc());
  auto transformed = transformer.to_robot(
    map_transformer::Point2D{110, 20});
  ASSERT_EQ(transformed.first, transformer.robot_map_size().first);
  ASSERT_EQ(transformed.second, 0);

  transformed = transformer.to_robot(
    map_transformer::Point2D{30, 130});
  ASSERT_EQ(transformed.first, 0);
  ASSERT_EQ(transformed.second, transformer.robot_map_size().second);

  transformed = transformer.to_robot(map_transformer::Point2D{110, 130});
  auto expected = map_transformer::Point2D{
      transformer.robot_map_size().first,
      transformer.robot_map_size().second};
  ASSERT_EQ(transformed, expected);

  transformed = transformer.to_robot(map_transformer::Point2D{100, 0});
  ASSERT_EQ(transformed.first, 70);
  ASSERT_EQ(transformed.second, -20);

  transformed = transformer.to_robot(map_transformer::Point2D{0, 100});
  ASSERT_EQ(transformed.first, -30);
  ASSERT_EQ(transformed.second, 80);

  transformed = transformer.to_robot(map_transformer::Point2D{100, 100});
  ASSERT_EQ(transformed.first, 70);
  ASSERT_EQ(transformed.second, 80);
}

TEST_F(TestData, transform_offset_corr_points_to_ref) {
  map_transformer::Transformer transformer(OffsetMapYamlDoc());
  auto transformed = transformer.to_ref(transformer.robot_map_corr_points()[1]);
  ASSERT_EQ(transformed, transformer.ref_map_corr_points()[1]);

  transformed = transformer.to_ref(transformer.robot_map_corr_points()[4]);
  ASSERT_EQ(transformed, transformer.ref_map_corr_points()[4]);
}

TEST_F(TestData, transform_offset_corr_points_to_robot) {
  map_transformer::Transformer transformer(OffsetMapYamlDoc());
  auto transformed = transformer.to_robot(transformer.ref_map_corr_points()[1]);
  ASSERT_EQ(transformed, transformer.robot_map_corr_points()[1]);

  transformed = transformer.to_robot(transformer.ref_map_corr_points()[4]);
  ASSERT_EQ(transformed, transformer.robot_map_corr_points()[4]);
}

TEST_F(TestData, transform_offset_midpoints_to_ref) {
  map_transformer::Transformer transformer(OffsetMapYamlDoc());
  auto transformed = transformer.to_ref(map_transformer::Point2D{30, 0});
  ASSERT_FLOAT_EQ(transformed.first, 55);
  ASSERT_FLOAT_EQ(transformed.second, 20);

  transformed = transformer.to_ref(map_transformer::Point2D{25, 53});
  ASSERT_FLOAT_EQ(transformed.first, 55);
  ASSERT_FLOAT_EQ(transformed.second, 70);

  transformed = transformer.to_ref(map_transformer::Point2D{23, 66});
  ASSERT_FLOAT_EQ(transformed.first, 56.209679);
  ASSERT_FLOAT_EQ(transformed.second, 85.51344);
}

TEST_F(TestData, transform_offset_midpoints_to_robot) {
  map_transformer::Transformer transformer(OffsetMapYamlDoc());
  auto transformed = transformer.to_robot(map_transformer::Point2D{55, 20});
  ASSERT_FLOAT_EQ(transformed.first, 30);
  ASSERT_NEAR(transformed.second, 0, 0.0000001);

  transformed = transformer.to_robot(map_transformer::Point2D{55, 70});
  ASSERT_FLOAT_EQ(transformed.first, 25);
  ASSERT_FLOAT_EQ(transformed.second, 53);

  transformed = transformer.to_robot(map_transformer::Point2D{56, 85});
  ASSERT_FLOAT_EQ(transformed.first, 22.89655);
  ASSERT_FLOAT_EQ(transformed.second, 65.547127);
}

TEST_F(TestData, transform_offset_triangle_edges_to_ref) {
  // Horizontal
  // Start on one side of the edge
  map_transformer::Transformer transformer(OffsetMapYamlDoc());
  auto transformed = transformer.to_ref(map_transformer::Point2D{9, 10});
  ASSERT_FLOAT_EQ(transformed.first, 39);
  ASSERT_FLOAT_EQ(transformed.second, 35);
  // Move one pixel across the edge and ensure it still transforms properly
  transformed = transformer.to_ref(map_transformer::Point2D{10, 10});
  ASSERT_FLOAT_EQ(transformed.first, 40);
  ASSERT_FLOAT_EQ(transformed.second, 35);
  // Move one pixel across the edge and ensure it still transforms properly
  transformed = transformer.to_ref(map_transformer::Point2D{11, 10});
  ASSERT_FLOAT_EQ(transformed.first, 40.83333);
  ASSERT_FLOAT_EQ(transformed.second, 35);

  // Vertical
  // Start on one side of the edge
  transformed = transformer.to_ref(map_transformer::Point2D{29, 19});
  ASSERT_FLOAT_EQ(transformed.first, 55.83333);
  ASSERT_FLOAT_EQ(transformed.second, 48.5);
  // Move one pixel across the edge and ensure it still transforms properly
  transformed = transformer.to_ref(map_transformer::Point2D{29, 20});
  ASSERT_FLOAT_EQ(transformed.first, 55.83333);
  ASSERT_FLOAT_EQ(transformed.second, 50);
  // Move one pixel across the edge and ensure it still transforms properly
  transformed = transformer.to_ref(map_transformer::Point2D{29, 21});
  ASSERT_FLOAT_EQ(transformed.first, 55.976189);
  ASSERT_FLOAT_EQ(transformed.second, 50.57143);
}

TEST_F(TestData, transform_offset_triangle_edges_to_robot) {
  // Horizontal
  // Start on one side of the edge
  map_transformer::Transformer transformer(OffsetMapYamlDoc());
  auto transformed = transformer.to_robot(map_transformer::Point2D{39, 35});
  ASSERT_FLOAT_EQ(transformed.first, 9);
  ASSERT_FLOAT_EQ(transformed.second, 10);
  // Move one pixel across the edge and ensure it still transforms properly
  transformed = transformer.to_robot(map_transformer::Point2D{40, 35});
  ASSERT_FLOAT_EQ(transformed.first, 10);
  ASSERT_FLOAT_EQ(transformed.second, 10);
  // Move one pixel across the edge and ensure it still transforms properly
  transformed = transformer.to_robot(map_transformer::Point2D{41, 35});
  ASSERT_FLOAT_EQ(transformed.first, 11.2);
  ASSERT_FLOAT_EQ(transformed.second, 10);

  // Vertical
  // Start on one side of the edge
  transformed = transformer.to_robot(map_transformer::Point2D{55, 48});
  ASSERT_FLOAT_EQ(transformed.first, 28);
  ASSERT_FLOAT_EQ(transformed.second, 18.666667);
  // Move one pixel across the edge and ensure it still transforms properly
  transformed = transformer.to_robot(map_transformer::Point2D{55, 49});
  ASSERT_FLOAT_EQ(transformed.first, 28);
  ASSERT_FLOAT_EQ(transformed.second, 19.33333);
  // Move one pixel across the edge and ensure it still transforms properly
  transformed = transformer.to_robot(map_transformer::Point2D{55, 50});
  ASSERT_FLOAT_EQ(transformed.first, 28);
  ASSERT_FLOAT_EQ(transformed.second, 20);
}

TEST_F(TestData, transform_offset_triangle_centers_to_ref) {
  map_transformer::Transformer transformer(OffsetMapYamlDoc());
  auto transformed = transformer.to_ref(map_transformer::Point2D{23, 13});
  ASSERT_FLOAT_EQ(transformed.first, 50.83333);
  ASSERT_FLOAT_EQ(transformed.second, 39.5);

  transformed = transformer.to_ref(map_transformer::Point2D{33, 31});
  ASSERT_FLOAT_EQ(transformed.first, 60.73809);
  ASSERT_FLOAT_EQ(transformed.second, 56.28571);

  transformed = transformer.to_ref(map_transformer::Point2D{48, 64});
  ASSERT_FLOAT_EQ(transformed.first, 79.90833);
  ASSERT_FLOAT_EQ(transformed.second, 79.22361);
}

TEST_F(TestData, transform_offset_triangle_centers_to_robot) {
  map_transformer::Transformer transformer(OffsetMapYamlDoc());
  auto transformed = transformer.to_robot(map_transformer::Point2D{50, 39});
  ASSERT_FLOAT_EQ(transformed.first, 22);
  ASSERT_FLOAT_EQ(transformed.second, 12.66667);

  transformed = transformer.to_robot(map_transformer::Point2D{60, 56});
  ASSERT_FLOAT_EQ(transformed.first, 32.2);
  ASSERT_FLOAT_EQ(transformed.second, 30.5);

  transformed = transformer.to_robot(map_transformer::Point2D{79, 79});
  ASSERT_FLOAT_EQ(transformed.first, 47.13793);
  ASSERT_FLOAT_EQ(transformed.second, 63.689655);
}

TEST_F(TestData, transform_offset_outside_triangulated_area_to_ref) {
  map_transformer::Transformer transformer(OffsetMapYamlDoc());
  auto transformed = transformer.to_ref(map_transformer::Point2D{69, 0});
  ASSERT_FLOAT_EQ(transformed.first, 99);
  ASSERT_FLOAT_EQ(transformed.second, 20);

  transformed = transformer.to_ref(map_transformer::Point2D{0, 79});
  ASSERT_FLOAT_EQ(transformed.first, 30);
  ASSERT_FLOAT_EQ(transformed.second, 99);

  transformed = transformer.to_ref(map_transformer::Point2D{69, 79});
  ASSERT_FLOAT_EQ(transformed.first, 99);
  ASSERT_FLOAT_EQ(transformed.second, 99);
}

TEST_F(TestData, transform_offset_outside_triangulated_area_to_robot) {
  map_transformer::Transformer transformer(OffsetMapYamlDoc());
  auto transformed = transformer.to_robot(map_transformer::Point2D{99, 99});
  ASSERT_FLOAT_EQ(transformed.first, 69);
  ASSERT_FLOAT_EQ(transformed.second, 79);

  transformed = transformer.to_robot(map_transformer::Point2D{30, 99});
  ASSERT_FLOAT_EQ(transformed.first, 0);
  ASSERT_FLOAT_EQ(transformed.second, 79);

  transformed = transformer.to_robot(map_transformer::Point2D{99, 20});
  ASSERT_FLOAT_EQ(transformed.first, 69);
  ASSERT_FLOAT_EQ(transformed.second, 0);
}

TEST_F(TestData, transform_offset_outside_ref_map_to_ref) {
  map_transformer::Transformer transformer(OffsetMapYamlDoc());
  auto transformed = transformer.to_ref(map_transformer::Point2D{79, 109});
  ASSERT_FLOAT_EQ(transformed.first, 109);
  ASSERT_FLOAT_EQ(transformed.second, 129);

  transformed = transformer.to_ref(map_transformer::Point2D{79, 40});
  ASSERT_FLOAT_EQ(transformed.first, 109);
  ASSERT_FLOAT_EQ(transformed.second, 60);
}

TEST_F(TestData, transform_offset_outside_ref_map_to_robot) {
  map_transformer::Transformer transformer(OffsetMapYamlDoc());
  auto transformed = transformer.to_robot(map_transformer::Point2D{109, 60});
  ASSERT_FLOAT_EQ(transformed.first, 79);
  ASSERT_FLOAT_EQ(transformed.second, 40);

  transformed = transformer.to_robot(map_transformer::Point2D{109, 129});
  ASSERT_FLOAT_EQ(transformed.first, 79);
  ASSERT_FLOAT_EQ(transformed.second, 109);
}

TEST_F(TestData, transform_calculate_bounding_box) {
  map_transformer::Transformer aligned_transformer(AlignedMapYamlDoc());
  std::pair<map_transformer::Point2D, map_transformer::Point2D> expected{
    map_transformer::Point2D{0, 0},
    map_transformer::Point2D{694, 386}};
  ASSERT_EQ(aligned_transformer.bounding_box(), expected);

  map_transformer::Transformer offset_transformer(OffsetMapYamlDoc());
  expected.first.first = 0; expected.first.second = 0;
  expected.second.first = 110; expected.second.second = 130;
  ASSERT_EQ(offset_transformer.bounding_box(), expected);
}
