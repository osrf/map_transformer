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

#include <gtest/gtest.h>
#include <yaml-cpp/yaml.h>

#include <stdexcept>
#include <string>

using map_transformer::test::TEST_DATA_DIRECTORY;


class TestData : public ::testing::Test {
protected:
  const std::string CorrectYamlDoc() {
    return std::string(
R"(ref_map:
  name: ref
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
  name: test_map_distorted
  image_file: )" + TEST_DATA_DIRECTORY + R"(/aligned_map_robot.png
  size: [694, 386]
  transform:
    scale: [1, 1]
    rotation: 0
    translation: [0, 0]
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

  const std::string NotAYamlDoc() {
    return "This is not a YAML document.";
  }

  const std::string NonOverlappingYamlDoc() {
    return std::string(
R"(ref_map:
  name: ref
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
  name: test_map_distorted
  image_file: )" + TEST_DATA_DIRECTORY + R"(/aligned_map_robot.png
  size: [694, 386]
  transform:
    scale: [1, 1]
    rotation: 0
    translation: [10000, 10000]
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

  const std::string NoRefCorrPointsYamlDoc() {
    return std::string(
R"(ref_map:
  name: ref
  image_file: )") + TEST_DATA_DIRECTORY + R"(/aligned_map_ref.png
  size: [694, 386]
  correspondence_points:
robot_map:
  name: test_map_distorted
  image_file: )" + TEST_DATA_DIRECTORY + R"(/aligned_map_robot.png
  size: [694, 386]
  transform:
    scale: [1, 1]
    rotation: 0
    translation: [0, 0]
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

  const std::string NoRobotCorrPointsYamlDoc() {
    return std::string(
R"(ref_map:
  name: ref
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
  name: test_map_distorted
  image_file: )" + TEST_DATA_DIRECTORY + R"(/aligned_map_robot.png
  size: [694, 386]
  transform:
    scale: [1, 1]
    rotation: 0
    translation: [0, 0])";
  }

  const std::string DifferentNumCorrPointsYamlDoc() {
    return std::string(
R"(ref_map:
  name: ref
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
  name: test_map_distorted
  image_file: )" + TEST_DATA_DIRECTORY + R"(/aligned_map_robot.png
  size: [694, 386]
  transform:
    scale: [1, 1]
    rotation: 0
    translation: [0, 0]
  correspondence_points:
    - [0, 138]
    - [0, 241]
    - [433, 384]
    - [692, 138]
    - [692, 241]
    - [262, 138]
    - [262, 241]
    - [433, 201]
    - [433, 304])";
  }

  const std::string NoAffineTransformYamlDoc() {
    return std::string(
R"(ref_map:
  name: ref
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
  name: test_map_distorted
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

  const std::string ZeroXScaleAffineTransformYamlDoc() {
    return std::string(
R"(ref_map:
  name: ref
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
  name: test_map_distorted
  image_file: )" + TEST_DATA_DIRECTORY + R"(/aligned_map_robot.png
  size: [694, 386]
  transform:
    scale: [0, 1]
    rotation: 0
    translation: [0, 0]
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

  const std::string ZeroYScaleAffineTransformYamlDoc() {
    return std::string(
R"(ref_map:
  name: ref
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
  name: test_map_distorted
  image_file: )" + TEST_DATA_DIRECTORY + R"(/aligned_map_robot.png
  size: [694, 386]
  transform:
    scale: [1, 0]
    rotation: 0
    translation: [0, 0]
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

  const std::string ZeroBothScaleAffineTransformYamlDoc() {
    return std::string(
R"(ref_map:
  name: ref
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
  name: test_map_distorted
  image_file: )" + TEST_DATA_DIRECTORY + R"(/aligned_map_robot.png
  size: [694, 386]
  transform:
    scale: [0, 0]
    rotation: 0
    translation: [0, 0]
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

  const std::string NoRefMapImageYamlDoc() {
    return std::string(
R"(ref_map:
  name: ref
  size: [694, 386]
  correspondence_points:
    - [0, 138]
    - [0, 241]
    - [262, 0]
    - [262, 384]
robot_map:
  name: test_map_distorted
  image_file: )") + TEST_DATA_DIRECTORY + R"(/aligned_map_robot.png
  size: [694, 386]
  transform:
    scale: [1, 1]
    rotation: 0
    translation: [0, 0]
  correspondence_points:
    - [0, 138]
    - [0, 241]
    - [262, 0]
    - [262, 384])";
  }

  const std::string NoRobotMapImageYamlDoc() {
    return std::string(
R"(ref_map:
  name: ref
  image_file: )") + TEST_DATA_DIRECTORY + R"(/aligned_map_ref.png
  size: [694, 386]
  correspondence_points:
    - [0, 138]
    - [0, 241]
    - [262, 0]
    - [262, 384]
robot_map:
  name: test_map_distorted
  size: [694, 386]
  transform:
    scale: [1, 1]
    rotation: 0
    translation: [0, 0]
  correspondence_points:
    - [0, 138]
    - [0, 241]
    - [262, 0]
    - [262, 384])";
  }

  const std::string NoRefMapSizeYamlDoc() {
    return std::string(
R"(ref_map:
  name: ref
  image_file: )") + TEST_DATA_DIRECTORY + R"(/aligned_map_ref.png
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
  name: test_map_distorted
  image_file: )" + TEST_DATA_DIRECTORY + R"(/aligned_map_robot.png
  size: [694, 386]
  transform:
    scale: [1, 1]
    rotation: 0
    translation: [0, 0]
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

  const std::string NoRobotMapSizeYamlDoc() {
    return std::string(
R"(ref_map:
  name: ref
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
  name: test_map_distorted
  image_file: )" + TEST_DATA_DIRECTORY + R"(/aligned_map_robot.png
  transform:
    scale: [1, 1]
    rotation: 0
    translation: [0, 0]
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

  const std::string YamlAndRefImageDiffSizesYamlDoc() {
    return std::string(
R"(ref_map:
  name: ref
  image_file: )") + TEST_DATA_DIRECTORY + R"(/aligned_map_ref.png
  size: [594, 286]
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
  name: test_map_distorted
  image_file: )" + TEST_DATA_DIRECTORY + R"(/aligned_map_robot.png
  size: [694, 386]
  transform:
    scale: [1, 1]
    rotation: 0
    translation: [0, 0]
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

  const std::string YamlAndRobotImageDiffSizesYamlDoc() {
    return std::string(
R"(ref_map:
  name: ref
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
  name: test_map_distorted
  image_file: )" + TEST_DATA_DIRECTORY + R"(/aligned_map_robot.png
  size: [594, 286]
  transform:
    scale: [1, 1]
    rotation: 0
    translation: [0, 0]
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

  const std::string RefMapImageFileDoesntExistYamlDoc() {
    return std::string(
R"(ref_map:
  name: ref
  image_file: )") + TEST_DATA_DIRECTORY + R"(/nonexistent.png
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
  name: test_map_distorted
  image_file: )" + TEST_DATA_DIRECTORY + R"(/aligned_map_robot.png
  size: [694, 386]
  transform:
    scale: [1, 1]
    rotation: 0
    translation: [0, 0]
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

  const std::string RobotMapImageFileDoesntExistYamlDoc() {
    return std::string(
R"(ref_map:
  name: ref
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
  name: test_map_distorted
  image_file: )" + TEST_DATA_DIRECTORY + R"(/nonexistent.png
  size: [694, 386]
  transform:
    scale: [1, 1]
    rotation: 0
    translation: [0, 0]
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
};


void assert_loaded_data_equal_to_yaml(
  map_transformer::Transformer &transformer,
  std::string const &yaml_doc)
{
  YAML::Node root = YAML::Load(yaml_doc);
  ASSERT_EQ(transformer.ref_map_name(), root["ref_map"]["name"].as<std::string>());
  ASSERT_EQ(transformer.ref_map_image_file(), root["ref_map"]["image_file"].as<std::string>());
  map_transformer::Vector2D size;
  size.first = root["ref_map"]["size"][0].as<double>();
  size.second = root["ref_map"]["size"][1].as<double>();
  ASSERT_EQ(transformer.ref_map_size(), size);
  ASSERT_EQ(transformer.robot_map_name(), root["robot_map"]["name"].as<std::string>());
  ASSERT_EQ(transformer.robot_map_image_file(), root["robot_map"]["image_file"].as<std::string>());
  size.first = root["robot_map"]["size"][0].as<double>();
  size.second = root["robot_map"]["size"][1].as<double>();
  ASSERT_EQ(transformer.robot_map_size(), size);
  map_transformer::Vector2D scale;
  scale.first = root["robot_map"]["transform"]["scale"][0].as<double>();
  scale.second = root["robot_map"]["transform"]["scale"][1].as<double>();
  ASSERT_EQ(transformer.robot_map_scale(), scale);
  double rotation = root["robot_map"]["transform"]["rotation"].as<double>();
  ASSERT_EQ(transformer.robot_map_rotation(), rotation);
  map_transformer::Point2D translation;
  translation.first = root["robot_map"]["transform"]["translation"][0].as<double>();
  translation.second = root["robot_map"]["transform"]["translation"][1].as<double>();
  ASSERT_EQ(transformer.robot_map_translation(), translation);
  auto index = 0;
  for (auto p : root["ref_map"]["correspondence_points"]) {
    double x = p[0].as<double>();
    double y = p[1].as<double>();
    ASSERT_TRUE(transformer.ref_map_corr_points()[index].first == x &&
        transformer.ref_map_corr_points()[index].second == y);
    ++index;
  }
  index = 0;
  for (auto p : root["robot_map"]["correspondence_points"]) {
    double x = p[0].as<double>();
    double y = p[1].as<double>();
    ASSERT_TRUE(transformer.robot_map_corr_points()[index].first == x &&
        transformer.robot_map_corr_points()[index].second == y);
    ++index;
  }
}


TEST_F(TestData, load_correct_constructor) {
  ASSERT_NO_THROW(map_transformer::Transformer transformer(CorrectYamlDoc()));

  map_transformer::Transformer transformer(CorrectYamlDoc());
  assert_loaded_data_equal_to_yaml(transformer, CorrectYamlDoc());
}

TEST_F(TestData, load_correct_method) {
  map_transformer::Transformer transformer;
  ASSERT_NO_THROW(transformer.load(CorrectYamlDoc()));
  assert_loaded_data_equal_to_yaml(transformer, CorrectYamlDoc());

  ASSERT_THROW(transformer.load(CorrectYamlDoc()), std::logic_error);
  assert_loaded_data_equal_to_yaml(transformer, CorrectYamlDoc());

  transformer.reset();
  ASSERT_NO_THROW(transformer.load(CorrectYamlDoc()));
  assert_loaded_data_equal_to_yaml(transformer, CorrectYamlDoc());
}

TEST_F(TestData, load_reset) {
  map_transformer::Transformer transformer(CorrectYamlDoc());
  assert_loaded_data_equal_to_yaml(transformer, CorrectYamlDoc());

  transformer.reset();

  ASSERT_THROW(transformer.ref_map_name(), std::logic_error);
  ASSERT_THROW(transformer.ref_map_image_file(), std::logic_error);
  ASSERT_THROW(transformer.ref_map_size(), std::logic_error);
  ASSERT_THROW(transformer.robot_map_name(), std::logic_error);
  ASSERT_THROW(transformer.robot_map_image_file(), std::logic_error);
  ASSERT_THROW(transformer.robot_map_size(), std::logic_error);
  ASSERT_THROW(transformer.robot_map_scale(), std::logic_error);
  ASSERT_THROW(transformer.robot_map_rotation(), std::logic_error);
  ASSERT_THROW(transformer.robot_map_translation(), std::logic_error);
  ASSERT_THROW(transformer.ref_map_corr_points(), std::logic_error);
  ASSERT_THROW(transformer.robot_map_corr_points(), std::logic_error);
  ASSERT_THROW(transformer.triangle_indices(), std::logic_error);
  ASSERT_THROW(transformer.bounding_box(), std::logic_error);
  map_transformer::Point2D point;
  ASSERT_THROW(transformer.to_ref(point), std::logic_error);
  ASSERT_THROW(transformer.to_robot(point), std::logic_error);
}

TEST_F(TestData, load_not_a_yaml_doc) {
  ASSERT_THROW(
      map_transformer::Transformer transformer(NotAYamlDoc()),
      std::runtime_error);
}

TEST_F(TestData, load_nonoverlapping) {
  ASSERT_THROW(
    map_transformer::Transformer transformer(NonOverlappingYamlDoc()),
    std::runtime_error);
}

TEST_F(TestData, load_no_ref_coor_points) {
  ASSERT_THROW(
    map_transformer::Transformer transformer(NoRefCorrPointsYamlDoc()),
    std::runtime_error);
}

TEST_F(TestData, load_no_robot_coor_points) {
  ASSERT_THROW(
    map_transformer::Transformer transformer(NoRobotCorrPointsYamlDoc()),
    std::runtime_error);
}

TEST_F(TestData, load_different_number_of_coor_points) {
  ASSERT_THROW(
    map_transformer::Transformer transformer(DifferentNumCorrPointsYamlDoc()),
    std::runtime_error);
}

TEST_F(TestData, load_no_affine_transform) {
  ASSERT_NO_THROW(map_transformer::Transformer transformer(NoAffineTransformYamlDoc()));
  map_transformer::Transformer transformer(NoAffineTransformYamlDoc());
  map_transformer::Vector2D scale{1, 1};
  ASSERT_EQ(transformer.robot_map_scale(), scale);
  ASSERT_EQ(transformer.robot_map_rotation(), 0);
  map_transformer::Point2D translation{0, 0};
  ASSERT_EQ(transformer.robot_map_translation(), translation);
}

TEST_F(TestData, load_zero_scale_affine_transform) {
  ASSERT_THROW(
    map_transformer::Transformer transformer(ZeroXScaleAffineTransformYamlDoc()),
    std::runtime_error);
  ASSERT_THROW(
    map_transformer::Transformer transformer(ZeroYScaleAffineTransformYamlDoc()),
    std::runtime_error);
  ASSERT_THROW(
    map_transformer::Transformer transformer(ZeroBothScaleAffineTransformYamlDoc()),
    std::runtime_error);
}

TEST_F(TestData, load_no_ref_map_image) {
  ASSERT_NO_THROW(map_transformer::Transformer transformer(NoRefMapImageYamlDoc()));
  map_transformer::Transformer transformer(NoRefMapImageYamlDoc());
  ASSERT_EQ(transformer.ref_map_image_file(), "");
}

TEST_F(TestData, load_no_robot_map_image) {
  ASSERT_NO_THROW(map_transformer::Transformer transformer(NoRobotMapImageYamlDoc()));
  map_transformer::Transformer transformer(NoRobotMapImageYamlDoc());
  ASSERT_EQ(transformer.robot_map_image_file(), "");
}

TEST_F(TestData, load_no_ref_map_size) {
  ASSERT_THROW(
    map_transformer::Transformer transformer(NoRefMapSizeYamlDoc()),
    std::runtime_error);
}

TEST_F(TestData, load_no_robot_map_size) {
  ASSERT_THROW(
    map_transformer::Transformer transformer(NoRobotMapSizeYamlDoc()),
    std::runtime_error);
}

TEST_F(TestData, load_yaml_and_ref_image_sizes_differ) {
  ASSERT_THROW(
    map_transformer::Transformer transformer(YamlAndRefImageDiffSizesYamlDoc()),
    std::runtime_error);
}

TEST_F(TestData, load_yaml_and_robot_image_sizes_differ) {
  ASSERT_THROW(
    map_transformer::Transformer transformer(YamlAndRobotImageDiffSizesYamlDoc()),
    std::runtime_error);
}

TEST_F(TestData, load_nonexistent_ref_map_image_file) {
  ASSERT_THROW(
    map_transformer::Transformer transformer(RefMapImageFileDoesntExistYamlDoc()),
    std::runtime_error);
}

TEST_F(TestData, load_nonexistent_robot_map_image_file) {
  ASSERT_THROW(
    map_transformer::Transformer transformer(RobotMapImageFileDoesntExistYamlDoc()),
    std::runtime_error);
}

TEST(TestLoading, load_no_data_is_logic_error) {
  map_transformer::Transformer transformer;

  ASSERT_THROW(transformer.ref_map_name(), std::logic_error);
  ASSERT_THROW(transformer.ref_map_image_file(), std::logic_error);
  ASSERT_THROW(transformer.ref_map_size(), std::logic_error);
  ASSERT_THROW(transformer.robot_map_name(), std::logic_error);
  ASSERT_THROW(transformer.robot_map_image_file(), std::logic_error);
  ASSERT_THROW(transformer.robot_map_size(), std::logic_error);
  ASSERT_THROW(transformer.robot_map_scale(), std::logic_error);
  ASSERT_THROW(transformer.robot_map_rotation(), std::logic_error);
  ASSERT_THROW(transformer.robot_map_translation(), std::logic_error);
  ASSERT_THROW(transformer.ref_map_corr_points(), std::logic_error);
  ASSERT_THROW(transformer.robot_map_corr_points(), std::logic_error);
  ASSERT_THROW(transformer.triangle_indices(), std::logic_error);
  ASSERT_THROW(transformer.bounding_box(), std::logic_error);
  map_transformer::Point2D point;
  ASSERT_THROW(transformer.to_ref(point), std::logic_error);
  ASSERT_THROW(transformer.to_robot(point), std::logic_error);
}
