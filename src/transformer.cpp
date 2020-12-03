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

#include "map_transformer/transformer.hpp"

#include <filesystem>
#include <opencv2/core.hpp>
#include <opencv2/imgcodecs.hpp>
#include <stdexcept>
#include <yaml-cpp/yaml.h>


namespace map_transformer
{

Transformer::Transformer() {
  reset();
}

Transformer::Transformer(std::string const &yaml_doc) {
  reset();
  load(yaml_doc);
}

void Transformer::load(std::string const &yaml_doc) {
  // Check first that this transformer is empty
  if (!_empty()) {
    throw std::logic_error("Transformer must be empty prior to calling load()");
  }

  Transformer loaded;
  YAML::Node root = YAML::Load(yaml_doc);

  loaded._base_map_name = root["base_map"]["name"].as<std::string>();
  if (root["base_map"]["image_file"]) {
    loaded._base_map_image_file = root["base_map"]["image_file"].as<std::string>();
  }
  loaded._base_map_size.first = root["base_map"]["size"][0].as<double>();
  loaded._base_map_size.second = root["base_map"]["size"][1].as<double>();

  loaded._robot_map_name = root["robot_map"]["name"].as<std::string>();
  if (root["robot_map"]["image_file"]) {
    loaded._robot_map_image_file = root["robot_map"]["image_file"].as<std::string>();
  }
  loaded._robot_map_size.first = root["robot_map"]["size"][0].as<double>();
  loaded._robot_map_size.second = root["robot_map"]["size"][1].as<double>();

  if (root["robot_map"]["transform"]) {
    loaded._robot_map_scale.first = root["robot_map"]["transform"]["scale"][0].as<double>();
    loaded._robot_map_scale.second = root["robot_map"]["transform"]["scale"][1].as<double>();
    loaded._robot_map_rotation = root["robot_map"]["transform"]["rotation"].as<double>();
    loaded._robot_map_translation.first =
      root["robot_map"]["transform"]["translation"][0].as<double>();
    loaded._robot_map_translation.second =
      root["robot_map"]["transform"]["translation"][1].as<double>();
  }

  for (auto p : root["base_map"]["correspondence_points"]) {
    loaded._base_corr_points.push_back(Point2D{p[0].as<double>(), p[1].as<double>()});
  }
  for (auto p : root["robot_map"]["correspondence_points"]) {
    loaded._robot_corr_points.push_back(Point2D{p[0].as<double>(), p[1].as<double>()});
  }

  // Validate the loaded data
  loaded._validate();
  // All checked out, so claim the data
  *this = loaded;
}

void Transformer::reset() {
  _base_map_name = "";
  _base_map_image_file = "";
  _base_map_size = Vector2D{0, 0};
  _robot_map_name = "";
  _robot_map_image_file = "";
  _robot_map_size = Vector2D{0, 0};
  _robot_map_scale = Vector2D{1, 1};
  _robot_map_rotation = 0;
  _robot_map_translation = Vector2D{0, 0};
  _base_corr_points.clear();
  _robot_corr_points.clear();
}

std::string Transformer::base_map_name() const {
  return _base_map_name;
}

std::string Transformer::base_map_image_file() const {
  return _base_map_image_file;
}

Vector2D Transformer::base_map_size() const {
  return _base_map_size;
}

std::string Transformer::robot_map_name() const {
  return _robot_map_name;
}

std::string Transformer::robot_map_image_file() const {
  return _robot_map_image_file;
}

Vector2D Transformer::robot_map_size() const {
  return _robot_map_size;
}

Vector2D Transformer::robot_map_scale() const {
  return _robot_map_scale;
}

double Transformer::robot_map_rotation() const {
  return _robot_map_rotation;
}

Vector2D Transformer::robot_map_translation() const {
  return _robot_map_translation;
}

const CorrelationPoints& Transformer::base_map_corr_points() const {
  return _base_corr_points;
}

const CorrelationPoints& Transformer::robot_map_corr_points() const {
  return _robot_corr_points;
}

bool Transformer::_empty() const {
  return _base_map_name == "" &&
    _base_map_image_file == "" &&
    _base_map_size == Vector2D{0, 0} &&
    _robot_map_name == "" &&
    _robot_map_image_file == "" &&
    _robot_map_size == Vector2D{0, 0} &&
    _robot_map_scale == Vector2D{1, 1} &&
    _robot_map_rotation == 0 &&
    _robot_map_translation == Vector2D{0, 0} &&
    _base_corr_points.empty() &&
    _robot_corr_points.empty();
}

void Transformer::_validate() const {
  // Must have some correspondence points for all maps
  if (_base_corr_points.empty()) {
    throw std::runtime_error("No base map correspondence points provided");
  }
  if (_robot_corr_points.empty()) {
    throw std::runtime_error("No robot map correspondence points provided");
  }
  // Must have an equal number of correspondence points
  if (_base_corr_points.size() != _robot_corr_points.size()) {
    throw std::runtime_error("Number of base correspondence points and number of robot "
      "correspondence points do not match");
  }

  // The robot map must at least partly overlap the base map
  if (_robot_map_translation.first > _base_map_size.first ||
    _robot_map_translation.second > _base_map_size.second ||
    (_robot_map_translation.first + _robot_map_size.first) < 0 ||
    (_robot_map_translation.second + _robot_map_size.second) < 0)
  {
    throw std::runtime_error("Base map and robot map do not overlap");
  }

  // Cannot scale the robot map to zero
  if (_robot_map_scale.first == 0 || _robot_map_scale.second == 0) {
    throw std::runtime_error("Invalid scale value: 0");
  }

  // Map image files must exist
  if (!_base_map_image_file.empty()) {
    auto path = std::filesystem::path(_base_map_image_file);
    if (!std::filesystem::exists(path) || !std::filesystem::is_regular_file(path)) {
      throw std::runtime_error("Base map image file does not exist or is not accessible");
    }
  }
  if (!_robot_map_image_file.empty()) {
    auto path = std::filesystem::path(_robot_map_image_file);
    if (!std::filesystem::exists(path) || !std::filesystem::is_regular_file(path)) {
      throw std::runtime_error("Robot map image file does not exist or is not accessible");
    }
  }

  // Map image file dimensions must match claimed map dimensions
  if (!_base_map_image_file.empty()) {
    cv::Mat image = cv::imread(_base_map_image_file, cv::IMREAD_COLOR);
    if (image.empty()) {
      throw std::runtime_error("Base map image file does not exist or is not accessible");
    }
    if (image.cols != _base_map_size.first || image.rows != _base_map_size.second) {
      throw std::runtime_error("Base map image file dimensions do not match map dimensions");
    }
  }
  if (!_robot_map_image_file.empty()) {
    cv::Mat image = cv::imread(_robot_map_image_file, cv::IMREAD_COLOR);
    if (image.empty()) {
      throw std::runtime_error("Robot map image file does not exist or is not accessible");
    }
    if (image.cols != _robot_map_size.first || image.rows != _robot_map_size.second) {
      throw std::runtime_error("Robot map image file dimensions do not match map dimensions");
    }
  }
}

}  // namespace map_transformer
