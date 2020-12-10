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

#include <algorithm>
#include <filesystem>
#include <opencv2/core.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/imgproc.hpp>
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

  loaded._ref_map_name = root["ref_map"]["name"].as<std::string>();
  if (root["ref_map"]["image_file"]) {
    loaded._ref_map_image_file = root["ref_map"]["image_file"].as<std::string>();
  }
  loaded._ref_map_size.first = root["ref_map"]["size"][0].as<int>();
  loaded._ref_map_size.second = root["ref_map"]["size"][1].as<int>();

  loaded._robot_map_name = root["robot_map"]["name"].as<std::string>();
  if (root["robot_map"]["image_file"]) {
    loaded._robot_map_image_file = root["robot_map"]["image_file"].as<std::string>();
  }
  loaded._robot_map_size.first = root["robot_map"]["size"][0].as<int>();
  loaded._robot_map_size.second = root["robot_map"]["size"][1].as<int>();

  if (root["robot_map"]["transform"]) {
    loaded._robot_map_scale.first = root["robot_map"]["transform"]["scale"][0].as<int>();
    loaded._robot_map_scale.second = root["robot_map"]["transform"]["scale"][1].as<int>();
    loaded._robot_map_rotation = root["robot_map"]["transform"]["rotation"].as<double>();
    loaded._robot_map_translation.first =
      root["robot_map"]["transform"]["translation"][0].as<int>();
    loaded._robot_map_translation.second =
      root["robot_map"]["transform"]["translation"][1].as<int>();
  }

  for (auto p : root["ref_map"]["correspondence_points"]) {
    loaded._ref_corr_points.push_back(Point2D{p[0].as<int>(), p[1].as<int>()});
  }
  for (auto p : root["robot_map"]["correspondence_points"]) {
    loaded._robot_corr_points.push_back(Point2D{p[0].as<int>(), p[1].as<int>()});
  }

  // Validate the loaded data
  loaded._validate();
  // All checked out, so claim the data
  *this = loaded;
  // Pre-calculate that which needs to be pre-calculated
  precalculate();
}

void Transformer::reset() {
  _ref_map_name = "";
  _ref_map_image_file = "";
  _ref_map_size = Vector2D{0, 0};
  _robot_map_name = "";
  _robot_map_image_file = "";
  _robot_map_size = Vector2D{0, 0};
  _robot_map_scale = Vector2D{1, 1};
  _robot_map_rotation = 0;
  _robot_map_translation = Vector2D{0, 0};
  _ref_corr_points.clear();
  _robot_corr_points.clear();
  _triangles.clear();
}

std::string Transformer::ref_map_name() const {
  return _ref_map_name;
}

std::string Transformer::ref_map_image_file() const {
  return _ref_map_image_file;
}

Vector2D Transformer::ref_map_size() const {
  return _ref_map_size;
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

const CorrespondencePoints& Transformer::ref_map_corr_points() const {
  return _ref_corr_points;
}

const CorrespondencePoints& Transformer::robot_map_corr_points() const {
  return _robot_corr_points;
}

const TriangleList& Transformer::triangle_indices() const {
  return _triangles;
}

std::pair<Point2D, Point2D> Transformer::bounding_box() const {
  Point2D top_left, bottom_right;
  top_left.first = std::min(0.0f, _robot_map_translation.first);
  top_left.second = std::min(0.0f, _robot_map_translation.second);
  bottom_right.first = std::max(
    _ref_map_size.first,
    _robot_map_size.first + _robot_map_translation.first);
  bottom_right.second = std::max(
    _ref_map_size.second,
    _robot_map_size.second + _robot_map_translation.second);

  return std::pair<Point2D, Point2D>{top_left, bottom_right};
}

Point2D Transformer::to_ref(Point2D const &point) const {
  // Check first it it's a correspondence point because we can shortcircuit much of the
  // calculations for those
  int corr_point_index = get_correspondence_point_index(point, _robot_corr_points);
  if (corr_point_index >= 0) {
    return _ref_corr_points[corr_point_index];
  }

  auto containing_triangle = find_containing_triangle_in_robot(point);

  if (containing_triangle < 0) {
    // No triangle found, so only transform by the map transform
    return transform_to_ref_by_map_transform(point);
  }

  cv::Mat transform = _to_ref_transforms[containing_triangle];
  Point2D transformed_point;
  transformed_point.first = transform.at<double>(0, 0) *
    point.first + transform.at<double>(0, 1) * point.second +
    transform.at<double>(0, 2);
  transformed_point.second = transform.at<double>(1, 0) *
    point.first + transform.at<double>(1, 1) * point.second +
    transform.at<double>(1, 2);
  return transformed_point;
}

Point2D Transformer::to_robot(Point2D const &point) const {
  // Check first it it's a correspondence point because we can shortcircuit much of the
  // calculations for those
  int corr_point_index = get_correspondence_point_index(point, _ref_corr_points);
  if (corr_point_index >= 0) {
    return _robot_corr_points[corr_point_index];
  }

  auto containing_triangle = find_containing_triangle_in_ref(point);

  if (containing_triangle < 0) {
    // No triangle found, so only transform by the map transform
    return transform_from_ref_by_map_transform(point);
  }

  cv::Mat transform = _to_robot_transforms[containing_triangle];
  Point2D transformed_point;
  transformed_point.first = transform.at<double>(0, 0) *
    point.first + transform.at<double>(0, 1) * point.second +
    transform.at<double>(0, 2);
  transformed_point.second = transform.at<double>(1, 0) *
    point.first + transform.at<double>(1, 1) * point.second +
    transform.at<double>(1, 2);
  return transformed_point;
}

bool Transformer::_empty() const {
  return _ref_map_name == "" &&
    _ref_map_image_file == "" &&
    _ref_map_size == Vector2D{0, 0} &&
    _robot_map_name == "" &&
    _robot_map_image_file == "" &&
    _robot_map_size == Vector2D{0, 0} &&
    _robot_map_scale == Vector2D{1, 1} &&
    _robot_map_rotation == 0 &&
    _robot_map_translation == Vector2D{0, 0} &&
    _ref_corr_points.empty() &&
    _robot_corr_points.empty() &&
    _triangles.empty();
}

void Transformer::_validate() const {
  // Must have some correspondence points for all maps
  if (_ref_corr_points.empty()) {
    throw std::runtime_error("No reference map correspondence points provided");
  }
  if (_robot_corr_points.empty()) {
    throw std::runtime_error("No robot map correspondence points provided");
  }
  // Must have an equal number of correspondence points
  if (_ref_corr_points.size() != _robot_corr_points.size()) {
    throw std::runtime_error("Number of reference correspondence points and number of robot "
      "correspondence points do not match");
  }

  // The robot map must at least partly overlap the ref map
  if (_robot_map_translation.first > _ref_map_size.first ||
    _robot_map_translation.second > _ref_map_size.second ||
    (_robot_map_translation.first + _robot_map_size.first) < 0 ||
    (_robot_map_translation.second + _robot_map_size.second) < 0)
  {
    throw std::runtime_error("Reference map and robot map do not overlap");
  }

  // Cannot scale the robot map to zero
  if (_robot_map_scale.first == 0 || _robot_map_scale.second == 0) {
    throw std::runtime_error("Invalid scale value: 0");
  }

  // Map image files must exist
  if (!_ref_map_image_file.empty()) {
    auto path = std::filesystem::path(_ref_map_image_file);
    if (!std::filesystem::exists(path) || !std::filesystem::is_regular_file(path)) {
      throw std::runtime_error("Reference map image file does not exist or is not accessible");
    }
  }
  if (!_robot_map_image_file.empty()) {
    auto path = std::filesystem::path(_robot_map_image_file);
    if (!std::filesystem::exists(path) || !std::filesystem::is_regular_file(path)) {
      throw std::runtime_error("Robot map image file does not exist or is not accessible");
    }
  }

  // Map image file dimensions must match claimed map dimensions
  if (!_ref_map_image_file.empty()) {
    cv::Mat image = cv::imread(_ref_map_image_file, cv::IMREAD_COLOR);
    if (image.empty()) {
      throw std::runtime_error("Reference map image file does not exist or is not accessible");
    }
    if (image.cols != _ref_map_size.first || image.rows != _ref_map_size.second) {
      throw std::runtime_error("Reference map image file dimensions do not match map dimensions");
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


void Transformer::precalculate() {
  subdivide_and_index_triangles();
  precalculate_triangle_transforms();
}


CorrespondencePoints Transformer::calculate_correspondence_midpoints() const {
  CorrespondencePoints midpoints;
  for (CorrespondencePoints::size_type ii = 0; ii < _ref_corr_points.size(); ++ii) {
    auto x = _ref_corr_points[ii].first +
      (_robot_corr_points[ii].first - _ref_corr_points[ii].first) / 2;
    auto y = _ref_corr_points[ii].second +
      (_robot_corr_points[ii].second - _ref_corr_points[ii].second) / 2;
    midpoints.push_back(Point2D{x, y});
  }
  return midpoints;
}


void Transformer::subdivide_and_index_triangles() {
  CorrespondencePoints midpoints = calculate_correspondence_midpoints();
  auto bb = bounding_box();
  auto subdiv = cv::Subdiv2D(cv::Rect(0, 0, bb.second.first, bb.second.second));
  for (auto& p : midpoints) {
    subdiv.insert(cv::Point2f(p.first, p.second));
  }

  std::vector<cv::Vec6f> raw_triangles;
  subdiv.getTriangleList(raw_triangles);
  for (auto& t : raw_triangles) {
    Point2D p0(t[0], t[1]);
    Point2D p1(t[2], t[3]);
    Point2D p2(t[4], t[5]);
    auto offset = std::find(std::begin(midpoints), std::end(midpoints), p0);
    if (offset == std::end(midpoints)) {
      throw std::runtime_error("Could not find expected triangle point");
    }
    unsigned int i0 = std::distance(std::begin(midpoints), offset);
    offset = std::find(std::begin(midpoints), std::end(midpoints), p1);
    if (offset == std::end(midpoints)) {
      throw std::runtime_error("Could not find expected triangle point");
    }
    unsigned int i1 = std::distance(std::begin(midpoints), offset);
    offset = std::find(std::begin(midpoints), std::end(midpoints), p2);
    if (offset == std::end(midpoints)) {
      throw std::runtime_error("Could not find expected triangle point");
    }
    unsigned int i2 = std::distance(std::begin(midpoints), offset);
    _triangles.push_back(Triangle{i0, i1, i2});
  }
}


void Transformer::precalculate_triangle_transforms() {
  for (auto& t : _triangles) {
    cv::Point2f t_ref[3], t_robot[3];

    auto point = _ref_corr_points[std::get<0>(t)];
    t_ref[0].x = point.first; t_ref[0].y = point.second;
    point = _ref_corr_points[std::get<1>(t)];
    t_ref[1].x = point.first; t_ref[1].y = point.second;
    point = _ref_corr_points[std::get<2>(t)];
    t_ref[2].x = point.first; t_ref[2].y = point.second;

    point = _robot_corr_points[std::get<0>(t)];
    t_robot[0].x = point.first; t_robot[0].y = point.second;
    point = _robot_corr_points[std::get<1>(t)];
    t_robot[1].x = point.first; t_robot[1].y = point.second;
    point = _robot_corr_points[std::get<2>(t)];
    t_robot[2].x = point.first; t_robot[2].y = point.second;

    _to_ref_transforms.push_back(cv::getAffineTransform(t_robot, t_ref));
    _to_robot_transforms.push_back(cv::getAffineTransform(t_ref, t_robot));
  }
}

int Transformer::get_correspondence_point_index(
  Point2D const &point,
  CorrespondencePoints const &points) const
{
  auto result = std::find(std::begin(points), std::end(points), point);
  if (result != std::end(points)) {
    return std::distance(std::begin(points), result);
  }
  return -1;
}

// TODO(gbiggs) Refactor this with the below version
int Transformer::find_containing_triangle_in_ref(Point2D const &point) const {
  for (unsigned int ii = 0; ii < _triangles.size(); ++ii) {
    auto triangle = triangle_points(_triangles[ii], _ref_corr_points);
    auto in_triangle = cv::pointPolygonTest(
      triangle,
      cv::Point2f(point.first, point.second),
      false);
    if (in_triangle >= 0) {
      return ii;
    }
  }
  return -1;
}

// TODO(gbiggs) Refactor this with the above version
int Transformer::find_containing_triangle_in_robot(Point2D const &point) const {
  for (unsigned int ii = 0; ii < _triangles.size(); ++ii) {
    auto triangle = triangle_points(_triangles[ii], _robot_corr_points);
    auto in_triangle = cv::pointPolygonTest(
      triangle,
      cv::Point2f(point.first, point.second),
      false);
    if (in_triangle >= 0) {
      return ii;
    }
  }
  return -1;
}


// TODO(gbiggs) Refactor this with the below version
Point2D Transformer::transform_to_ref_by_map_transform(Point2D const& point) const {
  Point2D transformed_point;
  transformed_point.first = point.first * _robot_map_scale.first;
  transformed_point.second = point.second * _robot_map_scale.second;

  if (_robot_map_rotation != 0) {
    Point2D pre(transformed_point);
    transformed_point.first = std::cos(_robot_map_rotation) * pre.first -
      std::sin(_robot_map_rotation) * pre.second;
    transformed_point.second = std::sin(_robot_map_rotation) * pre.first +
      std::cos(_robot_map_rotation) * pre.second;
  }

  transformed_point.first += _robot_map_translation.first;
  transformed_point.second += _robot_map_translation.second;

  return transformed_point;
}


// TODO(gbiggs) Refactor this with the above version
Point2D Transformer::transform_from_ref_by_map_transform(Point2D const& point) const {
  Point2D transformed_point;
  transformed_point.first = point.first / _robot_map_scale.first;
  transformed_point.second = point.second / _robot_map_scale.second;

  if (_robot_map_rotation != 0) {
    Point2D pre(transformed_point);
    transformed_point.first = std::cos(-_robot_map_rotation) * pre.first -
      std::sin(-_robot_map_rotation) * pre.second;
    transformed_point.second = std::sin(-_robot_map_rotation) * pre.first +
      std::cos(-_robot_map_rotation) * pre.second;
  }

  transformed_point.first -= _robot_map_translation.first;
  transformed_point.second -= _robot_map_translation.second;

  return transformed_point;
}


cv::Mat Transformer::triangle_points(
  Triangle const& triangle,
  CorrespondencePoints const& points) const
{
  cv::Mat result(3, 2, CV_32F);

  Point2D point = points[std::get<0>(triangle)];
  result.at<float>(0, 0) = point.first;
  result.at<float>(0, 1) = point.second;

  point = points[std::get<1>(triangle)];
  result.at<float>(1, 0) = point.first;
  result.at<float>(1, 1) = point.second;

  point = points[std::get<2>(triangle)];
  result.at<float>(2, 0) = point.first;
  result.at<float>(2, 1) = point.second;

  return result;
}

}  // namespace map_transformer
