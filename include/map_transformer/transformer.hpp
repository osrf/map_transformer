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

#ifndef MAP_TRANSFORMER__TRANSFORMER_HPP_
#define MAP_TRANSFORMER__TRANSFORMER_HPP_

#include "map_transformer/visibility_control.h"

#include <opencv2/imgproc.hpp>
#include <string>
#include <tuple>
#include <vector>

namespace map_transformer {

using Point2D = std::pair<float, float>;
using CorrespondencePoints = std::vector<Point2D>;
using Vector2D = std::pair<float, float>;
using Triangle = std::tuple<int, int, int>;
using TriangleList = std::vector<Triangle>;

class Transformer {
public:
  Transformer();

  explicit Transformer(std::string const &yaml_doc);

  virtual ~Transformer() {};

  void load(std::string const &yaml_doc);

  void reset();

  std::string ref_map_name() const;

  std::string ref_map_image_file() const;

  Vector2D ref_map_size() const;

  std::string robot_map_name() const;

  std::string robot_map_image_file() const;

  Vector2D robot_map_size() const;

  Vector2D robot_map_scale() const;

  double robot_map_rotation() const;

  Point2D robot_map_translation() const;

  const CorrespondencePoints& ref_map_corr_points() const;

  const CorrespondencePoints& robot_map_corr_points() const;

  const TriangleList& triangle_indices() const;

  std::pair<Point2D, Point2D> bounding_box() const;

  Point2D to_ref(Point2D const &point) const;

  Point2D to_robot(Point2D const &point) const;

private:
  // Loaded data
  std::string _ref_map_name;
  std::string _ref_map_image_file;
  Vector2D _ref_map_size;
  std::string _robot_map_name;
  std::string _robot_map_image_file;
  Vector2D _robot_map_size;
  Vector2D _robot_map_scale;
  double _robot_map_rotation;
  Vector2D _robot_map_translation;
  CorrespondencePoints _ref_corr_points;
  CorrespondencePoints _robot_corr_points;

  // Loaded data management
  bool _empty() const;
  void _validate() const;

  // Pre-calculated data for performing transforms
  TriangleList _triangles;
  std::vector<cv::Mat> _to_ref_transforms;
  std::vector<cv::Mat> _to_robot_transforms;

  // Transformation support
  void precalculate();
  CorrespondencePoints calculate_correspondence_midpoints() const;
  void subdivide_and_index_triangles();
  void precalculate_triangle_transforms();
  int get_correspondence_point_index(
    Point2D const &point,
    CorrespondencePoints const &points) const;
  int find_containing_triangle_in_ref(Point2D const &point) const;
  int find_containing_triangle_in_robot(Point2D const &point) const;
  Point2D transform_to_ref_by_map_transform(Point2D const& point) const;
  Point2D transform_from_ref_by_map_transform(Point2D const& point) const;
  cv::Mat triangle_points(
    Triangle const& triangle,
    CorrespondencePoints const& points) const;
};

}  // namespace map_transformer

#endif  // MAP_TRANSFORMER__TRANSFORMER_HPP_
