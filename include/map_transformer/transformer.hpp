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
#include "map_transformer/xypair.hpp"

#include <string>
#include <tuple>
#include <vector>

namespace map_transformer {

using Point2D = std::pair<double, double>;
using CorrelationPoints = std::vector<Point2D>;
using Vector2D = std::pair<double, double>;

class Transformer {
public:
  Transformer();

  explicit Transformer(std::string const &yaml_doc);

  virtual ~Transformer() {};

  void load(std::string const &yaml_doc);

  void reset();

  std::string base_map_name() const;

  std::string base_map_image_file() const;

  Vector2D base_map_size() const;

  std::string robot_map_name() const;

  std::string robot_map_image_file() const;

  Vector2D robot_map_size() const;

  Vector2D robot_map_scale() const;

  double robot_map_rotation() const;

  Vector2D robot_map_translation() const;

  const CorrelationPoints& base_map_corr_points() const;

  const CorrelationPoints& robot_map_corr_points() const;

private:
  std::string _base_map_name;
  std::string _base_map_image_file;
  Vector2D _base_map_size;
  std::string _robot_map_name;
  std::string _robot_map_image_file;
  Vector2D _robot_map_size;
  Vector2D _robot_map_scale;
  double _robot_map_rotation;
  Vector2D _robot_map_translation;
  CorrelationPoints _base_corr_points;
  CorrelationPoints _robot_corr_points;

  bool _empty() const;
  void _validate() const;
};

}  // namespace map_transformer

#endif  // MAP_TRANSFORMER__TRANSFORMER_HPP_
