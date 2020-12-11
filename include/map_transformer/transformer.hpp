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

/// The Transformer class provides transformation of points between two maps.
/**
 * The maps are related by a non-linear transformation. In other words, the relation between two
 * equivalent points in one part of the map is not necessarily the same as between two other
 * equivalent points elsewhere in the map.
 */
class Transformer {
public:
  /// Create a new empty transformer object.
  Transformer();

  /// Create a new transformer object and load map information from the provided YAML document.
  /**
   * \param[in] yaml_doc The YAML document to load map information from. Must conform to the
   * required format.
   * \sa Transformer::load()
   * \throws std::RuntimeError if there is an error translating the YAML document.
   */
  explicit Transformer(std::string const &yaml_doc);

  virtual ~Transformer() {};

  /// Load map information from the provided YAML document.
  /**
   * \pre The \ref Transformer object must be empty (must not already contain map information)
   * before calling \ref Transformer::load(). Call \ref Transformer::reset() to clear a \ref
   * Transformer instance prior to loading new map information. Transformer object instances are
   * empty when first constructed.
   * \param[in] yaml_doc The YAML document to load map information from. Must conform to the
   * required format.
   * \throws std::RuntimeError if there is an error translating the YAML document.
   * \throws std::LogicError if the Transformer is not empty.
   */
  void load(std::string const &yaml_doc);

  /// Clear any loaded map information.
  void reset();

  /// Get the name of the reference map that is loaded.
  /**
   * \return The name of the reference map, as loaded from the YAML document.
   */
  std::string ref_map_name() const;

  /// Get the path to the image file for the reference map, if there is one.
  /**
   * \return The path to the image file for the reference map, or an empty string if no image file
   * is available.
   * \throw std::LogicError if the Transformer has no loaded map information.
   */
  std::string ref_map_image_file() const;

  /// Get the dimensions of the reference map.
  /**
   * The dimensions are measured in arbitrary units, equivalent to the number of pixels (i.e. the
   * image resolution) in the reference map image if there is one.
   *
   * \return The dimensions of the reference map.
   * \throw std::LogicError if the Transformer has no loaded map information.
   */
  Vector2D ref_map_size() const;

  /// Get the name of the robot map that is loaded.
  /**
   * \return A string containing the name of the robot map, as loaded from the YAML document.
   * \throw std::LogicError if the Transformer has no loaded map information.
   */
  std::string robot_map_name() const;

  /// Get the path to the image file for the robot map, if there is one.
  /**
   * \return The path to the image file for the robot map, or an empty string if no image file
   * is available.
   * \throw std::LogicError if the Transformer has no loaded map information.
   */
  std::string robot_map_image_file() const;

  /// Get the dimensions of the robot map.
  /**
   * The dimensions are measured in arbitrary units, equivalent to the number of pixels (i.e. the
   * image resolution) in the robot map image if there is one.
   *
   * \return The dimensions of the robot map, as a \ref Vector2D.
   * \throw std::LogicError if the Transformer has no loaded map information.
   */
  Vector2D robot_map_size() const;

  /// Get the relative scale of the robot map to the reference map.
  /**
   * The robot map may be at a different scale to the reference map. The scale is loaded from the
   * YAML document, and is provided as a value with 1 meaning equal scale, less than 1 meaning the
   * robot map scale is smaller than the reference map, and greater than one meaning the robot
   * map scale is larger than the reference map. The X and Y values can be used as a scaling
   * transformation between the reference map and the robot map.
   *
   * \return The relative scale of the robot map to the reference map, as an X,Y pair.
   * \throw std::LogicError if the Transformer has no loaded map information.
   */
  Vector2D robot_map_scale() const;

  /// Get the relative rotation of the robot map around the reference map's origin.
  /**
   * The robot map may be rotated relative to the reference map. The rotation is loaded from the
   * YAML document, and is provided via this member as an angle in radians. This value can be used
   * to construct a rotational transform from the reference map to the robot map and vice versa.
   *
   * \return The relative rotation of the robot map to the reference map, as an angle in radians.
   * \throw std::LogicError if the Transformer has no loaded map information.
   */
  double robot_map_rotation() const;

  /// Get the relative translation of the robot map from the reference map's origin.
  /**
   * The robot map may be offset from the reference map. The offset is loaded from the YAML
   * document, and is provided via this member as an offset in X and Y. This value can be used to
   * construct a translational transform from the reference map to the robot map and vice versa.
   *
   * \return The translation from the reference map's origin to the robot map's origin.
   * \throw std::LogicError if the Transformer has no loaded map information.
   */
  Point2D robot_map_translation() const;

  /// Get the list of correspondence points in the reference map.
  /**
   * The correspondence points in the reference map are one-to-one matched to the correspondence
   * points in the robot map. This means that each entry in this list is matched to its
   * same-indexed entry in the list provided by \ref robot_map_corr_points(). For
   * example, the point in this list at index 5 is matched to the point in the robot map
   * correspondence points list at index 5.
   *
   * This list is provided for visualisation and debugging purposes.
   *
   * \return The list of correspondence points in the reference map.
   * \throw std::LogicError if the Transformer has no loaded map information.
   */
  const CorrespondencePoints& ref_map_corr_points() const;

  /// Get the list of correspondence points in the robot map.
  /**
   * The correspondence points in the robot map are one-to-one matched to the correspondence points
   * in the reference map. This means that each entry in this list is matched to its same-indexed
   * entry in the list provided by \ref ref_map_corr_points(). For example, the point in
   * this list at index 5 is matched to the point in the reference map correspondence points list
   * at index 5.
   *
   * This list is provided for visualisation and debugging purposes.
   *
   * \return The list of correspondence points in the robot map.
   * \throw std::LogicError if the Transformer has no loaded map information.
   */
  const CorrespondencePoints& robot_map_corr_points() const;

  /// Get the list of triangles calculated by the Delaunay triangulation.
  /**
   * This triangle list is provided for visualisation and debugging purposes.
   *
   * \return The calculated triangles, as indices into the correspondence point lists.
   * \throw std::LogicError if the Transformer has no loaded map information.
   */
  const TriangleList& triangle_indices() const;

  /// Get the bounding box of the two maps.
  /**
   * Returns the bounding box (with one corner at 0, 0) of the two maps. This is the total size of
   * the two maps. If the robot map is aligned with the reference map, it will be the size of the
   * reference/robot map. However, if the robot map is offset, it will be larger, containing the
   * size of an image that is needed to hold both the reference map and the robot map.
   *
   * \return The bounding box of the two maps.
   * \throw std::LogicError if the Transformer has no loaded map information.
   */
  std::pair<Point2D, Point2D> bounding_box() const;

  /// Transform a point in the robot map to its equivalent point in the reference map.
  /**
   * The transform is performed according to the affine transforms of the Delaunay triangles that
   * were calculated when the map information was loaded, along with the transformation from the
   * robot map to the reference map, if any.
   *
   * \note If the point lies outside of all Delaunay triangles, it will be transformed only by the
   * relative map transformation. This may or may not be accurate depending on your maps. In the
   * general case, you should assume that any points that lie outside the Delaunay triangulation
   * (i.e. are not enclosed by correspondence points) cannot be transformed accurately.
   *
   * \param point The point in the robot map to transform.
   * \return The transformed point in the reference map.
   * \throw std::RuntimeError if an error occurs in the calculations.
   * \throw std::LogicError if the Transformer has no loaded map information.
   */
  Point2D to_ref(Point2D const &point) const;

  /// Transform a point in the reference map to its equivalent point in the robot map.
  /**
   * The transform is performed according to the affine transforms of the Delaunay triangles that
   * were calculated when the map information was loaded, along with the transformation from the
   * reference map to the robot map, if any.
   *
   * \note If the point lies outside of all Delaunay triangles, it will be transformed only by the
   * relative map transformation. This may or may not be accurate depending on your maps. In the
   * general case, you should assume that any points that lie outside the Delaunay triangulation
   * (i.e. are not enclosed by correspondence points) cannot be transformed accurately.
   *
   * \param point The point in the reference map to transform.
   * \return The transformed point in the robot map.
   * \throw std::RuntimeError if an error occurs in the calculations.
   * \throw std::LogicError if the Transformer has no loaded map information.
   */
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
