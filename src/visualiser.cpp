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

#include <fstream>
#include <iostream>
#include <string>

#include <map_transformer/transformer.hpp>
#include <opencv2/highgui.hpp>

cv::Mat ref_map_image;
cv::Mat robot_map_image;
map_transformer::Transformer transformer;

void draw_point(cv::Mat & image, cv::Point point, cv::Scalar const & colour) {
  // Horizontal line
  cv::line(image, cv::Point(point.x - 5, point.y), cv::Point(point.x + 5, point.y), colour, 2);
  // Vertical line
  cv::line(image, cv::Point(point.x, point.y - 5), cv::Point(point.x, point.y + 5), colour, 2);
}


void draw_correspondence_points(
  cv::Mat & image,
  map_transformer::CorrespondencePoints const & points)
{
  cv::Scalar colour(255, 0, 0);
  for (auto& p : points) {
    draw_point(image, cv::Point(p.first, p.second), colour);
  }
}


void draw_triangulation(
  cv::Mat & image,
  map_transformer::CorrespondencePoints const & points,
  map_transformer::TriangleList const & triangle_indices,
  bool number_triangles)
{
  cv::Scalar colour(0, 200, 0);
  cv::Scalar label_colour(0, 128, 0);

  unsigned int label{0};
  for (auto& t : triangle_indices) {
    cv::Point p1(points[std::get<0>(t)].first, points[std::get<0>(t)].second);
    cv::Point p2(points[std::get<1>(t)].first, points[std::get<1>(t)].second);
    cv::Point p3(points[std::get<2>(t)].first, points[std::get<2>(t)].second);

    cv::line(image, p1, p2, colour);
    cv::line(image, p2, p3, colour);
    cv::line(image, p3, p1, colour);

    if (number_triangles) {
      cv::Point center(
        (p1.x + p2.x + p3.x) / 3,
        (p1.y + p2.y + p3.y) / 3);
      std::stringstream label_text;
      label_text << label;
      cv::putText(image, label_text.str(), center, cv::FONT_HERSHEY_SIMPLEX, 0.3, label_colour);
    }

    ++label;
  }
}


void pick_point_to_ref(int event, int x, int y, int, void*) {
  if (event != cv::EVENT_LBUTTONUP) {
    return;
  }

  cv::Scalar colour(0, 0, 255);
  // Plot the clicked point
  draw_point(robot_map_image, cv::Point(x, y), colour);
  // Plot the equivalent position on the reference map
  draw_point(
    ref_map_image,
    cv::Point(
      x + transformer.robot_map_translation().first,
      y + transformer.robot_map_translation().first),
    colour);
  // Transform the point according to the warping transformations
  auto transformed_point = transformer.to_ref(map_transformer::Point2D(x, y));
  // Plot the transformed point on the reference map
  colour[1] = 255; colour[2] = 0;
  draw_point(ref_map_image, cv::Point(transformed_point.first, transformed_point.second), colour);

  cv::imshow("Reference map", ref_map_image);
  cv::imshow("Robot map", robot_map_image);
  std::cout << "Transformed " << x << ", " << y << " (robot) to " << transformed_point.first <<
    ", " << transformed_point.second << " (reference)\n";
}


void pick_point_to_robot(int event, int x, int y, int, void*) {
  if (event != cv::EVENT_LBUTTONUP) {
    return;
  }

  cv::Scalar colour(0, 0, 255);
  // Plot the clicked point
  draw_point(ref_map_image, cv::Point(x, y), colour);
  // Plot the equivalent position on the robot map
  draw_point(
    robot_map_image,
    cv::Point(
      x - transformer.robot_map_translation().first,
      y - transformer.robot_map_translation().first),
    colour);
  // Transform the point according to the warping transformations
  auto transformed_point = transformer.to_robot(map_transformer::Point2D(x, y));
  // Plot the transformed point on the robot map
  colour[1] = 255; colour[2] = 0;
  draw_point(robot_map_image, cv::Point(transformed_point.first, transformed_point.second), colour);

  cv::imshow("Reference map", ref_map_image);
  cv::imshow("Robot map", robot_map_image);
  std::cout << "Transformed " << x << ", " << y << " (reference) to " << transformed_point.first <<
    ", " << transformed_point.second << " (robot)\n";
}


int main(int argc, char ** argv)
{
  const std::string keys =
    "{help h | | print this message}"
    "{c corr-points | false | display the correspondence points}"
    "{m map-info-file | | the YAML file containing the map information}"
    "{t triangulation | false | display the Delaunay triangulation}"
    "{n number-triangles | false | number the Delaunay triangles}";
  cv::CommandLineParser parser(argc, argv, keys);
  parser.about("Map transformer visualisation");

  if (parser.has("help")) {
    parser.printMessage();
    return 0;
  }
  if (parser.get<std::string>("map-info-file").size() == 0) {
    std::cerr << "No map information file provided\n\n";
    parser.printMessage();
    return 1;
  }

  std::cout << "Loading configuration from " << parser.get<std::string>("map-info-file") << '\n';

  std::ifstream yaml_file(parser.get<std::string>("map-info-file"));
  if (!yaml_file.is_open()) {
    std::cerr << "Could not read YAML document\n";
    return 1;
  }
  std::ostringstream sstr;
  sstr << yaml_file.rdbuf();
  std::string yaml_doc(sstr.str());

  transformer.load(yaml_doc);

  // Load the map images for the visualisation background
  ref_map_image = cv::imread(transformer.ref_map_image_file());
  if (ref_map_image.empty()) {
    std::cerr << "Could not load reference map image file\n";
    return 1;
  }
  robot_map_image = cv::imread(transformer.robot_map_image_file());
  if (robot_map_image.empty()) {
    std::cerr << "Could not load robot map image file\n";
    return 1;
  }

  if (parser.get<bool>("corr-points")) {
    draw_correspondence_points(ref_map_image, transformer.ref_map_corr_points());
    draw_correspondence_points(robot_map_image, transformer.robot_map_corr_points());
  }
  if (parser.get<bool>("triangulation")) {
    draw_triangulation(
      ref_map_image,
      transformer.ref_map_corr_points(),
      transformer.triangle_indices(),
      parser.get<bool>("number-triangles"));
    draw_triangulation(
      robot_map_image,
      transformer.robot_map_corr_points(),
      transformer.triangle_indices(),
      parser.get<bool>("number-triangles"));
  }

  cv::namedWindow("Reference map", cv::WINDOW_NORMAL);
  cv::namedWindow("Robot map", cv::WINDOW_NORMAL);
  cv::imshow("Reference map", ref_map_image);
  cv::imshow("Robot map", robot_map_image);

  cv::setMouseCallback("Reference map", pick_point_to_robot);
  cv::setMouseCallback("Robot map", pick_point_to_ref);

  std::cout << "Press q to quit\n";

  int key{0};
  while (key != 27 && key != 113) {
    key = cv::waitKey();
  }

  cv::destroyAllWindows();
  return 0;
}
