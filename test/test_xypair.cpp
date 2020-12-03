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

#include "map_transformer/xypair.hpp"

#include <gtest/gtest.h>

TEST(TestXYPair, equal_operator) {
  map_transformer::XYPair a{1.0, 3.14159};
  map_transformer::XYPair b{1.0, 3.14159};
  map_transformer::XYPair c{4.2, 42.42};
  EXPECT_TRUE(a == b);
  EXPECT_FALSE(a == c);
}

TEST(TestXYPair, notequal_operator) {
  map_transformer::XYPair a{1.0, 3.14159};
  map_transformer::XYPair b{1.0, 3.14159};
  map_transformer::XYPair c{4.2, 42.42};
  EXPECT_TRUE(a != c);
  EXPECT_FALSE(a != b);
}
