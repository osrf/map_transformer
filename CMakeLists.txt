cmake_minimum_required(VERSION 3.5)
project(map_transformer)

# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_ros REQUIRED)
find_package(OpenCV REQUIRED)
find_package(yaml-cpp REQUIRED)

add_library(map_transformer src/transformer.cpp)
target_include_directories(map_transformer PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_link_libraries(map_transformer ${YAML_CPP_LIBRARIES} ${OpenCV_LIBS})

target_compile_definitions(map_transformer PRIVATE "MAP_TRANSFORMER_BUILDING_LIBRARY")

install(
  DIRECTORY include/
  DESTINATION include
)
install(
  TARGETS map_transformer
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
  ament_add_gtest(test_transforms test/test_transforms.cpp)
  #if(TARGET test_transforms)
    #ament_target_dependencies(test_loading
      #)
    #target_link_libraries(test_loading ${YAML_CPP_LIBRARIES})
  #endif()
  ament_add_gtest(test_loading test/test_loading.cpp)
  if(TARGET test_loading)
    ament_target_dependencies(test_loading
      )
    target_link_libraries(test_loading map_transformer ${YAML_CPP_LIBRARIES})
  endif()
endif()

ament_export_include_directories(
  include
)
ament_export_libraries(
  map_transformer
)
ament_export_targets(
  export_${PROJECT_NAME}
)

ament_package()
