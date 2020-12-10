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

add_executable(transform_visualiser src/visualiser.cpp)
target_include_directories(transform_visualiser PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_link_libraries(transform_visualiser map_transformer ${YAML_CPP_LIBRARIES} ${OpenCV_LIBS})
target_compile_definitions(transform_visualiser PUBLIC "MAP_TRANSFORMER_BUILDING_LIBRARY")

set(SAMPLE_DIRECTORY ${CMAKE_INSTALL_PREFIX}/share/${PROJECT_NAME}/sample)
configure_file(
  sample/aligned_map.yaml.in
  ${CMAKE_CURRENT_BINARY_DIR}/sample/aligned_map.yaml
  )
configure_file(
  sample/offset_map.yaml.in
  ${CMAKE_CURRENT_BINARY_DIR}/sample/offset_map.yaml
  )
file(
  COPY
    sample/aligned_map_ref.png
    sample/aligned_map_robot.png
    sample/ref_map_100_100.png
    sample/robot_map_80_110.png
  DESTINATION
    ${CMAKE_CURRENT_BINARY_DIR}/sample
  )
install(
  DIRECTORY ${CMAKE_CURRENT_BINARY_DIR}/sample
  DESTINATION share/${PROJECT_NAME}
  )

install(
  DIRECTORY include/
  DESTINATION include
)
install(
  TARGETS map_transformer transform_visualiser
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()

  set(TEST_DATA_DIRECTORY "${PROJECT_SOURCE_DIR}/test")
  configure_file(
    test_config.hpp.in
    ${CMAKE_CURRENT_BINARY_DIR}/include/map_transformer/test_config.hpp
    )

  ament_add_gtest(test_transforms test/test_transforms.cpp)
  if(TARGET test_transforms)
    ament_target_dependencies(test_transforms
      )
    target_link_libraries(test_transforms map_transformer ${YAML_CPP_LIBRARIES})
    target_include_directories(
      test_transforms
      PUBLIC
      $<BUILD_INTERFACE:${CMAKE_CURRENT_BINARY_DIR}/include>
      )
  endif()
  ament_add_gtest(test_loading test/test_loading.cpp)
  if(TARGET test_loading)
    ament_target_dependencies(test_loading
      )
    target_link_libraries(test_loading map_transformer ${YAML_CPP_LIBRARIES})
    target_include_directories(
      test_loading
      PUBLIC
      $<BUILD_INTERFACE:${CMAKE_CURRENT_BINARY_DIR}/include>
      )
  endif()
endif()

find_package(Doxygen)
if(DOXYGEN_FOUND)
  set(DOXYGEN_OUTPUT_DIR "${PROJECT_BINARY_DIR}/xml")
  set(DOXYGEN_INDEX_FILE "${DOXYGEN_OUTPUT_DIR}/index.xml")
  set(DOXYGEN_GENERATE_XML YES)
  set(DOXYGEN_GENERATE_HTML NO)
  doxygen_add_docs(map_transformer_doxygen
    ${CMAKE_CURRENT_SOURCE_DIR}/include
    ${CMAKE_CURRENT_SOURCE_DIR}/src
    ALL)
  set(CMAKE_MODULE_PATH "${CMAKE_CURRENT_SOURCE_DIR}/cmake/Modules" ${CMAKE_MODULE_PATH})
  find_package(Sphinx REQUIRED)
  SET(SPHINX_INDEX_FILE "${CMAKE_CURRENT_BINARY_DIR}/doc/index.html")
  add_custom_command(OUTPUT ${SPHINX_INDEX_FILE}
    COMMAND ${SPHINX_EXECUTABLE} -b html
    -Dbreathe_projects.MapTransformer=${DOXYGEN_OUTPUT_DIR}
    ${CMAKE_CURRENT_SOURCE_DIR}/doc
    ${CMAKE_CURRENT_BINARY_DIR}/doc
    WORKING_DIRECTORY ${CMAKE_CURRENT_BINARY_DIR}
    DEPENDS
      MAIN_DEPENDENCY ${CMAKE_CURRENT_SOURCE_DIR}/doc/conf.py
      ${CMAKE_CURRENT_SOURCE_DIR}/doc/index.rst
      ${DOXYGEN_INDEX_FILE}
    )
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
