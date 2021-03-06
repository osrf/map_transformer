cmake_minimum_required(VERSION 3.16)
project(map_transformer VERSION 1.0.3)

set(CMAKE_C_STANDARD 99)
set(CMAKE_CXX_STANDARD 17)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(OpenCV REQUIRED)
find_package(yaml_cpp_vendor REQUIRED)

add_library(map_transformer src/transformer.cpp)
target_include_directories(map_transformer PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_link_libraries(map_transformer PUBLIC ${YAML_CPP_LIBRARIES} ${OpenCV_LIBS})
target_compile_definitions(map_transformer PRIVATE "MAP_TRANSFORMER_BUILDING_LIBRARY")

add_executable(transform_visualiser src/visualiser.cpp)
target_include_directories(transform_visualiser PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_link_libraries(transform_visualiser PUBLIC map_transformer ${YAML_CPP_LIBRARIES} ${OpenCV_LIBS})

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
    src/visualiser.cpp
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

option(BUILD_TESTING "Build tests" ON)
if(BUILD_TESTING)
  find_package(ament_lint_auto)
  if(AMENT_LINT_AUTO_FOUND)
    ament_lint_auto_find_test_dependencies()
  endif()

  set(TEST_DATA_DIRECTORY "${PROJECT_SOURCE_DIR}/test")
  configure_file(
    test_config.hpp.in
    ${CMAKE_CURRENT_BINARY_DIR}/include/map_transformer/test_config.hpp
    )

  find_package(GTest REQUIRED)
  include(GoogleTest)

  add_executable(test_transforms test/test_transforms.cpp)
  target_include_directories(test_transforms PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_BINARY_DIR}/include>
    )
  target_link_libraries(test_transforms
    map_transformer
    ${YAML_CPP_LIBRARIES}
    GTest::GTest
    GTest::Main)
  gtest_discover_tests(test_transforms)

  add_executable(test_loading test/test_loading.cpp)
  target_include_directories(test_loading PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_BINARY_DIR}/include>
    )
  target_link_libraries(test_loading
    map_transformer
    ${YAML_CPP_LIBRARIES}
    GTest::GTest
    GTest::Main)
  gtest_discover_tests(test_loading)
endif()

find_package(Doxygen)
if(DOXYGEN_FOUND)
  set(DOXYGEN_INPUT_DIRS
    "${CMAKE_CURRENT_SOURCE_DIR}/include ${CMAKE_CURRENT_SOURCE_DIR}/src")
  set(DOXYGEN_OUTPUT_DIR "${CMAKE_CURRENT_BINARY_DIR}")
  set(DOXYGEN_XML_DIR "${DOXYGEN_OUTPUT_DIR}/xml")
  set(DOXYGEN_INDEX_FILE "${DOXYGEN_OUTPUT_DIR}/index.xml")

  set(DOXYGEN_DOXYFILE_IN "${CMAKE_CURRENT_SOURCE_DIR}/doc/doxyfile.in")
  set(DOXYGEN_DOXYFILE "${CMAKE_CURRENT_BINARY_DIR}/doxyfile")
  configure_file(${DOXYGEN_DOXYFILE_IN} ${DOXYGEN_DOXYFILE})

  set(DOXYGEN_GENERATE_XML YES)
  set(DOXYGEN_GENERATE_HTML NO)
  add_custom_command(OUTPUT ${DOXYGEN_INDEX_FILE}
    COMMAND ${CMAKE_COMMAND} -E make_directory ${DOXYGEN_OUTPUT_DIR}
    COMMAND ${CMAKE_COMMAND} -E make_directory ${DOXYGEN_XML_DIR}
    COMMAND ${DOXYGEN_EXECUTABLE} ${DOXYGEN_DOXYFILE}
    VERBATIM)
  add_custom_target(doxygen ALL DEPENDS ${DOXYGEN_INDEX_FILE})

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
  add_custom_target(doc ALL DEPENDS ${SPHINX_INDEX_FILE} doxygen)
endif()
