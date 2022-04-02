cmake_minimum_required(VERSION 3.5.0)

# Extract package name and version
find_package(ros_industrial_cmake_boilerplate REQUIRED)
extract_package_metadata(pkg)
project(${pkg_extracted_name} VERSION ${pkg_extracted_version} LANGUAGES CXX)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  roslib
  std_srvs
  tesseract_monitoring
  tesseract_rosutils
)

find_package(tesseract_environment REQUIRED)
find_package(tesseract_examples REQUIRED)

catkin_package()

# Load variable for clang tidy args, compiler options and cxx version
tesseract_variables()

macro(add_example test_name test_file)
  add_executable(${test_name} ${test_file})
  target_link_libraries(${test_name}
    tesseract::tesseract_examples
    tesseract::tesseract_environment
    ${catkin_LIBRARIES})
  target_compile_options(${test_name} PRIVATE ${TESSERACT_COMPILE_OPTIONS})
  target_clang_tidy(${test_name} ARGUMENTS ${TESSERACT_CLANG_TIDY_ARGS} ENABLE ${TESSERACT_ENABLE_CLANG_TIDY})
  target_cxx_version(${test_name} PUBLIC VERSION ${TESSERACT_CXX_VERSION})
  target_include_directories(${test_name} PRIVATE
      "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
      "$<INSTALL_INTERFACE:include>")
  target_include_directories(${test_name} SYSTEM PRIVATE
      ${catkin_INCLUDE_DIRS})
endmacro()

add_example(${PROJECT_NAME}_basic_cartesian_example_node src/basic_cartesian_example_node.cpp)
add_example(${PROJECT_NAME}_car_seat_example_node src/car_seat_example_node.cpp)
add_example(${PROJECT_NAME}_freespace_hybrid_example_node src/freespace_hybrid_example_node.cpp)
add_example(${PROJECT_NAME}_freespace_ompl_example_node src/freespace_ompl_example_node.cpp)
add_example(${PROJECT_NAME}_glass_upright_example_node src/glass_upright_example_node.cpp)
add_example(${PROJECT_NAME}_online_planning_example_node src/online_planning_example_node.cpp)
add_example(${PROJECT_NAME}_pick_and_place_example_node src/pick_and_place_example_node.cpp)
add_example(${PROJECT_NAME}_puzzle_piece_auxillary_axes_example_node src/puzzle_piece_auxillary_axes_example_node.cpp)
add_example(${PROJECT_NAME}_puzzle_piece_example_node src/puzzle_piece_example_node.cpp)
add_example(${PROJECT_NAME}_scene_graph_example_node src/scene_graph_example_node.cpp)

## OMPL Constraint planning is not available in version less than 1.4
#if(NOT OMPL_VERSION VERSION_LESS "1.4.0")
#  add_gtest(${PROJECT_NAME}_glass_upright_ompl_example_node src/glass_upright_ompl_example_node.cpp)
#endif()


# Mark executables and/or libraries for installation
install(
  TARGETS
    ${PROJECT_NAME}_basic_cartesian_example_node
    ${PROJECT_NAME}_car_seat_example_node
    ${PROJECT_NAME}_freespace_ompl_example_node
    ${PROJECT_NAME}_freespace_hybrid_example_node
    ${PROJECT_NAME}_glass_upright_example_node
    ${PROJECT_NAME}_online_planning_example_node
    ${PROJECT_NAME}_pick_and_place_example_node
    ${PROJECT_NAME}_puzzle_piece_example_node
    ${PROJECT_NAME}_puzzle_piece_auxillary_axes_example_node
    ${PROJECT_NAME}_scene_graph_example_node
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

## OMPL Constraint planning is not available in version less than 1.4
#if(NOT OMPL_VERSION VERSION_LESS "1.4.0")
#  install(
#    TARGETS
#      ${PROJECT_NAME}_glass_upright_ompl_example_node
#    ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#    LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#    RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
#  )
#endif()

install(
  DIRECTORY config launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)


