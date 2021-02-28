cmake_minimum_required(VERSION 2.8.3)
project(tesseract_ros_examples)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  roslib
  tesseract_monitoring
  tesseract_rosutils
  tesseract_planning_server
  octomap_ros
  pcl_conversions
)

find_package(tesseract_environment REQUIRED)
find_package(tesseract_command_language REQUIRED)
find_package(tesseract_motion_planners REQUIRED)
find_package(tesseract_process_managers REQUIRED)
find_package(tesseract_common REQUIRED)
find_package(trajopt_sqp REQUIRED)
find_package(trajopt_ifopt REQUIRED)
find_package(ros_industrial_cmake_boilerplate REQUIRED)
find_package(PCL REQUIRED COMPONENTS core features filters io segmentation surface)

catkin_package()

# Load variable for clang tidy args, compiler options and cxx version
tesseract_variables()

add_library(${PROJECT_NAME}_basic_cartesian_example src/basic_cartesian_example.cpp)
target_link_libraries(${PROJECT_NAME}_basic_cartesian_example tesseract::tesseract_environment_core tesseract::tesseract_environment_ofkt tesseract::tesseract_process_managers ${PCL_LIBRARIES} ${catkin_LIBRARIES})
target_compile_options(${PROJECT_NAME}_basic_cartesian_example PUBLIC ${TESSERACT_COMPILE_OPTIONS})
target_clang_tidy(${PROJECT_NAME}_basic_cartesian_example ARGUMENTS ${TESSERACT_CLANG_TIDY_ARGS} ENABLE ${TESSERACT_ENABLE_CLANG_TIDY})
target_cxx_version(${PROJECT_NAME}_basic_cartesian_example PUBLIC VERSION 17)
foreach(DEF ${PCL_DEFINITIONS})
  string(STRIP ${DEF} DEF)
  if (NOT "${DEF}" STREQUAL "")
    string(SUBSTRING "${DEF}" 0 2 DEF_PREFIX)
    if ("${DEF_PREFIX}" STREQUAL "-m")
      string(REPLACE " " ";" DEF ${DEF})
      foreach(OPTION_DEF ${DEF})
        target_compile_options(${PROJECT_NAME}_basic_cartesian_example PUBLIC ${OPTION_DEF})
      endforeach()
    else()
      target_compile_definitions(${PROJECT_NAME}_basic_cartesian_example PUBLIC ${DEF})
    endif()
  endif()
endforeach()
target_include_directories(${PROJECT_NAME}_basic_cartesian_example PUBLIC
    "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
    "$<INSTALL_INTERFACE:include>")
target_include_directories(${PROJECT_NAME}_basic_cartesian_example SYSTEM PUBLIC
    ${catkin_INCLUDE_DIRS}
    ${PCL_INCLUDE_DIRS})

add_executable(${PROJECT_NAME}_basic_cartesian_example_node src/basic_cartesian_example_node.cpp)
target_link_libraries(${PROJECT_NAME}_basic_cartesian_example_node ${PROJECT_NAME}_basic_cartesian_example ${catkin_LIBRARIES})
target_compile_options(${PROJECT_NAME}_basic_cartesian_example_node PRIVATE ${TESSERACT_COMPILE_OPTIONS})
target_clang_tidy(${PROJECT_NAME}_basic_cartesian_example_node ARGUMENTS ${TESSERACT_CLANG_TIDY_ARGS} ENABLE ${TESSERACT_ENABLE_CLANG_TIDY})
target_cxx_version(${PROJECT_NAME}_basic_cartesian_example_node PUBLIC VERSION 17)
target_include_directories(${PROJECT_NAME}_basic_cartesian_example_node PRIVATE
    "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
    "$<INSTALL_INTERFACE:include>")
target_include_directories(${PROJECT_NAME}_basic_cartesian_example_node SYSTEM PRIVATE
    ${catkin_INCLUDE_DIRS})

add_library(${PROJECT_NAME}_freespace_ompl_example src/freespace_ompl_example.cpp)
target_link_libraries(${PROJECT_NAME}_freespace_ompl_example tesseract::tesseract_environment_core tesseract::tesseract_environment_ofkt tesseract::tesseract_process_managers ${catkin_LIBRARIES})
target_compile_options(${PROJECT_NAME}_freespace_ompl_example PUBLIC ${TESSERACT_COMPILE_OPTIONS})
target_clang_tidy(${PROJECT_NAME}_freespace_ompl_example ARGUMENTS ${TESSERACT_CLANG_TIDY_ARGS} ENABLE ${TESSERACT_ENABLE_CLANG_TIDY})
target_cxx_version(${PROJECT_NAME}_freespace_ompl_example PUBLIC VERSION 17)
target_include_directories(${PROJECT_NAME}_freespace_ompl_example PUBLIC
    "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
    "$<INSTALL_INTERFACE:include>")
target_include_directories(${PROJECT_NAME}_freespace_ompl_example SYSTEM PUBLIC
    ${catkin_INCLUDE_DIRS})

add_executable(${PROJECT_NAME}_freespace_ompl_example_node src/freespace_ompl_example_node.cpp)
target_link_libraries(${PROJECT_NAME}_freespace_ompl_example_node ${PROJECT_NAME}_freespace_ompl_example ${catkin_LIBRARIES})
target_compile_options(${PROJECT_NAME}_freespace_ompl_example_node PRIVATE ${TESSERACT_COMPILE_OPTIONS})
target_clang_tidy(${PROJECT_NAME}_freespace_ompl_example_node ARGUMENTS ${TESSERACT_CLANG_TIDY_ARGS} ENABLE ${TESSERACT_ENABLE_CLANG_TIDY})
target_cxx_version(${PROJECT_NAME}_freespace_ompl_example_node PUBLIC VERSION 17)
target_include_directories(${PROJECT_NAME}_freespace_ompl_example_node PRIVATE
    "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
    "$<INSTALL_INTERFACE:include>")
target_include_directories(${PROJECT_NAME}_freespace_ompl_example_node SYSTEM PRIVATE
    ${catkin_INCLUDE_DIRS})

add_library(${PROJECT_NAME}_freespace_hybrid_example src/freespace_hybrid_example.cpp)
target_link_libraries(${PROJECT_NAME}_freespace_hybrid_example tesseract::tesseract_environment_core tesseract::tesseract_environment_ofkt tesseract::tesseract_process_managers ${catkin_LIBRARIES})
target_compile_options(${PROJECT_NAME}_freespace_hybrid_example PUBLIC ${TESSERACT_COMPILE_OPTIONS})
target_clang_tidy(${PROJECT_NAME}_freespace_hybrid_example ARGUMENTS ${TESSERACT_CLANG_TIDY_ARGS} ENABLE ${TESSERACT_ENABLE_CLANG_TIDY})
target_cxx_version(${PROJECT_NAME}_freespace_hybrid_example PUBLIC VERSION 17)
target_include_directories(${PROJECT_NAME}_freespace_hybrid_example PUBLIC
    "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
    "$<INSTALL_INTERFACE:include>")
target_include_directories(${PROJECT_NAME}_freespace_hybrid_example SYSTEM PUBLIC
    ${catkin_INCLUDE_DIRS})

add_executable(${PROJECT_NAME}_freespace_hybrid_example_node src/freespace_hybrid_example_node.cpp)
target_link_libraries(${PROJECT_NAME}_freespace_hybrid_example_node ${PROJECT_NAME}_freespace_hybrid_example ${catkin_LIBRARIES})
target_compile_options(${PROJECT_NAME}_freespace_hybrid_example_node PRIVATE ${TESSERACT_COMPILE_OPTIONS})
target_clang_tidy(${PROJECT_NAME}_freespace_hybrid_example_node ARGUMENTS ${TESSERACT_CLANG_TIDY_ARGS} ENABLE ${TESSERACT_ENABLE_CLANG_TIDY})
target_cxx_version(${PROJECT_NAME}_freespace_hybrid_example_node PUBLIC VERSION 17)
target_include_directories(${PROJECT_NAME}_freespace_hybrid_example_node PRIVATE
    "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
    "$<INSTALL_INTERFACE:include>")
target_include_directories(${PROJECT_NAME}_freespace_hybrid_example_node SYSTEM PRIVATE
    ${catkin_INCLUDE_DIRS})

add_library(${PROJECT_NAME}_car_seat_example src/car_seat_example.cpp)
target_link_libraries(${PROJECT_NAME}_car_seat_example tesseract::tesseract_environment_core tesseract::tesseract_environment_ofkt tesseract::tesseract_process_managers ${catkin_LIBRARIES})
target_compile_options(${PROJECT_NAME}_car_seat_example PUBLIC ${TESSERACT_COMPILE_OPTIONS})
target_clang_tidy(${PROJECT_NAME}_car_seat_example ARGUMENTS ${TESSERACT_CLANG_TIDY_ARGS} ENABLE ${TESSERACT_ENABLE_CLANG_TIDY})
target_cxx_version(${PROJECT_NAME}_car_seat_example PUBLIC VERSION 17)
target_include_directories(${PROJECT_NAME}_car_seat_example PUBLIC
    "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
    "$<INSTALL_INTERFACE:include>")
target_include_directories(${PROJECT_NAME}_car_seat_example SYSTEM PUBLIC
    ${catkin_INCLUDE_DIRS})

add_executable(${PROJECT_NAME}_car_seat_example_node src/car_seat_example_node.cpp)
target_link_libraries(${PROJECT_NAME}_car_seat_example_node ${PROJECT_NAME}_car_seat_example ${catkin_LIBRARIES})
target_compile_options(${PROJECT_NAME}_car_seat_example_node PRIVATE ${TESSERACT_COMPILE_OPTIONS})
target_clang_tidy(${PROJECT_NAME}_car_seat_example_node ARGUMENTS ${TESSERACT_CLANG_TIDY_ARGS} ENABLE ${TESSERACT_ENABLE_CLANG_TIDY})
target_cxx_version(${PROJECT_NAME}_car_seat_example_node PUBLIC VERSION 17)
target_include_directories(${PROJECT_NAME}_car_seat_example_node PRIVATE
    "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
    "$<INSTALL_INTERFACE:include>")
target_include_directories(${PROJECT_NAME}_car_seat_example_node SYSTEM PRIVATE
    ${catkin_INCLUDE_DIRS})

add_library(${PROJECT_NAME}_glass_upright_example src/glass_upright_example.cpp)
target_link_libraries(${PROJECT_NAME}_glass_upright_example tesseract::tesseract_environment_core tesseract::tesseract_environment_ofkt tesseract::tesseract_process_managers ${catkin_LIBRARIES})
target_compile_options(${PROJECT_NAME}_glass_upright_example PUBLIC ${TESSERACT_COMPILE_OPTIONS})
target_clang_tidy(${PROJECT_NAME}_glass_upright_example ARGUMENTS ${TESSERACT_CLANG_TIDY_ARGS} ENABLE ${TESSERACT_ENABLE_CLANG_TIDY})
target_cxx_version(${PROJECT_NAME}_glass_upright_example PUBLIC VERSION 17)
target_include_directories(${PROJECT_NAME}_glass_upright_example PUBLIC
    "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
    "$<INSTALL_INTERFACE:include>")
target_include_directories(${PROJECT_NAME}_glass_upright_example SYSTEM PUBLIC
    ${catkin_INCLUDE_DIRS})

add_executable(${PROJECT_NAME}_glass_upright_example_node src/glass_upright_example_node.cpp)
target_link_libraries(${PROJECT_NAME}_glass_upright_example_node ${PROJECT_NAME}_glass_upright_example ${catkin_LIBRARIES})
target_compile_options(${PROJECT_NAME}_glass_upright_example_node PRIVATE ${TESSERACT_COMPILE_OPTIONS})
target_clang_tidy(${PROJECT_NAME}_glass_upright_example_node ARGUMENTS ${TESSERACT_CLANG_TIDY_ARGS} ENABLE ${TESSERACT_ENABLE_CLANG_TIDY})
target_cxx_version(${PROJECT_NAME}_glass_upright_example_node PUBLIC VERSION 17)
target_include_directories(${PROJECT_NAME}_glass_upright_example_node PRIVATE
    "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
    "$<INSTALL_INTERFACE:include>")
target_include_directories(${PROJECT_NAME}_glass_upright_example_node SYSTEM PRIVATE
    ${catkin_INCLUDE_DIRS})

## OMPL Constraint planning is not available in version less than 1.4
#if(NOT OMPL_VERSION VERSION_LESS "1.4.0")
#  add_library(${PROJECT_NAME}_glass_upright_ompl_example src/glass_upright_ompl_example.cpp)
#  target_link_libraries(${PROJECT_NAME}_glass_upright_ompl_example tesseract::tesseract_environment_core tesseract::tesseract_environment_ofkt trajopt::trajopt tesseract::tesseract_motion_planners_ompl tesseract::tesseract_motion_planners_trajopt jsoncpp_lib ${catkin_LIBRARIES})
#  target_compile_options(${PROJECT_NAME}_glass_upright_ompl_example PUBLIC ${TESSERACT_COMPILE_OPTIONS})
#  target_clang_tidy(${PROJECT_NAME}_glass_upright_ompl_example ARGUMENTS ${TESSERACT_CLANG_TIDY_ARGS} ENABLE ${TESSERACT_ENABLE_CLANG_TIDY})
#  target_cxx_version(${PROJECT_NAME}_glass_upright_ompl_example PUBLIC VERSION 17)
#  target_include_directories(${PROJECT_NAME}_glass_upright_ompl_example PUBLIC
#      "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
#      "$<INSTALL_INTERFACE:include>")
#  target_include_directories(${PROJECT_NAME}_glass_upright_ompl_example SYSTEM PUBLIC
#      ${catkin_INCLUDE_DIRS})

#  add_executable(${PROJECT_NAME}_glass_upright_ompl_example_node src/glass_upright_ompl_example_node.cpp)
#  target_link_libraries(${PROJECT_NAME}_glass_upright_ompl_example_node ${PROJECT_NAME}_glass_upright_ompl_example ${catkin_LIBRARIES})
#  target_compile_options(${PROJECT_NAME}_glass_upright_ompl_example_node PRIVATE ${TESSERACT_COMPILE_OPTIONS})
#  target_clang_tidy(${PROJECT_NAME}_glass_upright_ompl_example_node ARGUMENTS ${TESSERACT_CLANG_TIDY_ARGS} ENABLE ${TESSERACT_ENABLE_CLANG_TIDY})
#  target_cxx_version(${PROJECT_NAME}_glass_upright_ompl_example_node PUBLIC VERSION 17)
#  target_include_directories(${PROJECT_NAME}_glass_upright_ompl_example_node PRIVATE
#      "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
#      "$<INSTALL_INTERFACE:include>")
#  target_include_directories(${PROJECT_NAME}_glass_upright_ompl_example_node SYSTEM PRIVATE
#      ${catkin_INCLUDE_DIRS})
#endif()

add_library(${PROJECT_NAME}_online_planning_example src/online_planning_example.cpp)
target_link_libraries(${PROJECT_NAME}_online_planning_example tesseract::tesseract_environment_core tesseract::tesseract_environment_ofkt trajopt::trajopt_ifopt trajopt::trajopt_sqp ${catkin_LIBRARIES})
target_compile_options(${PROJECT_NAME}_online_planning_example PUBLIC ${TESSERACT_COMPILE_OPTIONS})
target_clang_tidy(${PROJECT_NAME}_online_planning_example ARGUMENTS ${TESSERACT_CLANG_TIDY_ARGS} ENABLE ${TESSERACT_ENABLE_CLANG_TIDY})
target_cxx_version(${PROJECT_NAME}_online_planning_example PUBLIC VERSION 17)
target_include_directories(${PROJECT_NAME}_online_planning_example PUBLIC
    "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
    "$<INSTALL_INTERFACE:include>")
target_include_directories(${PROJECT_NAME}_online_planning_example SYSTEM PUBLIC
    ${catkin_INCLUDE_DIRS})

add_executable(${PROJECT_NAME}_online_planning_example_node src/online_planning_example_node.cpp)
target_link_libraries(${PROJECT_NAME}_online_planning_example_node ${PROJECT_NAME}_online_planning_example ${catkin_LIBRARIES})
target_compile_options(${PROJECT_NAME}_online_planning_example_node PRIVATE ${TESSERACT_COMPILE_OPTIONS})
target_clang_tidy(${PROJECT_NAME}_online_planning_example_node ARGUMENTS ${TESSERACT_CLANG_TIDY_ARGS} ENABLE ${TESSERACT_ENABLE_CLANG_TIDY})
target_cxx_version(${PROJECT_NAME}_online_planning_example_node PUBLIC VERSION 17)
target_include_directories(${PROJECT_NAME}_online_planning_example_node PRIVATE
    "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
    "$<INSTALL_INTERFACE:include>")
target_include_directories(${PROJECT_NAME}_online_planning_example_node SYSTEM PRIVATE
    ${catkin_INCLUDE_DIRS})

add_library(${PROJECT_NAME}_pick_and_place_example src/pick_and_place_example.cpp)
target_link_libraries(${PROJECT_NAME}_pick_and_place_example tesseract::tesseract_environment_core tesseract::tesseract_environment_ofkt tesseract::tesseract_process_managers ${catkin_LIBRARIES})
target_compile_options(${PROJECT_NAME}_pick_and_place_example PUBLIC ${TESSERACT_COMPILE_OPTIONS})
target_clang_tidy(${PROJECT_NAME}_pick_and_place_example ARGUMENTS ${TESSERACT_CLANG_TIDY_ARGS} ENABLE ${TESSERACT_ENABLE_CLANG_TIDY})
target_cxx_version(${PROJECT_NAME}_pick_and_place_example PUBLIC VERSION 17)
target_include_directories(${PROJECT_NAME}_pick_and_place_example PUBLIC
    "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
    "$<INSTALL_INTERFACE:include>")
target_include_directories(${PROJECT_NAME}_pick_and_place_example SYSTEM PUBLIC
    ${catkin_INCLUDE_DIRS})

add_executable(${PROJECT_NAME}_pick_and_place_example_node src/pick_and_place_example_node.cpp)
target_link_libraries(${PROJECT_NAME}_pick_and_place_example_node ${PROJECT_NAME}_pick_and_place_example ${catkin_LIBRARIES})
target_compile_options(${PROJECT_NAME}_pick_and_place_example_node PRIVATE ${TESSERACT_COMPILE_OPTIONS})
target_clang_tidy(${PROJECT_NAME}_pick_and_place_example_node ARGUMENTS ${TESSERACT_CLANG_TIDY_ARGS} ENABLE ${TESSERACT_ENABLE_CLANG_TIDY})
target_cxx_version(${PROJECT_NAME}_pick_and_place_example_node PUBLIC VERSION 17)
target_include_directories(${PROJECT_NAME}_pick_and_place_example_node PRIVATE
    "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
    "$<INSTALL_INTERFACE:include>")
target_include_directories(${PROJECT_NAME}_pick_and_place_example_node SYSTEM PRIVATE
    ${catkin_INCLUDE_DIRS})

add_library(${PROJECT_NAME}_puzzle_piece_example src/puzzle_piece_example.cpp)
target_link_libraries(${PROJECT_NAME}_puzzle_piece_example tesseract::tesseract_environment_core tesseract::tesseract_environment_ofkt tesseract::tesseract_process_managers ${catkin_LIBRARIES})
target_compile_options(${PROJECT_NAME}_puzzle_piece_example PUBLIC ${TESSERACT_COMPILE_OPTIONS})
target_clang_tidy(${PROJECT_NAME}_puzzle_piece_example ARGUMENTS ${TESSERACT_CLANG_TIDY_ARGS} ENABLE ${TESSERACT_ENABLE_CLANG_TIDY})
target_cxx_version(${PROJECT_NAME}_puzzle_piece_example PUBLIC VERSION 17)
target_include_directories(${PROJECT_NAME}_puzzle_piece_example PUBLIC
    "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
    "$<INSTALL_INTERFACE:include>")
target_include_directories(${PROJECT_NAME}_puzzle_piece_example SYSTEM PUBLIC
    ${catkin_INCLUDE_DIRS})

add_executable(${PROJECT_NAME}_puzzle_piece_example_node src/puzzle_piece_example_node.cpp)
target_link_libraries(${PROJECT_NAME}_puzzle_piece_example_node ${PROJECT_NAME}_puzzle_piece_example ${catkin_LIBRARIES})
target_compile_options(${PROJECT_NAME}_puzzle_piece_example_node PRIVATE ${TESSERACT_COMPILE_OPTIONS})
target_clang_tidy(${PROJECT_NAME}_puzzle_piece_example_node ARGUMENTS ${TESSERACT_CLANG_TIDY_ARGS} ENABLE ${TESSERACT_ENABLE_CLANG_TIDY})
target_cxx_version(${PROJECT_NAME}_puzzle_piece_example_node PUBLIC VERSION 17)
target_include_directories(${PROJECT_NAME}_puzzle_piece_example_node PRIVATE
    "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
    "$<INSTALL_INTERFACE:include>")
target_include_directories(${PROJECT_NAME}_puzzle_piece_example_node SYSTEM PRIVATE
    ${catkin_INCLUDE_DIRS})

add_library(${PROJECT_NAME}_puzzle_piece_auxillary_axes_example src/puzzle_piece_auxillary_axes_example.cpp)
target_link_libraries(${PROJECT_NAME}_puzzle_piece_auxillary_axes_example tesseract::tesseract_environment_core tesseract::tesseract_environment_ofkt tesseract::tesseract_motion_planners_trajopt tesseract::tesseract_process_managers ${catkin_LIBRARIES})
target_compile_options(${PROJECT_NAME}_puzzle_piece_auxillary_axes_example PUBLIC ${TESSERACT_COMPILE_OPTIONS})
target_clang_tidy(${PROJECT_NAME}_puzzle_piece_auxillary_axes_example ARGUMENTS ${TESSERACT_CLANG_TIDY_ARGS} ENABLE ${TESSERACT_ENABLE_CLANG_TIDY})
target_cxx_version(${PROJECT_NAME}_puzzle_piece_auxillary_axes_example PUBLIC VERSION 17)
target_include_directories(${PROJECT_NAME}_puzzle_piece_auxillary_axes_example PUBLIC
    "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
    "$<INSTALL_INTERFACE:include>")
target_include_directories(${PROJECT_NAME}_puzzle_piece_auxillary_axes_example SYSTEM PUBLIC
    ${catkin_INCLUDE_DIRS})

add_executable(${PROJECT_NAME}_puzzle_piece_auxillary_axes_example_node src/puzzle_piece_auxillary_axes_example_node.cpp)
target_link_libraries(${PROJECT_NAME}_puzzle_piece_auxillary_axes_example_node ${PROJECT_NAME}_puzzle_piece_auxillary_axes_example ${catkin_LIBRARIES})
target_compile_options(${PROJECT_NAME}_puzzle_piece_auxillary_axes_example_node PRIVATE ${TESSERACT_COMPILE_OPTIONS})
target_clang_tidy(${PROJECT_NAME}_puzzle_piece_auxillary_axes_example_node ARGUMENTS ${TESSERACT_CLANG_TIDY_ARGS} ENABLE ${TESSERACT_ENABLE_CLANG_TIDY})
target_cxx_version(${PROJECT_NAME}_puzzle_piece_auxillary_axes_example_node PUBLIC VERSION 17)
target_include_directories(${PROJECT_NAME}_puzzle_piece_auxillary_axes_example_node PRIVATE
    "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
    "$<INSTALL_INTERFACE:include>")
target_include_directories(${PROJECT_NAME}_puzzle_piece_auxillary_axes_example_node SYSTEM PRIVATE
    ${catkin_INCLUDE_DIRS})

add_library(${PROJECT_NAME}_scene_graph_example src/scene_graph_example.cpp)
target_link_libraries(${PROJECT_NAME}_scene_graph_example tesseract::tesseract_environment_core tesseract::tesseract_environment_ofkt tesseract::tesseract_motion_planners_trajopt jsoncpp_lib ${catkin_LIBRARIES})
target_compile_options(${PROJECT_NAME}_scene_graph_example PUBLIC ${TESSERACT_COMPILE_OPTIONS})
target_clang_tidy(${PROJECT_NAME}_scene_graph_example ARGUMENTS ${TESSERACT_CLANG_TIDY_ARGS} ENABLE ${TESSERACT_ENABLE_CLANG_TIDY})
target_cxx_version(${PROJECT_NAME}_scene_graph_example PUBLIC VERSION 17)
target_include_directories(${PROJECT_NAME}_scene_graph_example PUBLIC
    "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
    "$<INSTALL_INTERFACE:include>")
target_include_directories(${PROJECT_NAME}_scene_graph_example SYSTEM PUBLIC
    ${catkin_INCLUDE_DIRS})

add_executable(${PROJECT_NAME}_scene_graph_example_node src/scene_graph_example_node.cpp)
target_link_libraries(${PROJECT_NAME}_scene_graph_example_node ${PROJECT_NAME}_scene_graph_example ${catkin_LIBRARIES})
target_compile_options(${PROJECT_NAME}_scene_graph_example_node PRIVATE ${TESSERACT_COMPILE_OPTIONS})
target_clang_tidy(${PROJECT_NAME}_scene_graph_example_node ARGUMENTS ${TESSERACT_CLANG_TIDY_ARGS} ENABLE ${TESSERACT_ENABLE_CLANG_TIDY})
target_cxx_version(${PROJECT_NAME}_scene_graph_example_node PUBLIC VERSION 17)
target_include_directories(${PROJECT_NAME}_scene_graph_example_node PRIVATE
    "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
    "$<INSTALL_INTERFACE:include>")
target_include_directories(${PROJECT_NAME}_scene_graph_example_node SYSTEM PRIVATE
    ${catkin_INCLUDE_DIRS})

# Mark cpp header files for installation
install(DIRECTORY include/${PROJECT_NAME}
  DESTINATION include
  FILES_MATCHING PATTERN "*.h"
  PATTERN ".svn" EXCLUDE
 )

# Mark executables and/or libraries for installation
install(
  TARGETS
    ${PROJECT_NAME}_basic_cartesian_example
    ${PROJECT_NAME}_basic_cartesian_example_node
    ${PROJECT_NAME}_car_seat_example
    ${PROJECT_NAME}_car_seat_example_node
    ${PROJECT_NAME}_freespace_ompl_example
    ${PROJECT_NAME}_freespace_ompl_example_node
    ${PROJECT_NAME}_glass_upright_example
    ${PROJECT_NAME}_glass_upright_example_node
    ${PROJECT_NAME}_online_planning_example
    ${PROJECT_NAME}_online_planning_example_node
    ${PROJECT_NAME}_pick_and_place_example
    ${PROJECT_NAME}_pick_and_place_example_node
    ${PROJECT_NAME}_puzzle_piece_example
    ${PROJECT_NAME}_puzzle_piece_example_node
    ${PROJECT_NAME}_puzzle_piece_auxillary_axes_example
    ${PROJECT_NAME}_puzzle_piece_auxillary_axes_example_node
    ${PROJECT_NAME}_scene_graph_example
    ${PROJECT_NAME}_scene_graph_example_node
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

## OMPL Constraint planning is not available in version less than 1.4
#if(NOT OMPL_VERSION VERSION_LESS "1.4.0")
#  install(
#    TARGETS
#      ${PROJECT_NAME}_glass_upright_ompl_example
#      ${PROJECT_NAME}_glass_upright_ompl_example_node
#    ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#    LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#    RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
#  )
#endif()

install(
  DIRECTORY config launch meshes urdf
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

install(
  DIRECTORY test
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
  FILES_MATCHING PATTERN "*.test"
)

if(CATKIN_ENABLE_TESTING)
    find_package(rostest REQUIRED)
    find_package(GTest REQUIRED)

    add_rostest_gtest(${PROJECT_NAME}_basic_cartesian_example_unit test/basic_cartesian_example_unit.test test/basic_cartesian_example_unit.cpp)
    target_link_libraries(${PROJECT_NAME}_basic_cartesian_example_unit GTest::GTest GTest::Main ${PROJECT_NAME}_basic_cartesian_example ${catkin_LIBRARIES})
    target_compile_options(${PROJECT_NAME}_basic_cartesian_example_unit PRIVATE ${TESSERACT_COMPILE_OPTIONS})
    target_clang_tidy(${PROJECT_NAME}_basic_cartesian_example_unit ARGUMENTS ${TESSERACT_CLANG_TIDY_ARGS} ENABLE ${TESSERACT_ENABLE_CLANG_TIDY})
    target_cxx_version(${PROJECT_NAME}_basic_cartesian_example_unit PUBLIC VERSION 17)
    target_include_directories(${PROJECT_NAME}_basic_cartesian_example_unit PRIVATE
        "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
        "$<INSTALL_INTERFACE:include>")
    target_include_directories(${PROJECT_NAME}_basic_cartesian_example_unit SYSTEM PRIVATE
        ${catkin_INCLUDE_DIRS})

    add_rostest_gtest(${PROJECT_NAME}_car_seat_example_unit test/car_seat_example_unit.test test/car_seat_example_unit.cpp)
    target_link_libraries(${PROJECT_NAME}_car_seat_example_unit GTest::GTest GTest::Main ${PROJECT_NAME}_car_seat_example ${catkin_LIBRARIES})
    target_compile_options(${PROJECT_NAME}_car_seat_example_unit PRIVATE ${TESSERACT_COMPILE_OPTIONS})
    target_clang_tidy(${PROJECT_NAME}_car_seat_example_unit ARGUMENTS ${TESSERACT_CLANG_TIDY_ARGS} ENABLE ${TESSERACT_ENABLE_CLANG_TIDY})
    target_cxx_version(${PROJECT_NAME}_car_seat_example_unit PUBLIC VERSION 17)
    target_include_directories(${PROJECT_NAME}_car_seat_example_unit PRIVATE
        "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
        "$<INSTALL_INTERFACE:include>")
    target_include_directories(${PROJECT_NAME}_car_seat_example_unit SYSTEM PRIVATE
        ${catkin_INCLUDE_DIRS})

    add_rostest_gtest(${PROJECT_NAME}_glass_upright_example_unit test/glass_upright_example_unit.test test/glass_upright_example_unit.cpp)
    target_link_libraries(${PROJECT_NAME}_glass_upright_example_unit GTest::GTest GTest::Main ${PROJECT_NAME}_glass_upright_example ${catkin_LIBRARIES})
    target_compile_options(${PROJECT_NAME}_glass_upright_example_unit PRIVATE ${TESSERACT_COMPILE_OPTIONS})
    target_clang_tidy(${PROJECT_NAME}_glass_upright_example_unit ARGUMENTS ${TESSERACT_CLANG_TIDY_ARGS} ENABLE ${TESSERACT_ENABLE_CLANG_TIDY})
    target_cxx_version(${PROJECT_NAME}_glass_upright_example_unit PUBLIC VERSION 17)
    target_include_directories(${PROJECT_NAME}_glass_upright_example_unit PRIVATE
        "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
        "$<INSTALL_INTERFACE:include>")
    target_include_directories(${PROJECT_NAME}_glass_upright_example_unit SYSTEM PRIVATE
        ${catkin_INCLUDE_DIRS})

    add_rostest_gtest(${PROJECT_NAME}_puzzle_piece_example_unit test/puzzle_piece_example_unit.test test/puzzle_piece_example_unit.cpp)
    target_link_libraries(${PROJECT_NAME}_puzzle_piece_example_unit GTest::GTest GTest::Main ${PROJECT_NAME}_puzzle_piece_example ${catkin_LIBRARIES})
    target_compile_options(${PROJECT_NAME}_puzzle_piece_example_unit PRIVATE ${TESSERACT_COMPILE_OPTIONS})
    target_clang_tidy(${PROJECT_NAME}_puzzle_piece_example_unit ARGUMENTS ${TESSERACT_CLANG_TIDY_ARGS} ENABLE ${TESSERACT_ENABLE_CLANG_TIDY})
    target_cxx_version(${PROJECT_NAME}_puzzle_piece_example_unit PUBLIC VERSION 17)
    target_include_directories(${PROJECT_NAME}_puzzle_piece_example_unit PRIVATE
        "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
        "$<INSTALL_INTERFACE:include>")
    target_include_directories(${PROJECT_NAME}_puzzle_piece_example_unit SYSTEM PRIVATE
        ${catkin_INCLUDE_DIRS})

    add_rostest_gtest(${PROJECT_NAME}_puzzle_piece_auxillary_axes_example_unit test/puzzle_piece_auxillary_axes_example_unit.test test/puzzle_piece_auxillary_axes_example_unit.cpp)
    target_link_libraries(${PROJECT_NAME}_puzzle_piece_auxillary_axes_example_unit GTest::GTest GTest::Main ${PROJECT_NAME}_puzzle_piece_auxillary_axes_example ${catkin_LIBRARIES})
    target_compile_options(${PROJECT_NAME}_puzzle_piece_auxillary_axes_example_unit PRIVATE ${TESSERACT_COMPILE_OPTIONS})
    target_clang_tidy(${PROJECT_NAME}_puzzle_piece_auxillary_axes_example_unit ARGUMENTS ${TESSERACT_CLANG_TIDY_ARGS} ENABLE ${TESSERACT_ENABLE_CLANG_TIDY})
    target_cxx_version(${PROJECT_NAME}_puzzle_piece_example_unit PUBLIC VERSION 17)
    target_include_directories(${PROJECT_NAME}_puzzle_piece_auxillary_axes_example_unit PRIVATE
        "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
        "$<INSTALL_INTERFACE:include>")
    target_include_directories(${PROJECT_NAME}_puzzle_piece_auxillary_axes_example_unit SYSTEM PRIVATE
        ${catkin_INCLUDE_DIRS})

    add_rostest_gtest(${PROJECT_NAME}_pick_and_place_example_unit test/pick_and_place_example_unit.test test/pick_and_place_example_unit.cpp)
    target_link_libraries(${PROJECT_NAME}_pick_and_place_example_unit GTest::GTest GTest::Main ${PROJECT_NAME}_pick_and_place_example ${catkin_LIBRARIES})
    target_compile_options(${PROJECT_NAME}_pick_and_place_example_unit PRIVATE ${TESSERACT_COMPILE_OPTIONS})
    target_clang_tidy(${PROJECT_NAME}_pick_and_place_example_unit ARGUMENTS ${TESSERACT_CLANG_TIDY_ARGS} ENABLE ${TESSERACT_ENABLE_CLANG_TIDY})
    target_cxx_version(${PROJECT_NAME}_pick_and_place_example_unit PUBLIC VERSION 17)
    target_include_directories(${PROJECT_NAME}_pick_and_place_example_unit PRIVATE
        "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
        "$<INSTALL_INTERFACE:include>")
    target_include_directories(${PROJECT_NAME}_pick_and_place_example_unit SYSTEM PRIVATE
        ${catkin_INCLUDE_DIRS})

    add_rostest_gtest(${PROJECT_NAME}_scene_graph_example_unit test/scene_graph_example_unit.test test/scene_graph_example_unit.cpp)
    target_link_libraries(${PROJECT_NAME}_scene_graph_example_unit GTest::GTest GTest::Main ${PROJECT_NAME}_scene_graph_example ${catkin_LIBRARIES})
    target_compile_options(${PROJECT_NAME}_scene_graph_example_unit PRIVATE ${TESSERACT_COMPILE_OPTIONS})
    target_clang_tidy(${PROJECT_NAME}_scene_graph_example_unit ARGUMENTS ${TESSERACT_CLANG_TIDY_ARGS} ENABLE ${TESSERACT_ENABLE_CLANG_TIDY})
    target_cxx_version(${PROJECT_NAME}_scene_graph_example_unit PUBLIC VERSION 17)
    target_include_directories(${PROJECT_NAME}_scene_graph_example_unit PRIVATE
        "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
        "$<INSTALL_INTERFACE:include>")
    target_include_directories(${PROJECT_NAME}_scene_graph_example_unit SYSTEM PRIVATE
        ${catkin_INCLUDE_DIRS})
endif()

