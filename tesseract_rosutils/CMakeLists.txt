cmake_minimum_required(VERSION 3.5.0)
project(tesseract_rosutils)

find_package(catkin REQUIRED COMPONENTS
  tesseract_msgs
  roscpp
  roslib
  eigen_conversions
  geometry_msgs
  std_msgs
)

find_package(Boost REQUIRED)
find_package(ros_industrial_cmake_boilerplate REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(tesseract_collision REQUIRED)
find_package(tesseract_common REQUIRED)
find_package(tesseract_environment REQUIRED)
find_package(tesseract_geometry REQUIRED) # This should not be required, must be doing something wrong when creating targets
find_package(tesseract_motion_planners REQUIRED)
find_package(tesseract_scene_graph REQUIRED)
find_package(tesseract_srdf REQUIRED)
find_package(tesseract_visualization REQUIRED)
find_package(tesseract_command_language REQUIRED)
find_package(tesseract_process_managers REQUIRED)

catkin_package(
  INCLUDE_DIRS
    include
    ${EIGEN3_INCLUDE_DIRS}
  LIBRARIES
    ${PROJECT_NAME}
  CATKIN_DEPENDS
    tesseract_msgs
    roscpp
    eigen_conversions
    roslib
    geometry_msgs
    std_msgs
  DEPENDS
    Boost
    EIGEN3
    tesseract_scene_graph
    tesseract_srdf
    tesseract_geometry
    tesseract_visualization
    tesseract_collision
    tesseract_common
    tesseract_motion_planners
    tesseract_command_language
    tesseract_environment
    tesseract_process_managers
)

# Load variable for clang tidy args, compiler options and cxx version
tesseract_variables()

add_library(${PROJECT_NAME} SHARED src/plotting.cpp src/conversions.cpp src/utils.cpp)
target_link_libraries(${PROJECT_NAME} PUBLIC
  tesseract::tesseract_environment_core
  tesseract::tesseract_environment_ofkt
  tesseract::tesseract_scene_graph
  tesseract::tesseract_srdf
  tesseract::tesseract_geometry
  tesseract::tesseract_collision_core
  tesseract::tesseract_visualization
  tesseract::tesseract_common
  tesseract::tesseract_motion_planners_core
  tesseract::tesseract_process_managers
  tesseract::tesseract_command_language
  ${Boost_LIBRARIES}
  ${catkin_LIBRARIES})
target_compile_options(${PROJECT_NAME} PRIVATE ${TESSERACT_COMPILE_OPTIONS})
target_clang_tidy(${PROJECT_NAME} ARGUMENTS ${TESSERACT_CLANG_TIDY_ARGS} ENABLE ${TESSERACT_ENABLE_CLANG_TIDY})
target_cxx_version(${PROJECT_NAME} PUBLIC VERSION 17)
target_include_directories(${PROJECT_NAME} PUBLIC
    "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
    "$<INSTALL_INTERFACE:include>")
target_include_directories(${PROJECT_NAME} SYSTEM PUBLIC
    ${catkin_INCLUDE_DIRS}
    ${EIGEN3_INCLUDE_DIRS}
    ${Boost_INCLUDE_DIRS})

install(
  TARGETS ${PROJECT_NAME}
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

# Mark cpp header files for installation
install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  FILES_MATCHING PATTERN "*.h"
  PATTERN ".svn" EXCLUDE
 )

if(CATKIN_ENABLE_TESTING)
  find_package(GTest REQUIRED)
  find_package(tesseract_support REQUIRED)
  catkin_add_gtest(${PROJECT_NAME}_unit test/tesseract_rosutils_unit.cpp)
  target_link_libraries(${PROJECT_NAME}_unit GTest::GTest GTest::Main tesseract::tesseract_support tesseract::tesseract_scene_graph tesseract::tesseract_geometry tesseract::tesseract_collision_core tesseract::tesseract_visualization tesseract::tesseract_common tesseract::tesseract_motion_planners_core ${PROJECT_NAME} ${catkin_LIBRARIES})
  target_compile_options(${PROJECT_NAME}_unit PRIVATE ${TESSERACT_COMPILE_OPTIONS})
  target_clang_tidy(${PROJECT_NAME}_unit ARGUMENTS ${TESSERACT_CLANG_TIDY_ARGS} ENABLE ${TESSERACT_ENABLE_CLANG_TIDY})
  target_cxx_version(${PROJECT_NAME}_unit PUBLIC VERSION 17)
  target_include_directories(${PROJECT_NAME}_unit PRIVATE
      "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
      "$<INSTALL_INTERFACE:include>")
  target_include_directories(${PROJECT_NAME}_unit SYSTEM PRIVATE
      ${catkin_INCLUDE_DIRS}
      ${EIGEN3_INCLUDE_DIRS})
endif()
