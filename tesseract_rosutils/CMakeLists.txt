cmake_minimum_required(VERSION 3.15.0)

# Extract package name and version
find_package(ros_industrial_cmake_boilerplate REQUIRED)
extract_package_metadata(pkg)
project(${pkg_extracted_name} VERSION ${pkg_extracted_version} LANGUAGES CXX)

find_package(catkin REQUIRED COMPONENTS
  tesseract_msgs
  roscpp
  roslib
  eigen_conversions
  geometry_msgs
  std_msgs
  visualization_msgs
)

find_package(Boost REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(tesseract_collision REQUIRED COMPONENTS core)
find_package(tesseract_common REQUIRED)
find_package(tesseract_environment REQUIRED)
find_package(tesseract_geometry REQUIRED) # This should not be required, must be doing something wrong when creating targets
find_package(tesseract_motion_planners REQUIRED COMPONENTS core)
find_package(tesseract_scene_graph REQUIRED)
find_package(tesseract_srdf REQUIRED)
find_package(tesseract_visualization REQUIRED)
find_package(tesseract_command_language REQUIRED)
find_package(tesseract_task_composer REQUIRED)
find_package(yaml-cpp REQUIRED)

# Catkin Packages
find_package(roscpp REQUIRED)
find_package(roslib REQUIRED)
find_package(eigen_conversions REQUIRED)
find_package(tesseract_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)

# Load variable for clang tidy args, compiler options and cxx version
tesseract_variables()

add_library(${PROJECT_NAME} SHARED src/plotting.cpp src/conversions.cpp src/utils.cpp)
target_link_libraries(${PROJECT_NAME} PUBLIC
  tesseract::tesseract_environment
  tesseract::tesseract_scene_graph
  tesseract::tesseract_srdf
  tesseract::tesseract_geometry
  tesseract::tesseract_collision_core
  tesseract::tesseract_visualization
  tesseract::tesseract_common
  tesseract::tesseract_motion_planners_core
  tesseract::tesseract_task_composer
  tesseract::tesseract_command_language
  yaml-cpp
  Eigen3::Eigen
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
    ${Boost_INCLUDE_DIRS})

configure_package(
  NAMESPACE tesseract
  TARGETS ${PROJECT_NAME}
  DEPENDENCIES
    Boost
    Eigen3
    "tesseract_collision COMPONENTS core"
    tesseract_common
    tesseract_environment
    "tesseract_motion_planners COMPONENTS core"
    tesseract_scene_graph
    tesseract_srdf
    tesseract_visualization
    tesseract_command_language
    tesseract_task_composer
    yaml-cpp
    "catkin COMPONENTS tesseract_msgs roscpp roslib eigen_conversions geometry_msgs std_msgs visualization_msgs"
)

# Mark cpp header files for installation
install(
  DIRECTORY include/${PROJECT_NAME}
  DESTINATION include
  FILES_MATCHING
  PATTERN "*.h"
  PATTERN ".svn" EXCLUDE)

if(TESSERACT_ENABLE_TESTING)
  enable_testing()
  add_subdirectory(test)
endif()
