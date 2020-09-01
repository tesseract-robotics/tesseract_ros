cmake_minimum_required(VERSION 3.5.0)
project(tesseract_plugins)

find_package(catkin REQUIRED COMPONENTS
  tesseract_msgs
  roscpp
  eigen_conversions
  pluginlib
)

find_package(Eigen3 REQUIRED)
find_package(tesseract_collision REQUIRED)
find_package(tesseract_common REQUIRED)
find_package(cmake_common_scripts REQUIRED)

catkin_package(
  INCLUDE_DIRS
    ${EIGEN3_INCLUDE_DIRS}
#  LIBRARIES
  CATKIN_DEPENDS
    tesseract_msgs
    roscpp
    eigen_conversions
    pluginlib
  DEPENDS
    EIGEN3
    tesseract_collision
    tesseract_common
)

# Load clang tidy args and compiler options
tesseract_clang_tidy_args()
tesseract_compile_options()

add_library(tesseract_collision_bullet_plugin src/plugins/bullet_plugin.cpp)
target_link_libraries(tesseract_collision_bullet_plugin PRIVATE tesseract::tesseract_collision_core tesseract::tesseract_collision_bullet ${catkin_LIBRARIES})
target_compile_options(tesseract_collision_bullet_plugin PRIVATE ${TESSERACT_COMPILE_OPTIONS})
target_cxx_version(tesseract_collision_bullet_plugin PUBLIC VERSION 14)
target_include_directories(tesseract_collision_bullet_plugin SYSTEM PRIVATE
    ${EIGEN3_INCLUDE_DIRS}
    ${catkin_INCLUDE_DIRS})

add_library(tesseract_collision_fcl_plugin src/plugins/fcl_plugin.cpp)
target_link_libraries(tesseract_collision_fcl_plugin PRIVATE tesseract::tesseract_collision_core tesseract::tesseract_collision_fcl ${catkin_LIBRARIES})
target_compile_options(tesseract_collision_fcl_plugin PRIVATE ${TESSERACT_COMPILE_OPTIONS})
target_cxx_version(tesseract_collision_fcl_plugin PUBLIC VERSION 14)
target_include_directories(tesseract_collision_fcl_plugin SYSTEM PRIVATE
    ${catkin_INCLUDE_DIRS})

# Mark executables and/or libraries for installation
install(TARGETS tesseract_collision_bullet_plugin tesseract_collision_fcl_plugin
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(FILES tesseract_collision_bullet_plugin_description.xml tesseract_collision_fcl_plugin_description.xml
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
