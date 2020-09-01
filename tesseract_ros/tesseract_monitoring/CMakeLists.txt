cmake_minimum_required(VERSION 3.5.0)
project(tesseract_monitoring)

find_package(tesseract REQUIRED)
find_package(orocos_kdl REQUIRED)
find_package(tesseract_common REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(cmake_common_scripts REQUIRED)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  tesseract_msgs
  tesseract_rosutils
  dynamic_reconfigure
  pluginlib
  tf2_ros
  tf2_eigen
  visualization_msgs
)

generate_dynamic_reconfigure_options(
  cfg/EnvironmentMonitorDynamicReconfigure.cfg
)

catkin_package(
  INCLUDE_DIRS
    include
  LIBRARIES
    ${PROJECT_NAME}_environment
    ${PROJECT_NAME}_contacts
  CATKIN_DEPENDS
    roscpp
    tesseract_msgs
    tesseract_rosutils
    pluginlib
    dynamic_reconfigure
    tf2_ros
    tf2_eigen
    visualization_msgs
  DEPENDS
    EIGEN3
    tesseract
    orocos_kdl
    tesseract_common
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

# Load clang tidy args and compiler options
tesseract_clang_tidy_args()
tesseract_compile_options()

# Tesseract ROS Nodes
add_library(${PROJECT_NAME}_contacts src/contact_monitor.cpp)
target_link_libraries(${PROJECT_NAME}_contacts PRIVATE tesseract::tesseract ${catkin_LIBRARIES})
target_compile_options(${PROJECT_NAME}_contacts PRIVATE ${TESSERACT_COMPILE_OPTIONS})
target_clang_tidy(${PROJECT_NAME}_contacts ARGUMENTS ${TESSERACT_CLANG_TIDY_ARGS} ENABLE ${TESSERACT_ENABLE_CLANG_TIDY})
target_cxx_version(${PROJECT_NAME}_contacts PUBLIC VERSION 17)
target_include_directories(${PROJECT_NAME}_contacts PRIVATE "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>")
target_include_directories(${PROJECT_NAME}_contacts SYSTEM PRIVATE ${catkin_INCLUDE_DIRS})
add_dependencies(${PROJECT_NAME}_contacts ${PROJECT_NAME}_gencfg)

add_executable(${PROJECT_NAME}_contacts_node src/contact_monitor_node.cpp)
target_link_libraries(${PROJECT_NAME}_contacts_node PRIVATE tesseract::tesseract ${PROJECT_NAME}_contacts ${catkin_LIBRARIES})
target_compile_options(${PROJECT_NAME}_contacts_node PRIVATE ${TESSERACT_COMPILE_OPTIONS})
target_clang_tidy(${PROJECT_NAME}_contacts_node ARGUMENTS ${TESSERACT_CLANG_TIDY_ARGS} ENABLE ${TESSERACT_ENABLE_CLANG_TIDY})
target_cxx_version(${PROJECT_NAME}_contacts_node PRIVATE VERSION 17)
target_include_directories(${PROJECT_NAME}_contacts_node PRIVATE "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>")
target_include_directories(${PROJECT_NAME}_contacts_node SYSTEM PRIVATE ${catkin_INCLUDE_DIRS})

add_library(${PROJECT_NAME}_environment SHARED src/environment_monitor.cpp src/current_state_monitor.cpp)
target_link_libraries(${PROJECT_NAME}_environment PUBLIC tesseract::tesseract ${catkin_LIBRARIES} ${Boost_LIBRARIES})
target_compile_options(${PROJECT_NAME}_environment PRIVATE ${TESSERACT_COMPILE_OPTIONS})
target_clang_tidy(${PROJECT_NAME}_environment ARGUMENTS ${TESSERACT_CLANG_TIDY_ARGS} ENABLE ${TESSERACT_ENABLE_CLANG_TIDY})
target_cxx_version(${PROJECT_NAME}_environment PUBLIC VERSION 17)
target_include_directories(${PROJECT_NAME}_environment PUBLIC "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>")
target_include_directories(${PROJECT_NAME}_environment SYSTEM PUBLIC ${catkin_INCLUDE_DIRS} ${Boost_INCLUDE_DIRS})
add_dependencies(${PROJECT_NAME}_environment ${PROJECT_NAME}_gencfg)

add_executable(${PROJECT_NAME}_environment_node src/environment_monitor_node.cpp)
target_link_libraries(${PROJECT_NAME}_environment_node PRIVATE ${PROJECT_NAME}_environment ${catkin_LIBRARIES})
target_compile_options(${PROJECT_NAME}_environment_node PRIVATE ${TESSERACT_COMPILE_OPTIONS})
target_clang_tidy(${PROJECT_NAME}_environment_node ARGUMENTS ${TESSERACT_CLANG_TIDY_ARGS} ENABLE ${TESSERACT_ENABLE_CLANG_TIDY})
target_cxx_version(${PROJECT_NAME}_environment_node PRIVATE VERSION 17)
target_include_directories(${PROJECT_NAME}_environment_node PUBLIC "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>")
target_include_directories(${PROJECT_NAME}_environment_node SYSTEM PUBLIC ${catkin_INCLUDE_DIRS})

add_executable(demo_scene demos/demo_scene.cpp)
target_link_libraries(demo_scene PRIVATE ${PROJECT_NAME}_environment ${catkin_LIBRARIES})
target_compile_options(demo_scene PRIVATE ${TESSERACT_COMPILE_OPTIONS})
target_clang_tidy(demo_scene ARGUMENTS ${TESSERACT_CLANG_TIDY_ARGS} ENABLE ${TESSERACT_ENABLE_CLANG_TIDY})
target_cxx_version(demo_scene PRIVATE VERSION 17)
target_include_directories(demo_scene PUBLIC "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>")
target_include_directories(demo_scene SYSTEM PUBLIC ${catkin_INCLUDE_DIRS})

# Mark executables and/or libraries for installation
install(TARGETS ${PROJECT_NAME}_environment ${PROJECT_NAME}_contacts ${PROJECT_NAME}_contacts_node ${PROJECT_NAME}_environment_node demo_scene
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

install(DIRECTORY launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
