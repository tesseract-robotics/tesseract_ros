cmake_minimum_required(VERSION 3.15.0)

# Extract package name and version
find_package(ros_industrial_cmake_boilerplate REQUIRED)
extract_package_metadata(pkg)
project(${pkg_extracted_name} VERSION ${pkg_extracted_version} LANGUAGES CXX)

find_package(tesseract_environment REQUIRED)
find_package(tesseract_common REQUIRED)
find_package(tesseract_rosutils REQUIRED)
find_package(orocos_kdl REQUIRED)
find_package(Eigen3 REQUIRED)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  tesseract_msgs
  dynamic_reconfigure
  pluginlib
  tf2_ros
  tf2_eigen
  visualization_msgs
)

# Load variable for clang tidy args, compiler options and cxx version
tesseract_variables()

# Tesseract ROS Nodes
add_library(${PROJECT_NAME}_environment SHARED src/environment_monitor.cpp src/current_state_monitor.cpp)
target_link_libraries(${PROJECT_NAME}_environment PUBLIC tesseract::tesseract_common tesseract::tesseract_environment tesseract::tesseract_rosutils ${catkin_LIBRARIES} ${Boost_LIBRARIES})
target_compile_options(${PROJECT_NAME}_environment PRIVATE ${TESSERACT_COMPILE_OPTIONS})
target_clang_tidy(${PROJECT_NAME}_environment ARGUMENTS ${TESSERACT_CLANG_TIDY_ARGS} ENABLE ${TESSERACT_ENABLE_CLANG_TIDY})
target_cxx_version(${PROJECT_NAME}_environment PUBLIC VERSION 17)
target_include_directories(${PROJECT_NAME}_environment PUBLIC "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>")
target_include_directories(${PROJECT_NAME}_environment SYSTEM PUBLIC ${catkin_INCLUDE_DIRS} ${Boost_INCLUDE_DIRS})

add_executable(${PROJECT_NAME}_environment_node src/environment_monitor_node.cpp)
target_link_libraries(${PROJECT_NAME}_environment_node PRIVATE ${PROJECT_NAME}_environment ${catkin_LIBRARIES})
target_compile_options(${PROJECT_NAME}_environment_node PRIVATE ${TESSERACT_COMPILE_OPTIONS})
target_clang_tidy(${PROJECT_NAME}_environment_node ARGUMENTS ${TESSERACT_CLANG_TIDY_ARGS} ENABLE ${TESSERACT_ENABLE_CLANG_TIDY})
target_cxx_version(${PROJECT_NAME}_environment_node PRIVATE VERSION 17)
target_include_directories(${PROJECT_NAME}_environment_node PUBLIC "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>")
target_include_directories(${PROJECT_NAME}_environment_node SYSTEM PUBLIC ${catkin_INCLUDE_DIRS})

add_library(${PROJECT_NAME}_contacts src/contact_monitor.cpp)
target_link_libraries(${PROJECT_NAME}_contacts PUBLIC tesseract::tesseract_common tesseract::tesseract_environment tesseract::tesseract_rosutils ${PROJECT_NAME}_environment ${catkin_LIBRARIES})
target_compile_options(${PROJECT_NAME}_contacts PRIVATE ${TESSERACT_COMPILE_OPTIONS})
target_clang_tidy(${PROJECT_NAME}_contacts ARGUMENTS ${TESSERACT_CLANG_TIDY_ARGS} ENABLE ${TESSERACT_ENABLE_CLANG_TIDY})
target_cxx_version(${PROJECT_NAME}_contacts PUBLIC VERSION 17)
target_include_directories(${PROJECT_NAME}_contacts PRIVATE "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>")
target_include_directories(${PROJECT_NAME}_contacts SYSTEM PRIVATE ${catkin_INCLUDE_DIRS})

add_executable(${PROJECT_NAME}_contacts_node src/contact_monitor_node.cpp)
target_link_libraries(${PROJECT_NAME}_contacts_node PRIVATE ${PROJECT_NAME}_contacts ${catkin_LIBRARIES})
target_compile_options(${PROJECT_NAME}_contacts_node PRIVATE ${TESSERACT_COMPILE_OPTIONS})
target_clang_tidy(${PROJECT_NAME}_contacts_node ARGUMENTS ${TESSERACT_CLANG_TIDY_ARGS} ENABLE ${TESSERACT_ENABLE_CLANG_TIDY})
target_cxx_version(${PROJECT_NAME}_contacts_node PRIVATE VERSION 17)
target_include_directories(${PROJECT_NAME}_contacts_node PRIVATE "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>")
target_include_directories(${PROJECT_NAME}_contacts_node SYSTEM PRIVATE ${catkin_INCLUDE_DIRS})

add_library(${PROJECT_NAME}_environment_interface SHARED src/environment_monitor_interface.cpp)
target_link_libraries(${PROJECT_NAME}_environment_interface PUBLIC tesseract::tesseract_common tesseract::tesseract_environment tesseract::tesseract_rosutils ${catkin_LIBRARIES})
target_compile_options(${PROJECT_NAME}_environment_interface PRIVATE ${TESSERACT_COMPILE_OPTIONS})
target_clang_tidy(${PROJECT_NAME}_environment_interface ARGUMENTS ${TESSERACT_CLANG_TIDY_ARGS} ENABLE ${TESSERACT_ENABLE_CLANG_TIDY})
target_cxx_version(${PROJECT_NAME}_environment_interface PUBLIC VERSION 17)
target_include_directories(${PROJECT_NAME}_environment_interface PUBLIC "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>")
target_include_directories(${PROJECT_NAME}_environment_interface SYSTEM PUBLIC ${catkin_INCLUDE_DIRS})

add_executable(demo_scene demos/demo_scene.cpp)
target_link_libraries(demo_scene PRIVATE ${PROJECT_NAME}_environment ${catkin_LIBRARIES})
target_compile_options(demo_scene PRIVATE ${TESSERACT_COMPILE_OPTIONS})
target_clang_tidy(demo_scene ARGUMENTS ${TESSERACT_CLANG_TIDY_ARGS} ENABLE ${TESSERACT_ENABLE_CLANG_TIDY})
target_cxx_version(demo_scene PRIVATE VERSION 17)
target_include_directories(demo_scene PUBLIC "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>")
target_include_directories(demo_scene SYSTEM PUBLIC ${catkin_INCLUDE_DIRS})

catkin_package(
  SKIP_CMAKE_CONFIG_GENERATION
  SKIP_PKG_CONFIG_GENERATION
)

configure_package(
  NAMESPACE tesseract
  TARGETS
    ${PROJECT_NAME}_environment
    ${PROJECT_NAME}_contacts
    ${PROJECT_NAME}_contacts_node
    ${PROJECT_NAME}_environment_node
    ${PROJECT_NAME}_environment_interface
    demo_scene
  DEPENDENCIES
    Eigen3
    orocos_kdl
    tesseract_common
    tesseract_environment
    tesseract_rosutils
    "catkin REQUIRED COMPONENTS roscpp tesseract_msgs dynamic_reconfigure pluginlib tf2_ros tf2_eigen visualization_msgs"
)

# Mark cpp header files for installation
install(
  DIRECTORY include/${PROJECT_NAME}
  DESTINATION include
  FILES_MATCHING
  PATTERN "*.h"
  PATTERN ".svn" EXCLUDE)

if(NOT ${CATKIN_INSTALL_INTO_PREFIX_ROOT})
  foreach(dir launch)
    install(DIRECTORY ${dir}/ DESTINATION share/${PROJECT_NAME}/${dir})
  endforeach()
endif()

