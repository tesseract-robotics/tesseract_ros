cmake_minimum_required(VERSION 3.5.0)
project(tesseract_planning_server)

find_package(catkin REQUIRED COMPONENTS
  tesseract_monitoring
  tesseract_msgs
)

find_package(Boost REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(tesseract_command_language REQUIRED)
find_package(tesseract_process_managers REQUIRED)
find_package(tesseract_motion_planners REQUIRED)


catkin_package(
  INCLUDE_DIRS
    include
#  LIBRARIES
  CATKIN_DEPENDS
    tesseract_msgs
    tesseract_monitoring
  DEPENDS
    tesseract_command_language
    tesseract_process_managers
    tesseract_motion_planners
)

add_library(${PROJECT_NAME} SHARED
  src/tesseract_planning_server.cpp
  src/taskflows/descartes_taskflow.cpp
  src/taskflows/ompl_taskflow.cpp
  src/taskflows/trajopt_taskflow.cpp
  src/taskflows/simple_cartesian_taskflow.cpp
  src/taskflows/simple_freespace_taskflow.cpp)
target_link_libraries(${PROJECT_NAME} PUBLIC tesseract::tesseract_command_language tesseract::tesseract_process_managers ${catkin_LIBRARIES})
tesseract_target_compile_options(${PROJECT_NAME} PUBLIC)
tesseract_clang_tidy(${PROJECT_NAME})
target_include_directories(${PROJECT_NAME} PUBLIC
    "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
    "$<INSTALL_INTERFACE:include>")
target_include_directories(${PROJECT_NAME} SYSTEM PUBLIC
    ${EIGEN3_INCLUDE_DIRS}
    ${catkin_INCLUDE_DIRS})

add_executable(${PROJECT_NAME}_node src/tesseract_planning_server_node.cpp)
target_link_libraries(${PROJECT_NAME}_node PRIVATE ${PROJECT_NAME} ${catkin_LIBRARIES})
tesseract_target_compile_options(${PROJECT_NAME}_node PUBLIC)
tesseract_clang_tidy(${PROJECT_NAME}_node)
target_include_directories(${PROJECT_NAME}_node PUBLIC
    "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
    "$<INSTALL_INTERFACE:include>")
target_include_directories(${PROJECT_NAME}_node SYSTEM PUBLIC
    ${catkin_INCLUDE_DIRS})

add_executable(${PROJECT_NAME}_raster_example examples/server_raster_example.cpp)
target_link_libraries(${PROJECT_NAME}_raster_example PRIVATE tesseract::tesseract_command_language tesseract::tesseract_process_managers ${catkin_LIBRARIES})
tesseract_target_compile_options(${PROJECT_NAME}_raster_example PUBLIC)
tesseract_clang_tidy(${PROJECT_NAME}_raster_example)
target_include_directories(${PROJECT_NAME}_raster_example PUBLIC
    "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
    "$<INSTALL_INTERFACE:include>")
target_include_directories(${PROJECT_NAME}_raster_example SYSTEM PUBLIC
    ${catkin_INCLUDE_DIRS})

install(
  TARGETS ${PROJECT_NAME} ${PROJECT_NAME}_node ${PROJECT_NAME}_raster_example
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

#if(CATKIN_ENABLE_TESTING)
#  find_package(GTest REQUIRED)
#  catkin_add_gtest(${PROJECT_NAME}_unit test/tesseract_rosutils_unit.cpp)
#  target_link_libraries(${PROJECT_NAME}_unit GTest::GTest GTest::Main tesseract::tesseract_scene_graph tesseract::tesseract_geometry tesseract::tesseract_collision_core tesseract::tesseract_visualization tesseract::tesseract_common tesseract::tesseract_motion_planners_core ${catkin_LIBRARIES})
#  tesseract_target_compile_options(${PROJECT_NAME}_unit PRIVATE)
#  tesseract_clang_tidy(${PROJECT_NAME}_unit)
#  target_include_directories(${PROJECT_NAME}_unit PRIVATE
#      "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
#      "$<INSTALL_INTERFACE:include>")
#  target_include_directories(${PROJECT_NAME}_unit SYSTEM PRIVATE
#      ${catkin_INCLUDE_DIRS}
#      ${EIGEN3_INCLUDE_DIRS})
#endif()
