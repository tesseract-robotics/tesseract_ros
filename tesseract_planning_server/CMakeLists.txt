cmake_minimum_required(VERSION 3.5.0)
project(tesseract_planning_server)

find_package(catkin REQUIRED COMPONENTS
  tesseract_monitoring
  tesseract_msgs
  tesseract_rosutils
  tf2_ros
  tf2_eigen
)

find_package(Boost REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(tesseract_command_language REQUIRED)
find_package(tesseract_process_managers REQUIRED)
find_package(ros_industrial_cmake_boilerplate REQUIRED)
find_package(tesseract_common REQUIRED)

catkin_package(
  INCLUDE_DIRS
    include
  LIBRARIES
    ${PROJECT_NAME}
  CATKIN_DEPENDS
    tesseract_msgs
    tesseract_monitoring
    tesseract_rosutils
    tf2_ros
    tf2_eigen
  DEPENDS
    tesseract_command_language
    tesseract_process_managers
    tesseract_common
)

# Load variable for clang tidy args, compiler options and cxx version
tesseract_variables()

add_library(${PROJECT_NAME} SHARED
  src/tesseract_planning_server.cpp)
target_link_libraries(${PROJECT_NAME} PUBLIC tesseract::tesseract_command_language tesseract::tesseract_process_managers ${catkin_LIBRARIES})
target_compile_options(${PROJECT_NAME} PRIVATE ${TESSERACT_COMPILE_OPTIONS})
target_clang_tidy(${PROJECT_NAME} ARGUMENTS ${TESSERACT_CLANG_TIDY_ARGS} ENABLE ${TESSERACT_ENABLE_CLANG_TIDY})
target_cxx_version(${PROJECT_NAME} PUBLIC VERSION 17)
target_include_directories(${PROJECT_NAME} PUBLIC
    "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
    "$<INSTALL_INTERFACE:include>")
target_include_directories(${PROJECT_NAME} SYSTEM PUBLIC
    ${EIGEN3_INCLUDE_DIRS}
    ${catkin_INCLUDE_DIRS})

add_executable(${PROJECT_NAME}_archive_viewer_node src/archive_viewer_node.cpp)
target_link_libraries(${PROJECT_NAME}_archive_viewer_node PRIVATE ${PROJECT_NAME} ${catkin_LIBRARIES})
target_compile_options(${PROJECT_NAME}_archive_viewer_node PRIVATE ${TESSERACT_COMPILE_OPTIONS})
target_clang_tidy(${PROJECT_NAME}_archive_viewer_node ARGUMENTS ${TESSERACT_CLANG_TIDY_ARGS} ENABLE ${TESSERACT_ENABLE_CLANG_TIDY})
target_cxx_version(${PROJECT_NAME}_archive_viewer_node PRIVATE VERSION 17)
target_include_directories(${PROJECT_NAME}_archive_viewer_node PUBLIC
    "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
    "$<INSTALL_INTERFACE:include>")
target_include_directories(${PROJECT_NAME}_archive_viewer_node SYSTEM PUBLIC
    ${catkin_INCLUDE_DIRS})


add_executable(${PROJECT_NAME}_node src/tesseract_planning_server_node.cpp)
target_link_libraries(${PROJECT_NAME}_node PRIVATE ${PROJECT_NAME} ${catkin_LIBRARIES})
target_compile_options(${PROJECT_NAME}_node PRIVATE ${TESSERACT_COMPILE_OPTIONS})
target_clang_tidy(${PROJECT_NAME}_node ARGUMENTS ${TESSERACT_CLANG_TIDY_ARGS} ENABLE ${TESSERACT_ENABLE_CLANG_TIDY})
target_cxx_version(${PROJECT_NAME}_node PRIVATE VERSION 17)
target_include_directories(${PROJECT_NAME}_node PUBLIC
    "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
    "$<INSTALL_INTERFACE:include>")
target_include_directories(${PROJECT_NAME}_node SYSTEM PUBLIC
    ${catkin_INCLUDE_DIRS})

add_executable(${PROJECT_NAME}_raster_example examples/server_raster_example.cpp)
target_link_libraries(${PROJECT_NAME}_raster_example PRIVATE tesseract::tesseract_command_language tesseract::tesseract_process_managers ${catkin_LIBRARIES})
target_compile_options(${PROJECT_NAME}_raster_example PRIVATE ${TESSERACT_COMPILE_OPTIONS})
target_clang_tidy(${PROJECT_NAME}_raster_example ARGUMENTS ${TESSERACT_CLANG_TIDY_ARGS} ENABLE ${TESSERACT_ENABLE_CLANG_TIDY})
target_cxx_version(${PROJECT_NAME}_raster_example PRIVATE VERSION 17)
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

install(DIRECTORY launch config
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
