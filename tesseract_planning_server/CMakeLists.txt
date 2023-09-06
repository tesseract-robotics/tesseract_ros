cmake_minimum_required(VERSION 3.5.0)

# Extract package name and version
find_package(ros_industrial_cmake_boilerplate REQUIRED)
extract_package_metadata(pkg)
project(${pkg_extracted_name} VERSION ${pkg_extracted_version} LANGUAGES CXX)

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
find_package(tesseract_motion_planners REQUIRED COMPONENTS core simple OPTIONAL_COMPONENTS descartes ompl trajopt trajopt_ifopt)
find_package(tesseract_task_composer REQUIRED COMPONENTS core planning)
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
    tesseract_motion_planners
    tesseract_task_composer
    tesseract_common
)

# Load variable for clang tidy args, compiler options and cxx version
tesseract_variables()

set(LIB_COMPILE_DEFINITIONS ${TESSERACT_COMPILE_DEFINITIONS})
set(LIB_SOURCE_LINK_LIBRARIES
  tesseract::tesseract_command_language
  tesseract::tesseract_motion_planners_core
  tesseract::tesseract_motion_planners_simple
  tesseract::tesseract_task_composer
  tesseract::tesseract_task_composer_planning)

if(tesseract_motion_planners_trajopt_FOUND)
  list(APPEND LIB_SOURCE_LINK_LIBRARIES tesseract::tesseract_motion_planners_trajopt)
  list(APPEND LIB_COMPILE_DEFINITIONS "-DTESSERACT_PLANNING_SERVER_HAS_TRAJOPT")
endif()

if(tesseract_motion_planners_trajopt_ifopt_FOUND)
  list(APPEND LIB_SOURCE_LINK_LIBRARIES tesseract::tesseract_motion_planners_trajopt_ifopt)
  list(APPEND LIB_COMPILE_DEFINITIONS "-DTESSERACT_PLANNING_SERVER_HAS_TRAJOPT_IFOPT")
endif()

if(tesseract_motion_planners_ompl_FOUND)
  list(APPEND LIB_SOURCE_LINK_LIBRARIES tesseract::tesseract_motion_planners_ompl)
  list(APPEND LIB_COMPILE_DEFINITIONS "-DTESSERACT_PLANNING_SERVER_HAS_OMPL")
endif()

if(tesseract_motion_planners_descartes_FOUND)
  list(APPEND LIB_SOURCE_LINK_LIBRARIES tesseract::tesseract_motion_planners_descartes)
  list(APPEND LIB_COMPILE_DEFINITIONS "-DTESSERACT_PLANNING_SERVER_HAS_DESCARTES")
endif()

add_library(${PROJECT_NAME} SHARED src/tesseract_planning_server.cpp)
target_link_libraries(${PROJECT_NAME} PUBLIC ${LIB_SOURCE_LINK_LIBRARIES} ${catkin_LIBRARIES})
target_compile_options(${PROJECT_NAME} PRIVATE ${TESSERACT_COMPILE_OPTIONS})
target_compile_definitions(${PROJECT_NAME} PUBLIC ${LIB_COMPILE_DEFINITIONS})
target_clang_tidy(${PROJECT_NAME} ARGUMENTS ${TESSERACT_CLANG_TIDY_ARGS} ENABLE ${TESSERACT_ENABLE_CLANG_TIDY})
target_cxx_version(${PROJECT_NAME} PUBLIC VERSION 17)
target_include_directories(${PROJECT_NAME} PUBLIC
    "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
    "$<INSTALL_INTERFACE:include>")
target_include_directories(${PROJECT_NAME} SYSTEM PUBLIC
    ${EIGEN3_INCLUDE_DIRS}
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
target_link_libraries(${PROJECT_NAME}_raster_example PRIVATE tesseract::tesseract_command_language tesseract::tesseract_task_composer ${catkin_LIBRARIES})
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

install(DIRECTORY launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
