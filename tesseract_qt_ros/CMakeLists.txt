cmake_minimum_required(VERSION 3.5.0)

# Extract package name and version
find_package(ros_industrial_cmake_boilerplate REQUIRED)
extract_package_metadata(pkg)
project(${pkg_extracted_name} VERSION ${pkg_extracted_version} LANGUAGES CXX)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  tesseract_msgs
  tesseract_rosutils
  tesseract_monitoring
)

## System dependencies are found with CMake's conventions
find_package(Boost REQUIRED thread date_time system filesystem)
find_package(tesseract_common REQUIRED)
find_package(tesseract_environment REQUIRED)
find_package(tesseract_qt REQUIRED)
find_package(Qt5 COMPONENTS Core Gui Widgets REQUIRED)

# Load variable for clang tidy args, compiler options and cxx version
tesseract_variables()

catkin_package(
    INCLUDE_DIRS include
    LIBRARIES
      ${PROJECT_NAME}_widgets
    CATKIN_DEPENDS
      roscpp
      tesseract_msgs
      tesseract_rosutils
      tesseract_monitoring
    DEPENDS
      tesseract_common
      tesseract_environment
      tesseract_qt
      QT
)

qt5_wrap_cpp(
  WIDGETS_headers_MOC
  include/tesseract_qt_ros/widgets/environment_monitor_widget.h
  include/tesseract_qt_ros/widgets/ros_topic_combo_box.h
  include/tesseract_qt_ros/widgets/ros_monitor.h)
qt5_wrap_ui(
  WIDGETS_headers_UI include/tesseract_qt_ros/widgets/environment_monitor_widget.ui)

add_library(
  ${PROJECT_NAME}_widgets
  ${WIDGETS_headers_MOC}
  ${WIDGETS_headers_UI}
  src/widgets/environment_monitor_widget.cpp
  src/widgets/ros_topic_combo_box.cpp
  src/widgets/ros_monitor.cpp)

target_link_libraries(
  ${PROJECT_NAME}_widgets
  PUBLIC tesseract::tesseract_common
         Qt5::Core
         Qt5::Gui
         Qt5::Widgets
         ${catkin_LIBRARIES}
         tesseract::tesseract_environment
         tesseract::tesseract_qt_common
         tesseract::tesseract_qt_studio_plugin_factory)
target_include_directories(
  ${PROJECT_NAME}_widgets PUBLIC "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
                                        "$<BUILD_INTERFACE:${CMAKE_BINARY_DIR}>" "$<INSTALL_INTERFACE:include>")
target_include_directories(${PROJECT_NAME}_widgets SYSTEM PUBLIC
  ${catkin_INCLUDE_DIRS})

qt5_wrap_cpp(
  PLUGINS_headers_MOC
  include/tesseract_qt_ros/plugins/studio_environment_monitor_dock_widget.h)
#qt5_wrap_ui(
#  PLUGINS_headers_UI )

add_library(
  ${PROJECT_NAME}_studio_plugins
  ${PLUGINS_headers_MOC}
#  ${PLUGINS_headers_UI}
  src/plugins/studio_environment_monitor_dock_widget.cpp
  src/plugins/studio_ros_plugin_factories.cpp)

target_link_libraries(
  ${PROJECT_NAME}_studio_plugins
  PUBLIC tesseract::tesseract_common
         Qt5::Core
         Qt5::Gui
         Qt5::Widgets
         tesseract::tesseract_qt_common
         ${PROJECT_NAME}_widgets)
target_include_directories(
  ${PROJECT_NAME}_studio_plugins PUBLIC "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
                                        "$<BUILD_INTERFACE:${CMAKE_BINARY_DIR}/studio>" "$<INSTALL_INTERFACE:include>")

add_executable(${PROJECT_NAME}_studio src/studio_app.cpp)
target_link_libraries(
  ${PROJECT_NAME}_studio
  PRIVATE Qt5::Core
          Qt5::Gui
          Qt5::Widgets
          ${catkin_LIBRARIES}
          tesseract::tesseract_qt_studio_widgets
          tesseract::tesseract_qt_common)
target_include_directories(${PROJECT_NAME}_studio SYSTEM PRIVATE ${catkin_INCLUDE_DIRS})

# Demos
add_subdirectory(demo)

# Mark executables and/or libraries for installation
install(TARGETS
    ${PROJECT_NAME}_studio
    ${PROJECT_NAME}_widgets
    ${PROJECT_NAME}_studio_plugins
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

install(DIRECTORY config
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
