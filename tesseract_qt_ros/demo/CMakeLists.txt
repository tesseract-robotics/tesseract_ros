add_executable(${PROJECT_NAME}_environment_monitor_demo environment_monitor_widget_demo.cpp)
target_link_libraries(
  ${PROJECT_NAME}_environment_monitor_demo
  PRIVATE ${PROJECT_NAME}_widgets
          tesseract::tesseract_common
          Qt5::Core
          Qt5::Widgets)

install_targets(TARGETS ${PROJECT_NAME}_environment_monitor_demo)
