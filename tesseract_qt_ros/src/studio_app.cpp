#include <tesseract_qt/studio/studio.h>
#include <tesseract_qt/common/theme_utils.h>
#include <QApplication>
#include <QDebug>
#include <QString>
#include <console_bridge/console.h>
#include <sstream>

// POSIX signal handler prototype section
#include <signal.h>
#include <stdlib.h>
#include <stdio.h>
#include <unistd.h>

// ROS section
#include <ros/init.h>
#include <ros/master.h>
#include <ros/console.h>

// SIGINT handler function
void handleShutdown(int /*s*/)
{
  ROS_INFO("Shutting down node...");
  ros::shutdown();        // Step 1: stopping ROS event loop
  QApplication::exit(0);  // Step 2: stopping Qt event loop
}

int main(int argc, char* argv[])
{
  // SIGINT handler setup
  struct sigaction sigIntHandler;
  sigIntHandler.sa_handler = handleShutdown;
  sigemptyset(&sigIntHandler.sa_mask);
  sigIntHandler.sa_flags = 0;
  sigaction(SIGINT, &sigIntHandler, NULL);

#if (QT_VERSION < QT_VERSION_CHECK(6, 0, 0))
  QCoreApplication::setAttribute(Qt::AA_UseHighDpiPixmaps);
#if QT_VERSION >= 0x050600
  QGuiApplication::setAttribute(Qt::AA_EnableHighDpiScaling);
#endif
#endif
  QApplication::setAttribute(Qt::AA_ShareOpenGLContexts);

  QApplication app(argc, argv);
  QApplication::setApplicationName("Tesseract Studio");

  console_bridge::setLogLevel(console_bridge::CONSOLE_BRIDGE_LOG_DEBUG);

  Q_INIT_RESOURCE(tesseract_qt_resources);
  Q_INIT_RESOURCE(qdarkstyle_dark);
  Q_INIT_RESOURCE(qdarkstyle_light);

  // setup stylesheet
  app.setStyleSheet(tesseract_gui::themes::getDarkTheme());

  // Instantiating ROS node object
  ros::init(argc, argv, "tesseract_studio", ros::InitOption::NoSigintHandler);

  // Running ROS non-blocking event loop
  ros::AsyncSpinner spinner(4);
  spinner.start();

  // Check if master is running, if not the node was killed by user
  if (!ros::master::check())
    return 0;

  // Show main window
  tesseract_gui::Studio widget;
  if (!widget.init(argc, argv))
    return 0;

  widget.show();

  // Running Qt event loop: blocking call
  int result = app.exec();

  // Waiting for signal handler's return
  ros::waitForShutdown();

  // Exit code: from Qt
  return result;
}
