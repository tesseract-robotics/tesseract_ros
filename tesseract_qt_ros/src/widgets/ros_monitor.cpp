/**
 * @author Levi Armstrong <levi.armstrong@gmail.com>
 *
 * @copyright Copyright (C) 2023 Levi Armstrong <levi.armstrong@gmail.com>
 *
 * @par License
 * GNU Lesser General Public License Version 3, 29 June 2007
 * @par
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU Lesser General Public
 * License as published by the Free Software Foundation; either
 * version 3 of the License, or (at your option) any later version.
 * @par
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
 * Lesser General Public License for more details.
 * @par
 * You should have received a copy of the GNU Lesser General Public License
 * along with this program; if not, write to the Free Software Foundation,
 * Inc., 51 Franklin Street, Fifth Floor, Boston, MA  02110-1301, USA.
 */

#include <tesseract_qt_ros/widgets/ros_monitor.h>

#include <tesseract_qt/common/icon_utils.h>
#include <tesseract_qt/common/utils.h>

#include <ros/ros.h>
#include <mutex>

#include <QToolBar>
#include <QLabel>
#include <QMainWindow>

namespace tesseract_gui
{
void ROSStatusThread::run()
{
  while (!stopped_)
  {
    Q_EMIT status(ros::master::check());
    for (int i = 0; i < 50; ++i)
    {
      if (stopped_)
        break;

      this->msleep(100);
    }
  }
}

void ROSStatusThread::stop() { stopped_ = true; }

struct ROSMonitor::Implementation
{
  std::unique_ptr<ros::AsyncSpinner> spinner;
  ROSStatusThread status_thread;
  QToolBar* tool_bar{ nullptr };
  QLabel* indicator{ nullptr };
};

ROSMonitor::ROSMonitor() : data_(std::make_unique<Implementation>())
{
  assert(!ros::isInitialized());
  int argc = 0;
  char** argv = NULL;
  ros::init(argc, argv, "tesseract_studio_client", ros::init_options::NoSigintHandler);

  connect(&data_->status_thread, SIGNAL(status(bool)), this, SLOT(onStatus(bool)));
  data_->status_thread.start(QThread::Priority::LowestPriority);

  auto* mw = getMainWindow();
  if (mw != nullptr)
  {
    // Create ROS status tool bar
    data_->tool_bar = mw->addToolBar("ROS");

    // Add label
    auto* label = new QLabel();
    label->setText("ROS Status:");
    data_->tool_bar->addWidget(label);

    // Add indicator
    data_->indicator = new QLabel();
    data_->indicator->setPixmap(icons::getRedCircleIcon().pixmap(QSize(16, 16)));
    data_->tool_bar->addWidget(data_->indicator);
  }
}

ROSMonitor::~ROSMonitor()
{
  data_->status_thread.stop();
  data_->status_thread.wait();
}

std::shared_ptr<ROSMonitor> ROSMonitor::singleton = nullptr;
std::once_flag ROSMonitor::init_instance_flag;

std::shared_ptr<ROSMonitor> ROSMonitor::instance()
{
  std::call_once(init_instance_flag, &ROSMonitor::initSingleton);
  return singleton;
}

void ROSMonitor::initSingleton() { singleton = std::make_shared<ROSMonitor>(); }

void ROSMonitor::onStatus(bool connected)
{
  if (connected && data_->spinner == nullptr)
  {
    data_->spinner = std::make_unique<ros::AsyncSpinner>(4);
    data_->spinner->start();
    if (data_->indicator != nullptr)
      data_->indicator->setPixmap(icons::getGreenCircleIcon().pixmap(QSize(16, 16)));
  }
  else if (!connected && data_->spinner != nullptr)
  {
    data_->spinner->stop();
    if (data_->indicator != nullptr)
      data_->indicator->setPixmap(icons::getRedCircleIcon().pixmap(QSize(16, 16)));
  }
  else if (connected && data_->spinner != nullptr)
  {
    data_->spinner->start();
    if (data_->indicator != nullptr)
      data_->indicator->setPixmap(icons::getGreenCircleIcon().pixmap(QSize(16, 16)));
  }

  Q_EMIT status(connected);
}

}  // namespace tesseract_gui
