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
#ifndef TESSERACT_QT_ROS_MONITOR_H
#define TESSERACT_QT_ROS_MONITOR_H

#include <memory>
#include <QThread>

namespace ros
{
class AsyncSpinner;
}

namespace tesseract_gui
{
/** @brief This thread monitors this nodes connection to ros core */
class ROSStatusThread : public QThread
{
  Q_OBJECT
public:
  void run() override;

  void stop();

Q_SIGNALS:
  void status(bool connected);

private:
  bool stopped_{ false };
};

/**
 * @brief This is a singleton class to manage initializing ros and
 * creating an async spinner to handle communication
 */
class ROSMonitor : public QObject
{
  Q_OBJECT
public:
  ROSMonitor();
  ~ROSMonitor();
  ROSMonitor(const ROSMonitor&) = delete;
  ROSMonitor& operator=(const ROSMonitor&) = delete;
  ROSMonitor(ROSMonitor&&) = delete;
  ROSMonitor& operator=(ROSMonitor&&) = delete;

  static std::shared_ptr<ROSMonitor> instance();

Q_SIGNALS:
  void status(bool connected);

private Q_SLOTS:
  void onStatus(bool connected);

private:
  struct Implementation;
  std::unique_ptr<Implementation> data_;

  static std::shared_ptr<ROSMonitor> singleton;
  static std::once_flag init_instance_flag;
  static void initSingleton();
};

}  // namespace tesseract_gui

#endif  // TESSERACT_QT_ROS_MONITOR_H
