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
#ifndef TESSERACT_QT_ROS_ENVIRONMENT_MONITOR_WIDGET_H
#define TESSERACT_QT_ROS_ENVIRONMENT_MONITOR_WIDGET_H

#ifndef Q_MOC_RUN
#include <memory>
#include <QWidget>

#include <ros/message_forward.h>
namespace tesseract_msgs
{
ROS_DECLARE_MESSAGE(Environment)
}
#endif

namespace YAML
{
class Node;
}

namespace Ui
{
class EnvironmentMonitorWidget;
}

namespace tesseract_gui
{
class ComponentInfo;
class EnvironmentMonitorWidget : public QWidget
{
  Q_OBJECT

public:
  explicit EnvironmentMonitorWidget(QWidget* parent = nullptr);
  explicit EnvironmentMonitorWidget(std::shared_ptr<const ComponentInfo> component_info, QWidget* parent = nullptr);

  ~EnvironmentMonitorWidget();

  void setComponentInfo(std::shared_ptr<const ComponentInfo> component_info);

  void loadConfig(const YAML::Node& config);

  YAML::Node getConfig() const;

Q_SIGNALS:
  void showMessage(const QString& message, int timeout = 0);

public Q_SLOTS:
  void onDisplayModeChanged();
  void onURDFDescriptionChanged();
  void onMonitorTopicChanged();
  void onSnapshotTopicChanged();
  void onJointStateTopicChanged();

private Q_SLOTS:
  void onStatus(bool connected);

private:
  struct Implementation;
  std::unique_ptr<Ui::EnvironmentMonitorWidget> ui;
  std::unique_ptr<Implementation> data_;

  void snapshotCallback(const tesseract_msgs::EnvironmentConstPtr& msg);
};
}  // namespace tesseract_gui

#endif  // TESSERACT_QT_ROS_ENVIRONMENT_MONITOR_WIDGET_H
