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

#include <tesseract_qt_ros/plugins/studio_environment_monitor_dock_widget.h>
#include <tesseract_qt_ros/widgets/environment_monitor_widget.h>

#include <tesseract_qt/common/component_info.h>
#include <tesseract_qt/common/component_info_manager.h>
#include <tesseract_qt/common/icon_utils.h>
#include <tesseract_qt/common/utils.h>
#include <tesseract_qt/common/widgets/component_info_dialog.h>

#include <QMenu>
#include <QAction>
#include <QStatusBar>

namespace tesseract_gui
{
struct StudioEnvironmentMonitorDockWidget::Implementation
{
  std::shared_ptr<const ComponentInfo> component_info;
  EnvironmentMonitorWidget* widget{ nullptr };
};

StudioEnvironmentMonitorDockWidget::StudioEnvironmentMonitorDockWidget(const QString& title, QWidget* parent)
  : StudioDockWidget(title, parent), data_(std::make_unique<Implementation>())
{
}

StudioEnvironmentMonitorDockWidget::~StudioEnvironmentMonitorDockWidget() = default;

std::string StudioEnvironmentMonitorDockWidget::getFactoryClassName() const
{
  return "StudioEnvironmentMonitorDockWidgetFactory";
}

void StudioEnvironmentMonitorDockWidget::loadConfig(const YAML::Node& config)
{
  data_->widget = new EnvironmentMonitorWidget();
  data_->widget->loadConfig(config);
  connect(data_->widget, SIGNAL(showMessage(QString, int)), this, SLOT(onShowMessage(QString, int)));
  setWidget(data_->widget);
  setFeature(ads::CDockWidget::DockWidgetFocusable, true);
}

YAML::Node StudioEnvironmentMonitorDockWidget::getConfig() const { return data_->widget->getConfig(); }

void StudioEnvironmentMonitorDockWidget::onInitialize()
{
  if (isInitialized())
    return;

  ComponentInfoDialog dialog(this);
  if (dialog.exec())
  {
    data_->component_info = dialog.getComponentInfo();
    if (data_->component_info == nullptr)
      return;

    data_->widget = new EnvironmentMonitorWidget(data_->component_info);
    connect(data_->widget, SIGNAL(showMessage(QString, int)), this, SLOT(onShowMessage(QString, int)));
    setWidget(data_->widget);
    setFeature(ads::CDockWidget::DockWidgetFocusable, true);
  }
}

void StudioEnvironmentMonitorDockWidget::onShowMessage(const QString& message, int timeout)
{
  getMainWindowStatusBar()->showMessage(message, timeout);
}

}  // namespace tesseract_gui
