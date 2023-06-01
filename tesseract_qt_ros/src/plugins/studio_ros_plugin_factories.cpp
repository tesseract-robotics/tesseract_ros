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
#include <tesseract_qt/studio/studio_dock_widget_factory.h>
#include <tesseract_qt_ros/plugins/studio_environment_monitor_dock_widget.h>

namespace tesseract_gui
{
using StudioEnvironmentMonitorDockWidgetFactory = StudioDockWidgetFactoryImpl<StudioEnvironmentMonitorDockWidget>;
}  // namespace tesseract_gui

// NOLINTNEXTLINE(cppcoreguidelines-avoid-non-const-global-variables)
TESSERACT_ADD_STUDIO_PLUGIN(tesseract_gui::StudioEnvironmentMonitorDockWidgetFactory,
                            StudioEnvironmentMonitorDockWidgetFactory)
