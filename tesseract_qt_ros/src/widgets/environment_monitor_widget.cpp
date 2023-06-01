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
#include <tesseract_qt_ros/widgets/environment_monitor_widget.h>
#include <tesseract_qt_ros/widgets/ros_monitor.h>
#include "ui_environment_monitor_widget.h"

#include <tesseract_qt/common/component_info.h>
#include <tesseract_qt/common/component_info_manager.h>
#include <tesseract_qt/common/environment_manager.h>
#include <tesseract_qt/common/environment_wrapper.h>
#include <tesseract_qt/common/utils.h>

#include <tesseract_monitoring/environment_monitor.h>

#include <tesseract_msgs/EnvironmentState.h>

#include <sensor_msgs/JointState.h>

#include <boost/uuid/uuid.hpp>
#include <boost/uuid/uuid_io.hpp>
#include <boost/lexical_cast.hpp>

namespace tesseract_gui
{
struct EnvironmentMonitorWidget::Implementation
{
  Implementation()
  {
    monitor_namespace_counter++;
    monitor_namespace_id = monitor_namespace_counter;
    monitor_namespace = "environment_monitor_widget_" + std::to_string(monitor_namespace_id);
  }

  std::shared_ptr<const tesseract_gui::ComponentInfo> component_info;
  tesseract_environment::EnvironmentMonitor::Ptr monitor;
  ros::Subscriber snapshot;
  std::unique_ptr<ros::NodeHandle> nh;

  std::shared_ptr<ROSMonitor> ros_monitor;
  bool connected{ false };

  /** @brief Keeps track of how many EnvironmentMonitorWidget's have been created for the default namespace */
  static int monitor_namespace_counter;  // NOLINT
  int monitor_namespace_id{ -1 };
  std::string monitor_namespace;
};

int EnvironmentMonitorWidget::Implementation::monitor_namespace_counter = -1;  // NOLINT

EnvironmentMonitorWidget::EnvironmentMonitorWidget(QWidget* parent) : EnvironmentMonitorWidget(nullptr, parent) {}

EnvironmentMonitorWidget::EnvironmentMonitorWidget(std::shared_ptr<const ComponentInfo> component_info, QWidget* parent)

  : QWidget(parent), ui(std::make_unique<Ui::EnvironmentMonitorWidget>()), data_(std::make_unique<Implementation>())
{
  ui->setupUi(this);
  setDisabled(true);
  data_->component_info = std::move(component_info);
  data_->ros_monitor = ROSMonitor::instance();
  connect(data_->ros_monitor.get(), SIGNAL(status(bool)), this, SLOT(onStatus(bool)));
}

EnvironmentMonitorWidget::~EnvironmentMonitorWidget() = default;

void EnvironmentMonitorWidget::setComponentInfo(std::shared_ptr<const ComponentInfo> component_info)
{
  tesseract_gui::EnvironmentManager::remove(data_->component_info);
  data_->component_info = std::move(component_info);
  onDisplayModeChanged();
}

void EnvironmentMonitorWidget::loadConfig(const YAML::Node& config)
{
  if (const YAML::Node& n = config["component_info"])  // NOLINT
  {
    auto uuid = boost::lexical_cast<boost::uuids::uuid>(n.as<std::string>());
    if (uuid.is_nil())
      throw std::runtime_error("EnvironmentMonitorWidget, config component info is nil.");

    data_->component_info = ComponentInfoManager::get(uuid);
    if (data_->component_info == nullptr)
      throw std::runtime_error("EnvironmentMonitorWidget, config component info was not found.");
  }
  else
  {
    throw std::runtime_error("EnvironmentMonitorWidget, config is missing component info.");
  }

  if (const YAML::Node& n = config["display_mode"])  // NOLINT
    ui->display_mode_combo_box->setCurrentIndex(n.as<int>());

  if (const YAML::Node& n = config["urdf_param"])  // NOLINT
    ui->urdf_param_line_edit->setText(n.as<std::string>().c_str());

  if (const YAML::Node& n = config["monitor_topic"])  // NOLINT
    ui->monitor_topic_combo_box->setCurrentText(n.as<std::string>().c_str());

  if (const YAML::Node& n = config["snapshot_topic"])  // NOLINT
    ui->snapshot_topic_combo_box->setCurrentText(n.as<std::string>().c_str());

  if (const YAML::Node& n = config["joint_state_topic"])  // NOLINT
    ui->joint_state_topic_combo_box->setCurrentText(n.as<std::string>().c_str());
}

YAML::Node EnvironmentMonitorWidget::getConfig() const
{
  YAML::Node config_node;
  config_node["component_info"] = boost::uuids::to_string(data_->component_info->getNamespace());
  config_node["display_mode"] = ui->display_mode_combo_box->currentIndex();
  config_node["urdf_param"] = ui->urdf_param_line_edit->text().toStdString();
  config_node["monitor_topic"] = ui->monitor_topic_combo_box->currentText().toStdString();
  config_node["snapshot_topic"] = ui->snapshot_topic_combo_box->currentText().toStdString();
  config_node["joint_state_topic"] = ui->joint_state_topic_combo_box->currentText().toStdString();
  return config_node;
}

void EnvironmentMonitorWidget::onDisplayModeChanged()
{
  if (!data_->connected)
    return;

  if (ui->display_mode_combo_box->currentIndex() == 0)
  {
    ui->monitor_topic_label->setHidden(true);
    ui->monitor_topic_combo_box->setHidden(true);
    ui->snapshot_topic_label->setHidden(true);
    ui->snapshot_topic_combo_box->setHidden(true);
    data_->snapshot.shutdown();

    ui->urdf_param_label->setHidden(false);
    ui->urdf_param_line_edit->setHidden(false);
    onURDFDescriptionChanged();
  }
  else if (ui->display_mode_combo_box->currentIndex() == 1)
  {
    ui->urdf_param_label->setHidden(true);
    ui->urdf_param_line_edit->setHidden(true);

    ui->snapshot_topic_label->setHidden(true);
    ui->snapshot_topic_combo_box->setHidden(true);
    data_->snapshot.shutdown();

    ui->monitor_topic_label->setHidden(false);
    ui->monitor_topic_combo_box->setHidden(false);
    onMonitorTopicChanged();
  }
  else if (ui->display_mode_combo_box->currentIndex() == 2)
  {
    ui->urdf_param_label->setHidden(true);
    ui->urdf_param_line_edit->setHidden(true);
    ui->monitor_topic_label->setHidden(true);
    ui->monitor_topic_combo_box->setHidden(true);

    ui->snapshot_topic_label->setHidden(false);
    ui->snapshot_topic_combo_box->setHidden(false);

    onSnapshotTopicChanged();
  }
  onJointStateTopicChanged();
}

void EnvironmentMonitorWidget::onURDFDescriptionChanged()
{
  if (!data_->connected || ui->display_mode_combo_box->currentIndex() != 0)
    return;

  tesseract_gui::EnvironmentManager::remove(data_->component_info);

  std::string urdf_xml_string, srdf_xml_string;
  data_->nh->getParam(ui->urdf_param_line_edit->text().toStdString(), urdf_xml_string);
  data_->nh->getParam(ui->urdf_param_line_edit->text().toStdString() + "_semantic", srdf_xml_string);

  auto env = std::make_shared<tesseract_environment::Environment>();
  auto locator = std::make_shared<tesseract_rosutils::ROSResourceLocator>();
  if (env->init(urdf_xml_string, srdf_xml_string, locator))
  {
    if (data_->monitor != nullptr)
      data_->monitor->shutdown();

    data_->monitor = std::make_unique<tesseract_monitoring::ROSEnvironmentMonitor>(
        env, ui->urdf_param_line_edit->text().toStdString());
    if (data_->monitor != nullptr)
    {
      auto env_wrapper =
          std::make_shared<tesseract_gui::MonitorEnvironmentWrapper>(data_->component_info, data_->monitor);
      tesseract_gui::EnvironmentManager::set(env_wrapper);

      onJointStateTopicChanged();
    }
  }
  else
  {
    Q_EMIT showMessage("Error: URDF file failed to parse", 300);
  }
}

void EnvironmentMonitorWidget::onMonitorTopicChanged()
{
  if (!data_->connected || ui->display_mode_combo_box->currentIndex() != 1)
    return;

  if (data_->monitor != nullptr)
    data_->monitor->shutdown();

  tesseract_gui::EnvironmentManager::remove(data_->component_info);

  auto env = std::make_shared<tesseract_environment::Environment>();
  data_->monitor = std::make_unique<tesseract_monitoring::ROSEnvironmentMonitor>(env, data_->monitor_namespace);

  if (data_->monitor != nullptr)
  {
    std::string ns = getEnvNamespaceFromTopic(ui->monitor_topic_combo_box->currentText().toStdString());
    if (!ns.empty())
    {
      data_->monitor->startMonitoringEnvironment(ns);
    }
    else
    {
      Q_EMIT showMessage("Error: Invalid environment monitor topic!", 300);
    }

    auto env_wrapper =
        std::make_shared<tesseract_gui::MonitorEnvironmentWrapper>(data_->component_info, data_->monitor);
    tesseract_gui::EnvironmentManager::set(env_wrapper);

    onJointStateTopicChanged();
  }
}

void EnvironmentMonitorWidget::snapshotCallback(const tesseract_msgs::Environment::ConstPtr& msg)
{
  if (data_->monitor != nullptr)
    data_->monitor->shutdown();

  tesseract_gui::EnvironmentManager::remove(data_->component_info);

  tesseract_environment::Commands commands = tesseract_rosutils::fromMsg(msg->command_history);
  std::unordered_map<std::string, double> jv;
  tesseract_rosutils::fromMsg(jv, msg->joint_states);
  auto env = std::make_shared<tesseract_environment::Environment>();
  auto locator = std::make_shared<tesseract_rosutils::ROSResourceLocator>();
  env->setResourceLocator(locator);
  if (env->init(commands))
  {
    env->setState(jv);

    if (data_->monitor != nullptr)
      data_->monitor->shutdown();

    data_->monitor = std::make_unique<tesseract_monitoring::ROSEnvironmentMonitor>(
        env, ui->urdf_param_line_edit->text().toStdString());
    if (data_->monitor != nullptr)
    {
      auto env_wrapper =
          std::make_shared<tesseract_gui::MonitorEnvironmentWrapper>(data_->component_info, data_->monitor);
      tesseract_gui::EnvironmentManager::set(env_wrapper);

      onJointStateTopicChanged();
    }
  }
  else
  {
    Q_EMIT showMessage("Error: Snapshot failed to load from message!", 300);
  }
}

void EnvironmentMonitorWidget::onSnapshotTopicChanged()
{
  if (!data_->connected || ui->display_mode_combo_box->currentIndex() != 2)
    return;

  if (data_->monitor != nullptr)
    data_->monitor->shutdown();

  // Shutdown the callback
  data_->snapshot.shutdown();

  tesseract_gui::EnvironmentManager::remove(data_->component_info);

  // Connect to new topic
  data_->snapshot = data_->nh->subscribe(
      ui->snapshot_topic_combo_box->currentText().toStdString(), 10, &EnvironmentMonitorWidget::snapshotCallback, this);
}

void EnvironmentMonitorWidget::onJointStateTopicChanged()
{
  if (!data_->connected)
    return;

  if (data_->monitor != nullptr)
    data_->monitor->startStateMonitor(ui->joint_state_topic_combo_box->currentText().toStdString(), false);
}

void EnvironmentMonitorWidget::onStatus(bool connected)
{
  if (!data_->connected && connected)
  {
    data_->nh = std::make_unique<ros::NodeHandle>();
    data_->connected = true;

    connect(ui->display_mode_combo_box, SIGNAL(currentTextChanged(QString)), this, SLOT(onDisplayModeChanged()));
    connect(ui->urdf_param_line_edit, SIGNAL(textChanged(QString)), this, SLOT(onURDFDescriptionChanged()));
    connect(ui->monitor_topic_combo_box, SIGNAL(currentTextChanged(QString)), this, SLOT(onMonitorTopicChanged()));
    connect(ui->snapshot_topic_combo_box, SIGNAL(currentTextChanged(QString)), this, SLOT(onSnapshotTopicChanged()));
    connect(
        ui->joint_state_topic_combo_box, SIGNAL(currentTextChanged(QString)), this, SLOT(onJointStateTopicChanged()));

    ui->monitor_topic_combo_box->setMessageType<tesseract_msgs::EnvironmentState>();
    ui->snapshot_topic_combo_box->setMessageType<tesseract_msgs::Environment>();
    ui->joint_state_topic_combo_box->setMessageType<sensor_msgs::JointState>();

    ui->joint_state_topic_combo_box->setCurrentText("/joint_states");

    setComponentInfo(data_->component_info);
    setEnabled(true);
  }
  else if (data_->connected && !connected)
  {
    data_->connected = false;
    setEnabled(false);
  }
}
}  // namespace tesseract_gui
