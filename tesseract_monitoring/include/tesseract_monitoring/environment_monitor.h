/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2011, Willow Garage, Inc.
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of Willow Garage nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *********************************************************************/

/* Author: Ioan Sucan */

#ifndef TESSERACT_MONITORING_ENVIRONMENT_H
#define TESSERACT_MONITORING_ENVIRONMENT_H

#include <tesseract_common/macros.h>
TESSERACT_COMMON_IGNORE_WARNINGS_PUSH
#include <memory>
TESSERACT_COMMON_IGNORE_WARNINGS_POP

#include <tesseract_monitoring/constants.h>
#include <tesseract_environment/environment_monitor.h>

namespace tesseract_monitoring
{
class CurrentStateMonitor;

/**
 * @brief TesseractMonitor
 * Subscribes to the topic \e tesseract_environment */
class ROSEnvironmentMonitor : public tesseract_environment::EnvironmentMonitor
{
public:
  using Ptr = std::shared_ptr<ROSEnvironmentMonitor>;
  using ConstPtr = std::shared_ptr<const ROSEnvironmentMonitor>;
  using UPtr = std::unique_ptr<ROSEnvironmentMonitor>;
  using ConstUPtr = std::unique_ptr<const ROSEnvironmentMonitor>;

  /**
   * @brief Constructor
   * @param robot_description The name of the ROS parameter that contains the URDF (in string format)
   * @param monitor_namespace A name identifying this monitor, must be unique
   */
  ROSEnvironmentMonitor(std::string robot_description, std::string monitor_namespace);

  /**
   * @brief Constructor
   * @param env The environment
   * @param monitor_namespace A name identifying this monitor, must be unique
   */
  ROSEnvironmentMonitor(std::shared_ptr<tesseract_environment::Environment> env, std::string monitor_namespace);

  ~ROSEnvironmentMonitor() override;
  ROSEnvironmentMonitor(const ROSEnvironmentMonitor&) = delete;
  ROSEnvironmentMonitor& operator=(const ROSEnvironmentMonitor&) = delete;
  ROSEnvironmentMonitor(ROSEnvironmentMonitor&&) = delete;
  ROSEnvironmentMonitor& operator=(ROSEnvironmentMonitor&&) = delete;

  /** @brief Get the stored robot description
   *  @return An instance of the stored robot description*/
  const std::string& getURDFDescription() const;

  bool waitForConnection(std::chrono::duration<double> duration = std::chrono::seconds(0)) const override final;

  void startPublishingEnvironment() override final;

  void stopPublishingEnvironment() override final;

  void setEnvironmentPublishingFrequency(double hz) override final;

  double getEnvironmentPublishingFrequency() const override final;

  void startStateMonitor(const std::string& joint_states_topic = DEFAULT_JOINT_STATES_TOPIC,
                         bool publish_tf = true) override final;

  void stopStateMonitor() override final;

  void setStateUpdateFrequency(double hz = 10) override final;

  double getStateUpdateFrequency() const override final;

  void updateEnvironmentWithCurrentState() override final;

  void startMonitoringEnvironment(const std::string& monitored_namespace,
                                  tesseract_environment::MonitoredEnvironmentMode mode =
                                      tesseract_environment::MonitoredEnvironmentMode::DEFAULT) override final;

  void stopMonitoringEnvironment() override final;

  bool waitForCurrentState(std::chrono::duration<double> duration = std::chrono::seconds(1)) override final;

  void shutdown() override final;

  /** @brief Get the stored instance of the stored current state monitor
   *  @return An instance of the stored current state monitor*/
  const CurrentStateMonitor& getStateMonitor() const;
  CurrentStateMonitor& getStateMonitor();

private:
  struct Implementation;
  std::unique_ptr<Implementation> impl_;
};

}  // namespace tesseract_monitoring

#endif
