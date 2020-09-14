/**
 * @file tesseract_planning_server.cpp
 * @brief A planning server with a default set of motion planners
 *
 * @author Levi Armstrong
 * @date August 18, 2020
 * @version TODO
 * @bug No known bugs
 *
 * @copyright Copyright (c) 2020, Southwest Research Institute
 *
 * @par License
 * Software License Agreement (Apache License)
 * @par
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 * http://www.apache.org/licenses/LICENSE-2.0
 * @par
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */
#include <tesseract_common/macros.h>
TESSERACT_COMMON_IGNORE_WARNINGS_PUSH
#include <ros/ros.h>
TESSERACT_COMMON_IGNORE_WARNINGS_POP

#include <tesseract_planning_server/tesseract_planning_server.h>

#include <tesseract_motion_planners/simple/profile/simple_planner_default_plan_profile.h>
#include <tesseract_motion_planners/ompl/profile/ompl_default_plan_profile.h>
#include <tesseract_motion_planners/descartes/profile/descartes_default_plan_profile.h>
#include <tesseract_motion_planners/trajopt/profile/trajopt_default_composite_profile.h>
#include <tesseract_motion_planners/trajopt/profile/trajopt_default_plan_profile.h>

#include <tesseract_process_managers/process_managers/raster_process_manager.h>
#include <tesseract_process_managers/process_managers/raster_dt_process_manager.h>
#include <tesseract_process_managers/process_managers/raster_waad_process_manager.h>
#include <tesseract_process_managers/process_managers/raster_waad_dt_process_manager.h>

#include <tesseract_process_managers/process_managers/simple_process_manager.h>
#include <tesseract_process_managers/taskflows/descartes_taskflow.h>
#include <tesseract_process_managers/taskflows/ompl_taskflow.h>
#include <tesseract_process_managers/taskflows/trajopt_taskflow.h>
#include <tesseract_process_managers/taskflows/cartesian_taskflow.h>
#include <tesseract_process_managers/taskflows/freespace_taskflow.h>

#include <tesseract_command_language/utils/utils.h>
#include <tesseract_command_language/deserialize.h>
#include <tesseract_command_language/serialize.h>

namespace tesseract_planning_server
{
const std::string TesseractPlanningServer::DEFAULT_GET_MOTION_PLAN_ACTION = "tesseract_get_motion_plan";

TesseractPlanningServer::TesseractPlanningServer(const std::string& robot_description,
                                                 std::string name,
                                                 std::string discrete_plugin,
                                                 std::string continuous_plugin)
  : nh_("~")
  , environment_(robot_description, name, discrete_plugin, continuous_plugin)
  , motion_plan_server_(nh_,
                        DEFAULT_GET_MOTION_PLAN_ACTION,
                        boost::bind(&TesseractPlanningServer::onMotionPlanningCallback, this, _1),
                        true)
{
  loadDefaultPlannerProfiles();
}

TesseractPlanningServer::TesseractPlanningServer(std::shared_ptr<tesseract::Tesseract> tesseract,
                                                 std::string name,
                                                 std::string discrete_plugin,
                                                 std::string continuous_plugin)
  : nh_("~")
  , environment_(tesseract, name, discrete_plugin, continuous_plugin)
  , motion_plan_server_(nh_,
                        DEFAULT_GET_MOTION_PLAN_ACTION,
                        boost::bind(&TesseractPlanningServer::onMotionPlanningCallback, this, _1),
                        true)
{
  loadDefaultPlannerProfiles();
}

tesseract_monitoring::EnvironmentMonitor& TesseractPlanningServer::getEnvironmentMonitor() { return environment_; }
const tesseract_monitoring::EnvironmentMonitor& TesseractPlanningServer::getEnvironmentMonitor() const
{
  return environment_;
}

void TesseractPlanningServer::onMotionPlanningCallback(const tesseract_msgs::GetMotionPlanGoalConstPtr& goal)
{
  tesseract_planning::Instruction program = tesseract_planning::fromXMLString(goal->request.instructions);
  const auto* composite_program = program.cast_const<tesseract_planning::CompositeInstruction>();

  tesseract_planning::Instruction seed = tesseract_planning::generateSkeletonSeed(*composite_program);
  if (!goal->request.seed.empty())
    seed = tesseract_planning::fromXMLString(goal->request.seed);

  tesseract_planning::GraphTaskflow::UPtr task = nullptr;
  if (goal->request.name == goal->TRAJOPT_PLANNER_NAME)
  {
    task = tesseract_planning::createTrajOptTaskflow(goal->request.seed.empty(),
                                                     simple_plan_profiles_,
                                                     simple_composite_profiles_,
                                                     trajopt_plan_profiles_,
                                                     trajopt_composite_profiles_);
  }
  else if (goal->request.name == goal->OMPL_PLANNER_NAME)
  {
    task = tesseract_planning::createOMPLTaskflow(
        goal->request.seed.empty(), simple_plan_profiles_, simple_composite_profiles_, ompl_plan_profiles_);
  }
  else if (goal->request.name == goal->DESCARTES_PLANNER_NAME)
  {
    tesseract_planning::DescartesTaskflowParams params;
    params.enable_simple_planner = goal->request.seed.empty();
    params.simple_plan_profiles = simple_plan_profiles_;
    params.simple_composite_profiles = simple_composite_profiles_;
    params.descartes_plan_profiles = descartes_plan_profiles_;
    task = tesseract_planning::createDescartesTaskflow(params);
  }
  else if (goal->request.name == goal->CARTESIAN_PLANNER_NAME)
  {
    task = tesseract_planning::createCartesianTaskflow(goal->request.seed.empty(),
                                                       simple_plan_profiles_,
                                                       simple_composite_profiles_,
                                                       descartes_plan_profiles_,
                                                       trajopt_plan_profiles_,
                                                       trajopt_composite_profiles_);
  }
  else if (goal->request.name == goal->FREESPACE_PLANNER_NAME)
  {
    task = tesseract_planning::createFreespaceTaskflow(goal->request.seed.empty(),
                                                       simple_plan_profiles_,
                                                       simple_composite_profiles_,
                                                       ompl_plan_profiles_,
                                                       trajopt_plan_profiles_,
                                                       trajopt_composite_profiles_);
  }

  tesseract_msgs::GetMotionPlanResult result;
  tesseract_planning::ManipulatorInfo mi = composite_program->getManipulatorInfo();
  std::size_t nt = std::thread::hardware_concurrency();
  if (goal->request.num_threads != 0)
    nt = goal->request.num_threads;

  if (task != nullptr)
  {
    tesseract_planning::SimpleProcessManager sm(std::move(task), nt);
    sm.init(tesseract_planning::ProcessInput(environment_.getTesseract(), &program, mi, &seed));

    result.response.successful = sm.execute();
    result.response.results = tesseract_planning::toXMLString(seed);
  }
  else
  {
    tesseract_planning::GraphTaskflow::UPtr gtask = nullptr;
    tesseract_planning::GraphTaskflow::UPtr ctask1 = nullptr;
    tesseract_planning::GraphTaskflow::UPtr ftask1 = nullptr;
    tesseract_planning::GraphTaskflow::UPtr task2 = nullptr;
    tesseract_planning::GraphTaskflow::UPtr trajopt_task = nullptr;

    task = tesseract_planning::createFreespaceTaskflow(goal->request.seed.empty(),
                                                       simple_plan_profiles_,
                                                       simple_composite_profiles_,
                                                       ompl_plan_profiles_,
                                                       trajopt_plan_profiles_,
                                                       trajopt_composite_profiles_);
    ftask1 = tesseract_planning::createFreespaceTaskflow(goal->request.seed.empty(),
                                                         simple_plan_profiles_,
                                                         simple_composite_profiles_,
                                                         ompl_plan_profiles_,
                                                         trajopt_plan_profiles_,
                                                         trajopt_composite_profiles_);
    ctask1 = tesseract_planning::createCartesianTaskflow(goal->request.seed.empty(),
                                                         simple_plan_profiles_,
                                                         simple_composite_profiles_,
                                                         descartes_plan_profiles_,
                                                         trajopt_plan_profiles_,
                                                         trajopt_composite_profiles_);

    task2 = tesseract_planning::createCartesianTaskflow(goal->request.seed.empty(),
                                                        simple_plan_profiles_,
                                                        simple_composite_profiles_,
                                                        descartes_plan_profiles_,
                                                        trajopt_plan_profiles_,
                                                        trajopt_composite_profiles_);

    tesseract_planning::DescartesTaskflowParams params;
    params.enable_simple_planner = goal->request.seed.empty();
    params.enable_post_contact_discrete_check = false;
    params.enable_post_contact_continuous_check = false;
    params.enable_time_parameterization = false;
    params.simple_plan_profiles = simple_plan_profiles_;
    params.simple_composite_profiles = simple_composite_profiles_;
    params.descartes_plan_profiles = descartes_plan_profiles_;
    gtask = tesseract_planning::createDescartesTaskflow(params);

    trajopt_task = tesseract_planning::createTrajOptTaskflow(goal->request.seed.empty(),
                                                             simple_plan_profiles_,
                                                             simple_composite_profiles_,
                                                             trajopt_plan_profiles_,
                                                             trajopt_composite_profiles_);

    if (goal->request.name == goal->RASTER_FT_PLANNER_NAME)
    {
      tesseract_planning::RasterProcessManager rm(std::move(task), std::move(ftask1), std::move(task2), nt);
      rm.init(tesseract_planning::ProcessInput(environment_.getTesseract(), &program, mi, &seed));

      result.response.successful = rm.execute();
      result.response.results = tesseract_planning::toXMLString(seed);
    }
    else if (goal->request.name == goal->RASTER_FT_DT_PLANNER_NAME)
    {
      tesseract_planning::RasterDTProcessManager rm(std::move(task), std::move(ftask1), std::move(task2), nt);
      rm.init(tesseract_planning::ProcessInput(environment_.getTesseract(), &program, mi, &seed));

      result.response.successful = rm.execute();
      result.response.results = tesseract_planning::toXMLString(seed);
    }
    else if (goal->request.name == goal->RASTER_FT_WAAD_PLANNER_NAME)
    {
      tesseract_planning::RasterWAADProcessManager rm(std::move(task), std::move(ftask1), std::move(task2), nt);
      rm.init(tesseract_planning::ProcessInput(environment_.getTesseract(), &program, mi, &seed));

      result.response.successful = rm.execute();
      result.response.results = tesseract_planning::toXMLString(seed);
    }
    else if (goal->request.name == goal->RASTER_FT_WAAD_DT_PLANNER_NAME)
    {
      tesseract_planning::RasterWAADDTProcessManager rm(std::move(task), std::move(ftask1), std::move(task2), nt);
      rm.init(tesseract_planning::ProcessInput(environment_.getTesseract(), &program, mi, &seed));

      result.response.successful = rm.execute();
      result.response.results = tesseract_planning::toXMLString(seed);
    }
    else if (goal->request.name == goal->RASTER_CT_PLANNER_NAME)
    {
      tesseract_planning::RasterProcessManager rm(std::move(task), std::move(ctask1), std::move(task2), nt);
      rm.init(tesseract_planning::ProcessInput(environment_.getTesseract(), &program, mi, &seed));

      result.response.successful = rm.execute();
      result.response.results = tesseract_planning::toXMLString(seed);
    }
    else if (goal->request.name == goal->RASTER_CT_DT_PLANNER_NAME)
    {
      tesseract_planning::RasterDTProcessManager rm(std::move(task), std::move(ctask1), std::move(task2), nt);
      rm.init(tesseract_planning::ProcessInput(environment_.getTesseract(), &program, mi, &seed));

      result.response.successful = rm.execute();
      result.response.results = tesseract_planning::toXMLString(seed);
    }
    else if (goal->request.name == goal->RASTER_CT_WAAD_PLANNER_NAME)
    {
      tesseract_planning::RasterWAADProcessManager rm(std::move(task), std::move(ctask1), std::move(task2), nt);
      rm.init(tesseract_planning::ProcessInput(environment_.getTesseract(), &program, mi, &seed));

      result.response.successful = rm.execute();
      result.response.results = tesseract_planning::toXMLString(seed);
    }
    else if (goal->request.name == goal->RASTER_CT_WAAD_DT_PLANNER_NAME)
    {
      tesseract_planning::RasterWAADDTProcessManager rm(std::move(task), std::move(ctask1), std::move(task2), nt);
      rm.init(tesseract_planning::ProcessInput(environment_.getTesseract(), &program, mi, &seed));

      result.response.successful = rm.execute();
      result.response.results = tesseract_planning::toXMLString(seed);
    }
    else if (goal->request.name == goal->RASTER_G_FT_PLANNER_NAME)
    {
      tesseract_planning::RasterProcessManager rm(
          std::move(gtask), std::move(task), std::move(ftask1), std::move(trajopt_task), nt);
      rm.init(tesseract_planning::ProcessInput(environment_.getTesseract(), &program, mi, &seed));

      result.response.successful = rm.execute();
      result.response.results = tesseract_planning::toXMLString(seed);
    }
    else if (goal->request.name == goal->RASTER_G_CT_PLANNER_NAME)
    {
      tesseract_planning::RasterProcessManager rm(
          std::move(gtask), std::move(task), std::move(ctask1), std::move(trajopt_task), nt);
      rm.init(tesseract_planning::ProcessInput(environment_.getTesseract(), &program, mi, &seed));

      result.response.successful = rm.execute();
      result.response.results = tesseract_planning::toXMLString(seed);
    }
    else
    {
      result.response.successful = false;
      ROS_ERROR("Requested motion planner is not supported!");
    }
  }

  motion_plan_server_.setSucceeded(result);
}

void TesseractPlanningServer::loadDefaultPlannerProfiles()
{
  trajopt_plan_profiles_["DEFAULT"] = std::make_shared<tesseract_planning::TrajOptDefaultPlanProfile>();
  trajopt_composite_profiles_["DEFAULT"] = std::make_shared<tesseract_planning::TrajOptDefaultCompositeProfile>();
  descartes_plan_profiles_["DEFAULT"] = std::make_shared<tesseract_planning::DescartesDefaultPlanProfile<double>>();
  ompl_plan_profiles_["DEFAULT"] = std::make_shared<tesseract_planning::OMPLDefaultPlanProfile>();
  simple_plan_profiles_["DEFAULT"] = std::make_shared<tesseract_planning::SimplePlannerDefaultPlanProfile>();
}

}  // namespace tesseract_planning_server
