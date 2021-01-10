/**
 * @file constants.h
 * @brief Constants used throughout tesseract monitoring
 *
 * @author Levi Armstrong
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
#ifndef TESSERACT_MONITORING_CONSTANTS_H
#define TESSERACT_MONITORING_CONSTANTS_H

namespace tesseract_monitoring
{
/// The name of the topic used by default for receiving joint states
static const std::string DEFAULT_JOINT_STATES_TOPIC = "joint_states";

/// The name of the service used by default for requesting tesseract environment change history
static const std::string DEFAULT_GET_ENVIRONMENT_CHANGES_SERVICE = "/get_tesseract_changes";

/// The name of the service used by default for requesting tesseract environment information
static const std::string DEFAULT_GET_ENVIRONMENT_INFORMATION_SERVICE = "/get_tesseract_information";

/// The name of the service used by default for setting the full tesseract environment state
static const std::string DEFAULT_MODIFY_ENVIRONMENT_SERVICE = "/modify_tesseract";

/// The name of the service used by default for saving the scene graph as a DOT
static const std::string DEFAULT_SAVE_SCENE_GRAPH_SERVICE = "/save_scene_graph";

/// The name of the topic used by default for publishing the monitored tesseract environment;
static const std::string DEFAULT_PUBLISH_ENVIRONMENT_TOPIC = "/tesseract_published_environment";

/// The name of the topic used by default for publishing the monitored contact results;
static const std::string DEFAULT_PUBLISH_CONTACT_RESULTS_TOPIC = "/contact_results";

/// The name of the topic used by default for publishing the monitored contact results markers;
static const std::string DEFAULT_PUBLISH_CONTACT_MARKER_TOPIC = "/contact_results_markers";

/// The name of the service used by default for computing the contact results
static const std::string DEFAULT_COMPUTE_CONTACT_RESULTS_SERVICE = "/compute_contact_results";

}  // namespace tesseract_monitoring
#endif  // TESSERACT_MONITORING_CONSTANTS_H
