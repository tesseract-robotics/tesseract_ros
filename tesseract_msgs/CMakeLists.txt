cmake_minimum_required(VERSION 3.5.0)
project(tesseract_msgs)

if(CMAKE_CXX_COMPILER_ID STREQUAL "Clang")
  add_compile_options(
    -std=c++14
    -Wall
    -Wextra
    -Winconsistent-missing-override
    -Wconversion
    -Wsign-conversion)
else()
  add_compile_options(
    -std=c++14
    -Wall
    -Wextra
    -Wsuggest-override
    -Wconversion
    -Wsign-conversion)
endif()

find_package(catkin REQUIRED COMPONENTS
  message_generation
  std_msgs
  sensor_msgs
  geometry_msgs
  shape_msgs
  octomap_msgs
  trajectory_msgs
  actionlib_msgs
)

add_message_files(
  DIRECTORY msg
  FILES
    AllowedCollisionEntry.msg
    ChainGroup.msg
    CollisionGeometry.msg
    CollisionMarginData.msg
    CollisionMarginOverrideType.msg
    ContactManagersPluginInfo.msg
    ContactMarginPair.msg
    ContactResult.msg
    ContactResultVector.msg
    Environment.msg
    EnvironmentCommand.msg
    EnvironmentState.msg
    Geometry.msg
    GroupsJointState.msg
    GroupsJointStates.msg
    GroupsKinematicPlugins.msg
    GroupsTCP.msg
    GroupsTCPs.msg
    Inertial.msg
    Joint.msg
    JointCalibration.msg
    JointDynamics.msg
    JointGroup.msg
    JointLimits.msg
    JointMimic.msg
    JointSafety.msg
    JointState.msg
    KinematicsInformation.msg
    KinematicsPluginInfo.msg
    Link.msg
    LinkGroup.msg
    Material.msg
    Mesh.msg
    ObjectColor.msg
    OctreeSubType.msg
    PlannerProfileRemapping.msg
    PlanningRequest.msg
    PlanningRequestArchive.msg
    PlanningResponse.msg
    PlanningResponseArchive.msg
    PluginInfo.msg
    PluginInfoContainer.msg
    ProfileMap.msg
    SceneGraph.msg
    StringDoublePair.msg
    StringLimitsPair.msg
    StringPair.msg
    StringPluginInfoPair.msg
    TaskInfo.msg
    Trajectory.msg
    TransformMap.msg
    VisualGeometry.msg
)

add_service_files(
  DIRECTORY srv
  FILES
    ComputeContactResultVector.srv
    GetEnvironmentChanges.srv
    GetEnvironmentInformation.srv
    ModifyEnvironment.srv
    SaveSceneGraph.srv
)

add_action_files(
  DIRECTORY action
  FILES GetMotionPlan.action
)

generate_messages(
  DEPENDENCIES
    std_msgs
    sensor_msgs
    geometry_msgs
    shape_msgs
    octomap_msgs
    trajectory_msgs
    actionlib_msgs
)

catkin_package(
  CATKIN_DEPENDS
    message_runtime
    std_msgs
    sensor_msgs
    geometry_msgs
    shape_msgs
    octomap_msgs
    trajectory_msgs
    actionlib_msgs
)
