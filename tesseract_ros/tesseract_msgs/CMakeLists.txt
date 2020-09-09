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
    ContactResult.msg
    ContactResultVector.msg
    EnvironmentCommand.msg
    Geometry.msg
    GroupsJointState.msg
    GroupsJointStates.msg
    GroupsOPWKinematics.msg
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
    KinematicsInformation.msg
    Link.msg
    LinkGroup.msg
    Material.msg
    Mesh.msg
    ObjectColor.msg
    OctreeSubType.msg
    PlannerProfileRemapping.msg
    PlanningRequest.msg
    PlanningResponse.msg
    ProcessPlan.msg
    ProcessPlanPath.msg
    ProcessPlanSegment.msg
    ProcessPlanTransitionPair.msg
    ProfileMap.msg
    REPGroup.msg
    ROPGroup.msg
    SceneGraph.msg
    StringDoublePair.msg
    StringPair.msg
    TesseractState.msg
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
