#include <tesseract_common/macros.h>
TESSERACT_COMMON_IGNORE_WARNINGS_PUSH
#include <ros/ros.h>
#include <tesseract_msgs/GetMotionPlanAction.h>
TESSERACT_COMMON_IGNORE_WARNINGS_POP

#include <tesseract_environment/environment.h>
#include <tesseract_planning_server/tesseract_planning_server.h>

using namespace tesseract_environment;
using namespace tesseract_planning;

int main(int argc, char** argv)
{
  ros::init(argc, argv, "server_raster_example");
  ros::NodeHandle pnh("~");

  // Create Process Planning Request

  //  ProcessPlanningRequest request;
  //  request.name = tesseract_planning::process_planner_names::FREESPACE_PLANNER_NAME;
  //  request.instructions = Instruction(program);

  // Print Diagnostics
  // request.instructions.print("Program: ");

  //  Instruction program = rasterExampleProgram();

  ros::AsyncSpinner spinner(4);
  spinner.start();

  ros::waitForShutdown();

  return 0;
}
