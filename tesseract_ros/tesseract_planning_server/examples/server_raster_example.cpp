#include <tesseract_common/macros.h>
TESSERACT_COMMON_IGNORE_WARNINGS_PUSH
#include <ros/ros.h>
TESSERACT_COMMON_IGNORE_WARNINGS_POP

#include <tesseract_environment/core/environment.h>
#include <tesseract_planning_server/tesseract_planning_server.h>

using namespace tesseract_environment;
using namespace tesseract_planning;

int main(int argc, char** argv)
{
  ros::init(argc, argv, "server_raster_example");
  ros::NodeHandle pnh("~");

  //  Instruction program = rasterExampleProgram();

  ros::AsyncSpinner spinner(4);
  spinner.start();

  ros::waitForShutdown();

  return 0;
}
