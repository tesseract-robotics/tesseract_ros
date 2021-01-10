#include <tesseract_common/macros.h>
TESSERACT_COMMON_IGNORE_WARNINGS_PUSH
#include <ros/ros.h>
#include <memory>
#include <actionlib/server/simple_action_server.h>
#include <tesseract_msgs/GetMotionPlanAction.h>
#include <taskflow/taskflow.hpp>
#include <tesseract_common/utils.h>
TESSERACT_COMMON_IGNORE_WARNINGS_POP

// add_executable(${PROJECT_NAME}_action_server_memory examples/action_server_memory.cpp)
// target_link_libraries(${PROJECT_NAME}_action_server_memory PRIVATE tesseract::tesseract_command_language
// tesseract::tesseract_process_managers ${catkin_LIBRARIES})
// target_compile_options(${PROJECT_NAME}_action_server_memory PRIVATE ${TESSERACT_COMPILE_OPTIONS})
// target_clang_tidy(${PROJECT_NAME}_action_server_memory ARGUMENTS
// ${TESSERACT_CLANG_TIDY_ARGS} ENABLE ${TESSERACT_ENABLE_CLANG_TIDY})
// target_cxx_version(${PROJECT_NAME}_action_server_memory PRIVATE VERSION 17)
// target_include_directories(${PROJECT_NAME}_action_server_memory PUBLIC
//    "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
//    "$<INSTALL_INTERFACE:include>")
// target_include_directories(${PROJECT_NAME}_action_server_memory SYSTEM PUBLIC
//    ${catkin_INCLUDE_DIRS})

const std::string action_name = "action_test";
static std::shared_ptr<tf::Executor> executor;

void callbackTest(const tesseract_msgs::GetMotionPlanGoalConstPtr& goal)
{
  tf::Taskflow taskflow;

  for (std::size_t i = 0; i < 10; ++i)
  {
    tf::Taskflow taskflow;
    taskflow.emplace([]() {
      std::vector<double> t;
      t.reserve(1000000000);
      for (std::size_t j = 0; j < 1000000000; j++)
      {
        std::uniform_real_distribution<double> sample{ 0, 10 };
        t.push_back(sample(tesseract_common::mersenne));
      }
    });
    std::future<void> f = executor->run(taskflow);
    f.wait();
  }
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "action_server_memory");
  ros::NodeHandle nh;
  ros::NodeHandle pnh("~");

  executor = std::make_shared<tf::Executor>();
  actionlib::SimpleActionServer<tesseract_msgs::GetMotionPlanAction> action_temp(
      nh, action_name, boost::bind(&callbackTest, _1), true);

  ros::AsyncSpinner spinner(4);
  spinner.start();

  ros::waitForShutdown();

  return 0;
}
