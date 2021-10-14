#include <tesseract_common/macros.h>
TESSERACT_COMMON_IGNORE_WARNINGS_PUSH
#include <ros/ros.h>
#include <gtest/gtest.h>
TESSERACT_COMMON_IGNORE_WARNINGS_POP

#include <tesseract_ros_examples/glass_upright_example.h>

using namespace tesseract_ros_examples;

TEST(TesseractROSExamples, GlassUprightTrajOptExampleUnit)  // NOLINT
{
  ros::NodeHandle nh;
  GlassUprightExample example(nh, false, false, false, false, false);
  EXPECT_TRUE(example.run());
}

TEST(TesseractROSExamples, GlassUprightTrajOptIfoptExampleUnit)  // NOLINT
{
  ros::NodeHandle nh;
  GlassUprightExample example(nh, false, false, false, true, false);
  EXPECT_TRUE(example.run());
}

int main(int argc, char** argv)
{
  testing::InitGoogleTest(&argc, argv);
  ros::init(argc, argv, "glass_upright_example_unit");

  return RUN_ALL_TESTS();
}
