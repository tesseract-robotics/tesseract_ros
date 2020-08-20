#include <tesseract_common/macros.h>
TESSERACT_COMMON_IGNORE_WARNINGS_PUSH
#include <gtest/gtest.h>
#include <algorithm>
#include <memory>
TESSERACT_COMMON_IGNORE_WARNINGS_POP

#include <tesseract_rosutils/conversions.h>
#include <tesseract_rosutils/plotting.h>
#include <tesseract_rosutils/utils.h>
#include <tesseract/tesseract.h>
#include <tesseract_scene_graph/resource_locator.h>

#include <tesseract_msgs/TesseractState.h>

using namespace tesseract;
using namespace tesseract_environment;
using namespace tesseract_scene_graph;
using namespace tesseract_rosutils;

class TesseractROSUtilsUnit : public ::testing::Test
{
protected:
  Tesseract::Ptr tesseract_ptr_;

  void SetUp() override
  {
    tesseract_scene_graph::ResourceLocator::Ptr locator = std::make_shared<ROSResourceLocator>();
    Tesseract::Ptr tesseract = std::make_shared<Tesseract>();
    boost::filesystem::path urdf_path(std::string(TESSERACT_SUPPORT_DIR) + "/urdf/abb_irb2400.urdf");
    boost::filesystem::path srdf_path(std::string(TESSERACT_SUPPORT_DIR) + "/urdf/abb_irb2400.srdf");
    EXPECT_TRUE(tesseract->init(urdf_path, srdf_path, locator));
    tesseract_ptr_ = tesseract;
  }
};

TEST_F(TesseractROSUtilsUnit, Instantiation)  // NOLINT
{
  using namespace tesseract_rosutils;
}

TEST_F(TesseractROSUtilsUnit, processTesseractStateMsg)  // NOLINT
{
  ros::Time::init();

  tesseract_msgs::TesseractState tesseract_state_msg;
  Environment::Ptr env = tesseract_ptr_->getEnvironment();

  toMsg(tesseract_state_msg, *env);

  EXPECT_EQ(tesseract_state_msg.id, env->getName());
  EXPECT_EQ(tesseract_state_msg.revision, env->getRevision());
  EXPECT_EQ(tesseract_state_msg.commands.size(), env->getCommandHistory().size());

  const std::string link_name1 = "link_n1";
  const std::string joint_name1 = "joint_n1";
  auto link_1 = std::make_shared<Link>(link_name1);

  auto joint_1 = std::make_shared<Joint>(joint_name1);
  joint_1->parent_to_joint_origin_transform.translation()(0) = 1.25;
  joint_1->parent_link_name = "base_link";
  joint_1->child_link_name = link_name1;
  joint_1->type = JointType::FIXED;
  AddCommand cmd(link_1, joint_1);

  tesseract_msgs::EnvironmentCommand cmd_msg;
  toMsg(cmd_msg, cmd);

  tesseract_state_msg.commands.push_back(cmd_msg);
  tesseract_state_msg.revision += 1;

  EXPECT_TRUE(processMsg(*env, tesseract_state_msg));
  EXPECT_EQ(tesseract_state_msg.id, env->getName());
  EXPECT_EQ(tesseract_state_msg.revision, env->getRevision());
  EXPECT_EQ(tesseract_state_msg.commands.size(), env->getCommandHistory().size());
}

int main(int argc, char** argv)
{
  testing::InitGoogleTest(&argc, argv);

  return RUN_ALL_TESTS();
}
