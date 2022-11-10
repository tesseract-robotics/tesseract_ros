/**
 * @file tesseract_geometry_serialization_unit.cpp
 * @brief Tests serialization of geometry
 *
 * @author Levi Armstrong
 * @author Matthew Powelson
 * @date March 16, 2022
 * @version TODO
 * @bug No known bugs
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
#include <gtest/gtest.h>
TESSERACT_COMMON_IGNORE_WARNINGS_POP

#include <tesseract_common/unit_test_utils.h>
#include <tesseract_common/utils.h>
#include <tesseract_geometry/geometries.h>
#include <tesseract_geometry/mesh_parser.h>
#include <tesseract_support/tesseract_support_resource_locator.h>
#include <tesseract_rosutils/utils.h>

using namespace tesseract_geometry;
using namespace tesseract_rosutils;

/**
 * @brief Tests if the toMsg and fromMsg result in the same object
 * @param geometry Input msg
 */
inline void testToMsgFromMsg(const tesseract_geometry::Geometry& object)
{
  // Serialize to ros message
  tesseract_msgs::Geometry msg;
  EXPECT_TRUE(toMsg(msg, object));

  // Deserialize to object
  tesseract_geometry::Geometry::Ptr object_new;
  EXPECT_TRUE(fromMsg(object_new, msg));

  // Check for equality
  EXPECT_TRUE(object == *object_new);
}

TEST(TesseractRosutilsGeometryMsgConversions, Box)  // NOLINT
{
  auto object = std::make_shared<Box>(1, 2, 3);
  testToMsgFromMsg(*object);
}

TEST(TesseractRosutilsGeometryMsgConversions, Capsule)  // NOLINT
{
  auto object = std::make_shared<Capsule>(1, 2);
  testToMsgFromMsg(*object);
}

TEST(TesseractRosutilsGeometryMsgConversions, Cone)  // NOLINT
{
  auto object = std::make_shared<Cone>(1.1, 2.2);
  testToMsgFromMsg(*object);
}

TEST(TesseractRosutilsGeometryMsgConversions, ConvexMesh)  // NOLINT
{
  std::string path = std::string(TESSERACT_SUPPORT_DIR) + "/meshes/sphere_p25m.stl";
  tesseract_common::TesseractSupportResourceLocator locator;
  auto object = tesseract_geometry::createMeshFromResource<tesseract_geometry::ConvexMesh>(
      locator.locateResource(path), Eigen::Vector3d(.1, .2, .3), true, true, true, true, true);
  testToMsgFromMsg(*object.back());
}

TEST(TesseractRosutilsGeometryMsgConversions, Cylinder)  // NOLINT
{
  auto object = std::make_shared<Cylinder>(3.3, 4.4);
  testToMsgFromMsg(*object);
}

TEST(TesseractRosutilsGeometryMsgConversions, Mesh)  // NOLINT
{
  std::string path = std::string(TESSERACT_SUPPORT_DIR) + "/meshes/sphere_p25m.stl";
  tesseract_common::TesseractSupportResourceLocator locator;
  auto object = tesseract_geometry::createMeshFromResource<tesseract_geometry::Mesh>(
      locator.locateResource(path), Eigen::Vector3d(.1, .2, .3), true, true, true, true, true);
  testToMsgFromMsg(*object.back());
}

TEST(TesseractRosutilsGeometryMsgConversions, Octree)  // NOLINT
{
  struct TestPointCloud
  {
    struct point
    {
      point(double x, double y, double z) : x(x), y(y), z(z) {}
      double x;
      double y;
      double z;
    };

    std::vector<point> points;
  };

  TestPointCloud pc;
  pc.points.emplace_back(.5, 0.5, 0.5);
  pc.points.emplace_back(-.5, -0.5, -0.5);
  pc.points.emplace_back(-.5, 0.5, 0.5);
  {
    auto object =
        std::make_shared<tesseract_geometry::Octree>(pc, 1, tesseract_geometry::Octree::SubType::BOX, false, true);
    testToMsgFromMsg(*object);
  }
  {
    auto object =
        std::make_shared<tesseract_geometry::Octree>(pc, 1, tesseract_geometry::Octree::SubType::BOX, false, false);
    testToMsgFromMsg(*object);
  }
}

TEST(TesseractRosutilsGeometryMsgConversions, Plane)  // NOLINT
{
  auto object = std::make_shared<Plane>(1.1, 2, 3.3, 4);
  testToMsgFromMsg(*object);
}

TEST(TesseractRosutilsGeometryMsgConversions, PolygonMesh)  // NOLINT
{
  std::string path = std::string(TESSERACT_SUPPORT_DIR) + "/meshes/sphere_p25m.stl";
  tesseract_common::TesseractSupportResourceLocator locator;
  auto object = tesseract_geometry::createMeshFromResource<tesseract_geometry::PolygonMesh>(
      locator.locateResource(path), Eigen::Vector3d(.1, .2, .3), true, true, true, true, true);
  testToMsgFromMsg(*object.back());
}

TEST(TesseractRosutilsGeometryMsgConversions, SDFMesh)  // NOLINT
{
  std::string path = std::string(TESSERACT_SUPPORT_DIR) + "/meshes/sphere_p25m.stl";
  tesseract_common::TesseractSupportResourceLocator locator;
  auto object = tesseract_geometry::createMeshFromResource<tesseract_geometry::SDFMesh>(
      locator.locateResource(path), Eigen::Vector3d(.1, .2, .3), true, true, true, true, true);
  testToMsgFromMsg(*object.back());
}

TEST(TesseractRosutilsGeometryMsgConversions, Sphere)  // NOLINT
{
  auto object = std::make_shared<Sphere>(3.3);
  testToMsgFromMsg(*object);
}

int main(int argc, char** argv)
{
  testing::InitGoogleTest(&argc, argv);

  return RUN_ALL_TESTS();
}
