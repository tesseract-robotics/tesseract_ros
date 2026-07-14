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
#include <tesseract/common/macros.h>
TESSERACT_COMMON_IGNORE_WARNINGS_PUSH
#include <gtest/gtest.h>
TESSERACT_COMMON_IGNORE_WARNINGS_POP

#include <tesseract/common/resource_locator.h>
#include <tesseract/common/unit_test_utils.h>
#include <tesseract/common/utils.h>
#include <tesseract/geometry/geometries.h>
#include <tesseract/geometry/impl/octree_utils.h>
#include <tesseract/geometry/impl/signed_distance_field_utils.h>
#include <tesseract/geometry/mesh_parser.h>

#include <tesseract_msgs/Geometry.h>
#include <tesseract_rosutils/utils.h>

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <stdexcept>
#include <vector>

using namespace tesseract::geometry;
using namespace tesseract_rosutils;

/**
 * @brief Tests if the toMsg and fromMsg result in the same object
 * @param geometry Input msg
 */
inline void testToMsgFromMsg(const tesseract::geometry::Geometry& object)
{
  // Serialize to ros message
  tesseract_msgs::Geometry msg;
  EXPECT_TRUE(toMsg(msg, object));

  // Deserialize to object
  tesseract::geometry::Geometry::Ptr object_new;
  EXPECT_TRUE(fromMsg(object_new, msg));

  // Check for equality
  EXPECT_TRUE(object == *object_new);
}

/**
 * @brief Verify a SIGNED_DISTANCE_FIELD message deserializes back to the expected field data.
 * @details Geometry::operator== only compares type/uuid (and the ROS 1 message does not carry the
 * uuid), so the payload is checked field-by-field. Grids serialize as a float32 VDB/NanoVDB
 * FloatGrid, so distances survive only to float precision, not bit-exactly.
 */
inline void expectSdfMsgRoundTrip(const tesseract::geometry::SignedDistanceField& expected,
                                  const tesseract_msgs::Geometry& msg)
{
  tesseract::geometry::Geometry::Ptr geom_new;
  EXPECT_TRUE(fromMsg(geom_new, msg));
  ASSERT_NE(geom_new, nullptr);
  ASSERT_EQ(geom_new->getType(), tesseract::geometry::GeometryType::SIGNED_DISTANCE_FIELD);

  const auto& sdf = static_cast<const tesseract::geometry::SignedDistanceField&>(*geom_new);
  EXPECT_EQ(sdf.getDimensions(), expected.getDimensions());
  EXPECT_TRUE(sdf.getDomain().min().isApprox(expected.getDomain().min(), 1e-5));
  EXPECT_TRUE(sdf.getDomain().max().isApprox(expected.getDomain().max(), 1e-5));
  EXPECT_TRUE(sdf.getScale().isApprox(expected.getScale(), 1e-5));
  ASSERT_EQ(sdf.getDistances().size(), expected.getDistances().size());
  for (std::size_t i = 0; i < expected.getDistances().size(); ++i)
    EXPECT_NEAR(sdf.getDistances()[i], expected.getDistances()[i], 1e-6);
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
  tesseract::common::GeneralResourceLocator locator;
  std::string path = locator.locateResource("package://tesseract/support/meshes/sphere_p25m.stl")->getFilePath();
  auto object = tesseract::geometry::createMeshFromResource<tesseract::geometry::ConvexMesh>(
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
  tesseract::common::GeneralResourceLocator locator;
  std::string path = locator.locateResource("package://tesseract/support/meshes/sphere_p25m.stl")->getFilePath();
  auto object = tesseract::geometry::createMeshFromResource<tesseract::geometry::Mesh>(
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
    std::unique_ptr<octomap::OcTree> ot = createOctree(pc, 1, false, true);
    auto object = std::make_shared<tesseract::geometry::Octree>(
        std::move(ot), tesseract::geometry::OctreeSubType::BOX, false, true);
    testToMsgFromMsg(*object);
  }
  {
    std::unique_ptr<octomap::OcTree> ot = createOctree(pc, 1, false, true);
    auto object = std::make_shared<tesseract::geometry::Octree>(
        std::move(ot), tesseract::geometry::OctreeSubType::BOX, false, false);
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
  tesseract::common::GeneralResourceLocator locator;
  std::string path = locator.locateResource("package://tesseract/support/meshes/sphere_p25m.stl")->getFilePath();
  auto object = tesseract::geometry::createMeshFromResource<tesseract::geometry::PolygonMesh>(
      locator.locateResource(path), Eigen::Vector3d(.1, .2, .3), true, true, true, true, true);
  testToMsgFromMsg(*object.back());
}

TEST(TesseractRosutilsGeometryMsgConversions, SignedDistanceFieldVDB)  // NOLINT
{
  // Analytic sphere SDF; a non-default scale exercises that (blob-external) field. toMsg emits the
  // OpenVDB payload (never NanoVDB), so this round-trip covers the vdb branch of fromMsg.
  const tesseract::geometry::SignedDistanceFunction sphere = [](const Eigen::Vector3d& p) { return p.norm() - 0.5; };
  const Eigen::Vector3d scale(1, 2, 3);
  auto object = tesseract::geometry::createDiscreteSignedDistanceField(
      sphere, Eigen::Vector3d(-1, -1, -1), Eigen::Vector3d(1, 1, 1), Eigen::Vector3i(5, 5, 5), scale);

  tesseract_msgs::Geometry msg;
  EXPECT_TRUE(toMsg(msg, *object));
  EXPECT_FALSE(msg.signed_distance_field.vdb.empty());
  EXPECT_TRUE(msg.signed_distance_field.nvdb.empty());

  expectSdfMsgRoundTrip(*object, msg);
}

TEST(TesseractRosutilsGeometryMsgConversions, SignedDistanceFieldNVDB)  // NOLINT
{
  // toMsg only ever emits the vdb field. Serialize normally (which also populates the uuid that
  // fromMsg requires), then swap the payload for a NanoVDB one to cover the nvdb branch of fromMsg.
  const tesseract::geometry::SignedDistanceFunction sphere = [](const Eigen::Vector3d& p) { return p.norm() - 0.5; };
  const Eigen::Vector3d scale(1, 2, 3);
  auto object = tesseract::geometry::createDiscreteSignedDistanceField(
      sphere, Eigen::Vector3d(-1, -1, -1), Eigen::Vector3d(1, 1, 1), Eigen::Vector3i(5, 5, 5), scale);

  tesseract_msgs::Geometry msg;
  EXPECT_TRUE(toMsg(msg, *object));
  msg.signed_distance_field.vdb.clear();
  msg.signed_distance_field.nvdb = tesseract::geometry::writeSignedDistanceFieldNVDB(*object);
  ASSERT_TRUE(msg.signed_distance_field.vdb.empty());
  ASSERT_FALSE(msg.signed_distance_field.nvdb.empty());

  expectSdfMsgRoundTrip(*object, msg);
}

TEST(TesseractRosutilsGeometryMsgConversions, SignedDistanceFieldMissingPayloadThrows)  // NOLINT
{
  // A field message with neither payload set must be rejected, not silently produce a null shape.
  tesseract_msgs::Geometry msg;
  msg.type = tesseract_msgs::Geometry::SIGNED_DISTANCE_FIELD;
  ASSERT_TRUE(msg.signed_distance_field.vdb.empty());
  ASSERT_TRUE(msg.signed_distance_field.nvdb.empty());

  tesseract::geometry::Geometry::Ptr geom_new;
  EXPECT_THROW(fromMsg(geom_new, msg), std::runtime_error);  // NOLINT
}

TEST(TesseractRosutilsGeometryMsgConversions, SignedDistanceFieldBothPayloadsThrows)  // NOLINT
{
  // Exactly one payload is expected; a message carrying both is an ambiguous wire contract and
  // must be rejected rather than silently preferring one.
  const tesseract::geometry::SignedDistanceFunction sphere = [](const Eigen::Vector3d& p) { return p.norm() - 0.5; };
  auto object = tesseract::geometry::createDiscreteSignedDistanceField(
      sphere, Eigen::Vector3d(-1, -1, -1), Eigen::Vector3d(1, 1, 1), Eigen::Vector3i(5, 5, 5), Eigen::Vector3d(1, 2, 3));

  tesseract_msgs::Geometry msg;
  EXPECT_TRUE(toMsg(msg, *object));  // populates the vdb payload
  msg.signed_distance_field.nvdb = tesseract::geometry::writeSignedDistanceFieldNVDB(*object);
  ASSERT_FALSE(msg.signed_distance_field.vdb.empty());
  ASSERT_FALSE(msg.signed_distance_field.nvdb.empty());

  tesseract::geometry::Geometry::Ptr geom_new;
  EXPECT_THROW(fromMsg(geom_new, msg), std::runtime_error);  // NOLINT
}

TEST(TesseractRosutilsGeometryMsgConversions, SdfSurfaceShellPoints)  // NOLINT
{
  const SignedDistanceFunction sphere = [](const Eigen::Vector3d& p) { return p.norm() - 0.9; };
  // 41^3 over [-1, 1] -> 0.05 m voxels; band = 1 voxel.
  auto sdf = createDiscreteSignedDistanceField(
      sphere, Eigen::Vector3d(-1, -1, -1), Eigen::Vector3d(1, 1, 1), Eigen::Vector3i(41, 41, 41));
  const double band = 0.05;

  const std::vector<Eigen::Vector3d> shell = sdfSurfaceShellPoints(*sdf, band);

  // Non-empty, and every returned sample is within the band of the surface.
  ASSERT_FALSE(shell.empty());
  for (const Eigen::Vector3d& p : shell)
    EXPECT_LE(std::abs(sdf->getDistance(p)), band + 1e-6);

  // It is a SHELL, not the solid interior: the naive render drew every d <= 0 sample; assert
  // the shell is strictly fewer, and that a genuine deep interior (d < -band) exists + is dropped.
  const std::vector<double>& d = sdf->getDistances();
  const auto interior = static_cast<std::size_t>(std::count_if(d.begin(), d.end(), [](double v) { return v <= 0.0; }));
  const auto deep = static_cast<std::size_t>(std::count_if(d.begin(), d.end(), [&](double v) { return v < -band; }));
  ASSERT_GT(deep, 0u);
  EXPECT_LT(shell.size(), interior);
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
