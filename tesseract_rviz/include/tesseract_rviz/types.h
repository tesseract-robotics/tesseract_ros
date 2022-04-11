#ifndef TESSERACT_RVIZ_TYPES_H
#define TESSERACT_RVIZ_TYPES_H

#include <rviz/ogre_helpers/point_cloud.h>
#include <tesseract_geometry/impl/octree.h>

namespace tesseract_rviz
{
enum OctreeVoxelRenderMode
{
  OCTOMAP_FREE_VOXELS = 1,
  OCTOMAP_OCCUPIED_VOXELS = 2
};

enum OctreeVoxelColorMode
{
  OCTOMAP_Z_AXIS_COLOR,
  OCTOMAP_PROBABLILTY_COLOR,
};

struct OctreeDataContainer
{
  rviz::PointCloud* point_cloud;
  std::vector<rviz::PointCloud::Point> points;
  float size;
  tesseract_geometry::Octree::SubType shape_type;
};
}  // namespace tesseract_rviz
#endif  // TESSERACT_RVIZ_TYPES_H
