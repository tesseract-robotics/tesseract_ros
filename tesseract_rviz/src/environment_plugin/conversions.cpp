#include <tesseract_rviz/environment_plugin/conversions.h>

#include <OgreEntity.h>
#include <OgreManualObject.h>
#include <OgreMaterialManager.h>
#include <OgreMesh.h>
#include <OgreRibbonTrail.h>
#include <OgreSceneManager.h>
#include <OgreSceneNode.h>
#include <OgreSharedPtr.h>
#include <OgreSubEntity.h>
#include <OgreTechnique.h>
#include <OgreTextureManager.h>
#include <OgreNameGenerator.h>
#include <OgreVector3.h>

#include <resource_retriever/retriever.h>
#include <octomap/octomap.h>
#include <octomap_msgs/Octomap.h>
#include <ros/console.h>

#include <rviz/load_resource.h>
#include <rviz/mesh_loader.h>
#include <rviz/ogre_helpers/axes.h>
#include <rviz/ogre_helpers/mesh_shape.h>
#include <rviz/ogre_helpers/object.h>
#include <rviz/ogre_helpers/shape.h>
#include <rviz/ogre_helpers/point_cloud.h>

#include <tesseract_geometry/geometries.h>

namespace tesseract_rviz
{
static Ogre::NameGenerator material_name_generator("Tesseract_Material");

bool isMeshWithColor(const std::string& file_path)
{
  if (file_path.length() >= 4)
  {
    std::string last_four = file_path.substr(file_path.length() - 4);
    std::string last_four_lower;
    last_four_lower.resize(4);
    std::transform(last_four.begin(), last_four.end(), last_four_lower.begin(), ::tolower);
    return (last_four_lower == ".dae") || (last_four_lower == ".obj");
  }

  return false;
}

Ogre::Entity* createEntityForMeshData(Ogre::SceneManager& scene,
                                      tesseract_gui::EntityContainer& entity_container,
                                      const std::shared_ptr<const tesseract_common::VectorVector3d>& mesh_vertices,
                                      const std::shared_ptr<const Eigen::VectorXi>& mesh_faces)
{
  Ogre::ManualObject* object = new Ogre::ManualObject("the one and only");
  object->begin("BaseWhiteNoLighting", Ogre::RenderOperation::OT_TRIANGLE_LIST);

  unsigned int vertexCount = 0;
  Ogre::Vector3 normal(0.0, 0.0, 0.0);

  for (long t = 0; t < mesh_faces->size(); ++t)
  {
    if (vertexCount >= 2004)
    {
      // Subdivide large meshes into submeshes with at most 2004
      // vertices to prevent problems on some graphics cards.
      object->end();
      object->begin("BaseWhiteNoLighting", Ogre::RenderOperation::OT_TRIANGLE_LIST);
      vertexCount = 0;
    }

    size_t num_verts = static_cast<size_t>((*mesh_faces)[t]);
    assert(num_verts >= 3);

    std::vector<Ogre::Vector3> vertices(num_verts);
    std::vector<Ogre::Vector3> normals(num_verts);
    for (size_t k = 0; k < num_verts; ++k)
    {
      Eigen::Vector3f v = ((*mesh_vertices)[static_cast<size_t>((*mesh_faces)[++t])]).cast<float>();
      vertices[k] = Ogre::Vector3(v.x(), v.y(), v.z());
    }

    Ogre::Vector3 side1 = vertices[0] - vertices[1];
    Ogre::Vector3 side2 = vertices[1] - vertices[2];
    normal = side1.crossProduct(side2);
    normal.normalise();

    for (size_t k = 0; k < num_verts; ++k)
      normals[k] = normal;

    for (size_t k = 2; k < num_verts; ++k)
    {
      if (k == 2)
      {
        object->position(vertices[0]);
        object->normal(normals[0]);

        object->position(vertices[1]);
        object->normal(normals[1]);

        object->position(vertices[2]);
        object->normal(normals[2]);

        object->triangle(vertexCount + 0, vertexCount + 1, vertexCount + 2);
      }
      else
      {
        object->position(vertices[k]);
        object->normal(normals[k]);

        object->triangle(vertexCount + 0,
                         static_cast<Ogre::uint32>(vertexCount + (k - 1)),
                         static_cast<Ogre::uint32>(vertexCount + k));
      }
    }

    vertexCount += static_cast<Ogre::uint32>(num_verts);
  }

  object->end();

  auto entity = entity_container.addUntracked();
  Ogre::MeshPtr ogre_mesh = object->convertToMesh(entity.unique_name, Ogre::ResourceGroupManager::DEFAULT_RESOURCE_GROUP_NAME);
  ogre_mesh->buildEdgeList();

  Ogre::Entity* ogre_entity = scene.createEntity(entity.unique_name);

  delete object;

  return ogre_entity;
}

rviz::PointCloud* createPointCloud(std::vector<rviz::PointCloud::Point>&& points,
                                   tesseract_gui::EntityContainer &entity_container,
                                   float size,
                                   tesseract_geometry::Octree::SubType subtype)
{
  auto entity = entity_container.addUntracked();
  auto* cloud = new rviz::PointCloud();
  cloud->clear();
  cloud->setName(entity.unique_name);

  float new_size = size;
  if (subtype == tesseract_geometry::Octree::SubType::BOX)
  {
    cloud->setRenderMode(rviz::PointCloud::RM_BOXES);
  }
  else if (subtype == tesseract_geometry::Octree::SubType::SPHERE_INSIDE)
  {
    cloud->setRenderMode(rviz::PointCloud::RM_SPHERES);
  }
  else if (subtype == tesseract_geometry::Octree::SubType::SPHERE_OUTSIDE)
  {
    cloud->setRenderMode(rviz::PointCloud::RM_SPHERES);
    new_size = std::sqrt(float(2) * size * size);
  }

  cloud->setDimensions(new_size, new_size, new_size);
  cloud->addPoints(&points.front(), static_cast<unsigned>(points.size()));
  points.clear();
  return cloud;
}

std::vector<std::string> loadSceneGraph(Ogre::SceneManager& scene,
                                        tesseract_gui::EntityContainer &entity_container,
                                        const tesseract_scene_graph::SceneGraph& scene_graph,
                                        const std::string& prefix)
{
  std::vector<std::string> link_names;
  Ogre::SceneNode* root_node = scene.getRootSceneNode();
  if (prefix.empty())
  {
    for (const auto& link : scene_graph.getLinks())
    {
      root_node->addChild(loadLink(scene, entity_container, *link));
      link_names.push_back(link->getName());
    }
  }
  else
  {
    for (const auto& link : scene_graph.getLinks())
    {
      auto clone_link = link->clone(prefix + link->getName());
      root_node->addChild(loadLink(scene, entity_container, clone_link));
      link_names.push_back(clone_link.getName());
    }
  }
  return link_names;
}

Ogre::SceneNode* loadLink(Ogre::SceneManager& scene,
                                        tesseract_gui::EntityContainer &entity_container,
                                        const tesseract_scene_graph::Link& link)
{
  auto entity = entity_container.addVisual(link.getName());
  Ogre::SceneNode* scene_node = scene.createSceneNode(entity.unique_name);

  scene_node->addChild(loadLinkVisuals(scene, entity_container, link));
  scene_node->addChild(loadLinkCollisions(scene, entity_container, link));
  scene_node->addChild(loadLinkAxis(scene, entity_container, link));

  return scene_node;
}

Ogre::SceneNode* loadLinkVisuals(Ogre::SceneManager& scene,
                                 tesseract_gui::EntityContainer& entity_container,
                                 const tesseract_scene_graph::Link& link)
{
  std::string name = link.getName() + "::Visuals";
  auto entity = entity_container.addVisual(name);

  Ogre::SceneNode* scene_node = scene.createSceneNode(entity.unique_name);

  std::vector<tesseract_scene_graph::Visual::Ptr>::const_iterator vi;
  for (vi = link.visual.begin(); vi != link.visual.end(); vi++)
  {
    tesseract_scene_graph::Visual::Ptr visual = *vi;
    if (visual && visual->geometry)
    {
      Ogre::SceneNode* geom_scene_node = loadLinkGeometry(scene, entity_container, *visual->geometry, Eigen::Vector3d::Ones(), visual->origin, visual->material, false);
      scene_node->addChild(geom_scene_node);
    }
  }

  return scene_node;
}

Ogre::SceneNode* loadLinkCollisions(Ogre::SceneManager& scene,
                                    tesseract_gui::EntityContainer& entity_container,
                                    const tesseract_scene_graph::Link& link)
{
  std::string name = link.getName() + "::Collisions";
  auto entity = entity_container.addVisual(name);

  Ogre::SceneNode* scene_node = scene.createSceneNode(entity.unique_name);

  std::vector<tesseract_scene_graph::Collision::Ptr>::const_iterator vi;
  for (vi = link.collision.begin(); vi != link.collision.end(); vi++)
  {
    tesseract_scene_graph::Collision::Ptr collision = *vi;
    if (collision && collision->geometry)
    {
      Ogre::SceneNode* geom_scene_node = loadLinkGeometry(scene, entity_container, *collision->geometry, Eigen::Vector3d::Ones(), collision->origin, nullptr, false);
      scene_node->addChild(geom_scene_node);
    }
  }

  return scene_node;
}

Ogre::SceneNode* loadLinkAxis(Ogre::SceneManager& scene,
                                            tesseract_gui::EntityContainer& entity_container,
                                            const tesseract_scene_graph::Link& link)
{
  std::string name = link.getName() + "::Axis";
  auto entity = entity_container.addVisual(name);

  Ogre::SceneNode* axis = scene.createSceneNode(entity.unique_name);
  auto axis_red = std::make_shared<tesseract_scene_graph::Material>("tesseract_gui_axis_red_material");
  axis_red->color = Eigen::Vector4d(1, 0, 0, 1);
  auto axis_green = std::make_shared<tesseract_scene_graph::Material>("tesseract_gui_axis_green_material");
  axis_green->color = Eigen::Vector4d(0, 1, 0, 1);
  auto axis_blue = std::make_shared<tesseract_scene_graph::Material>("tesseract_gui_axis_blue_material");
  axis_blue->color = Eigen::Vector4d(0, 0, 1, 1);

//  {
//    auto gv_entity = entity_container.addUntracked();
//    Ogre::SceneNode* cylinder = scene.createSceneNode(gv_entity.unique_name);
//    Eigen::Isometry3d pose = Eigen::Isometry3d::Identity();
//    pose.translation() = Eigen::Vector3d(0,0,0.5);
//    cylinder->SetLocalPose(ignition::math::eigen3::convert(pose));
//    cylinder->AddGeometry(scene.CreateCylinder());
//    cylinder->Scale(0.1, 0.1, 1.0);
//    cylinder->SetMaterial(axis_blue);
//    axis->AddChild(cylinder);
//  }

//  {
//    auto gv_id = static_cast<unsigned>(entity_container.createEntityID());
//    ignition::rendering::VisualPtr cylinder = scene.CreateVisual(gv_id);
//    Eigen::Isometry3d pose = Eigen::Isometry3d::Identity();
//    pose.rotate(Eigen::AngleAxisd(M_PI_2, Eigen::Vector3d::UnitY()));
//    pose.translation() = Eigen::Vector3d(0.5,0,0);
//    cylinder->SetLocalPose(ignition::math::eigen3::convert(pose));
//    cylinder->AddGeometry(scene.CreateCylinder());
//    cylinder->Scale(0.1, 0.1, 1.0);
//    cylinder->SetMaterial(axis_red);
//    axis->AddChild(cylinder);
//  }

//  {
//    auto gv_id = static_cast<unsigned>(entity_container.createEntityID());
//    ignition::rendering::VisualPtr cylinder = scene.CreateVisual(gv_id);
//    Eigen::Isometry3d pose = Eigen::Isometry3d::Identity();
//    pose.rotate(Eigen::AngleAxisd(-M_PI_2, Eigen::Vector3d::UnitX()));
//    pose.translation() = Eigen::Vector3d(0,0.5,0);
//    cylinder->SetLocalPose(ignition::math::eigen3::convert(pose));
//    cylinder->AddGeometry(scene.CreateCylinder());
//    cylinder->Scale(0.1, 0.1, 1.0);
//    cylinder->SetMaterial(axis_green);
//    axis->AddChild(cylinder);
//  }

//  axis->SetInheritScale(false);
//  axis->Scale(0.1, 0.1, 0.1);
//  axis->SetVisible(true);
  return axis;
}

Ogre::SceneNode* loadLinkGeometry(Ogre::SceneManager& scene,
                      tesseract_gui::EntityContainer &entity_container,
                      const tesseract_geometry::Geometry& geometry,
                      const Eigen::Vector3d& scale,
                      const Eigen::Isometry3d& local_pose,
                      const tesseract_scene_graph::Material::ConstPtr& material,
                      bool is_visual)
{
  Ogre::Entity* ogre_entity = nullptr;  // default in case nothing works.
  Ogre::Vector3 ogre_scale(Ogre::Vector3::UNIT_SCALE);
  Ogre::Vector3 offset_position(Ogre::Vector3::ZERO);
  Ogre::Quaternion offset_orientation(Ogre::Quaternion::IDENTITY);

  {
    const Eigen::Vector3d& pos = local_pose.translation();
    Ogre::Vector3 position(static_cast<float>(pos(0)), static_cast<float>(pos(1)), static_cast<float>(pos(2)));

    Eigen::Quaterniond rot(local_pose.linear());
    Ogre::Quaternion orientation(Ogre::Quaternion::IDENTITY);
    orientation = orientation * Ogre::Quaternion(static_cast<float>(rot.w()),
                                                 static_cast<float>(rot.x()),
                                                 static_cast<float>(rot.y()),
                                                 static_cast<float>(rot.z()));

    offset_position = position;
    offset_orientation = orientation;
  }

  switch (geometry.getType())
  {
    case tesseract_geometry::GeometryType::SPHERE:
    {
      const auto& sphere = static_cast<const tesseract_geometry::Sphere&>(geometry);
      auto entity = entity_container.addUntracked();
      ogre_entity = scene.createEntity(entity.unique_name, "tesseract_sphere.mesh");
      float diameter = static_cast<float>(sphere.getRadius()) * 2.0f;
      ogre_scale = Ogre::Vector3(diameter, diameter, diameter);
      break;
    }
    case tesseract_geometry::GeometryType::BOX:
    {
      const auto& box = static_cast<const tesseract_geometry::Box&>(geometry);
      auto entity = entity_container.addUntracked();
      ogre_entity = scene.createEntity(entity.unique_name, "tesseract_cube.mesh");
      ogre_scale = Ogre::Vector3(static_cast<float>(box.getX()), static_cast<float>(box.getY()), static_cast<float>(box.getZ()));
      break;
    }
    case tesseract_geometry::GeometryType::CYLINDER:
    {
      const auto& cylinder = static_cast<const tesseract_geometry::Cylinder&>(geometry);
      auto entity = entity_container.addUntracked();
      ogre_entity = scene.createEntity(entity.unique_name, "tesseract_cylinder.mesh");
      ogre_scale = Ogre::Vector3(static_cast<float>(cylinder.getRadius() * 2),
                            static_cast<float>(cylinder.getRadius() * 2),
                            static_cast<float>(cylinder.getLength()));
      break;
    }
    case tesseract_geometry::GeometryType::CONE:
    {
      const auto& cone = static_cast<const tesseract_geometry::Cone&>(geometry);
      auto entity = entity_container.addUntracked();
      ogre_entity = scene.createEntity(entity.unique_name, "tesseract_cone.mesh");
      ogre_scale = Ogre::Vector3(static_cast<float>(cone.getRadius() * 2),
                            static_cast<float>(cone.getRadius() * 2),
                            static_cast<float>(cone.getLength()));
      break;
    }
    case tesseract_geometry::GeometryType::CAPSULE:
    {
      const auto& capsule = static_cast<const tesseract_geometry::Capsule&>(geometry);
      auto entity = entity_container.addUntracked();
      ogre_entity = scene.createEntity(entity.unique_name, "tesseract_capsule.mesh");
      ogre_scale = Ogre::Vector3(static_cast<float>(capsule.getRadius() * 2),
                            static_cast<float>(capsule.getRadius() * 2),
                            static_cast<float>((0.5 * capsule.getLength()) + capsule.getRadius()));
      break;
    }
    case tesseract_geometry::GeometryType::MESH:
    {
      const auto& mesh = static_cast<const tesseract_geometry::Mesh&>(geometry);

      if (mesh.getResource() && mesh.getResource()->isFile() && is_visual)
      {
        std::string model_name = "file://" + mesh.getResource()->getFilePath();

        const Eigen::Vector3d& mesh_scale = mesh.getScale();
        ogre_scale = Ogre::Vector3(
            static_cast<float>(mesh_scale.x()), static_cast<float>(mesh_scale.y()), static_cast<float>(mesh_scale.z()));

        try
        {
          rviz::loadMeshFromResource(model_name);
          auto entity = entity_container.addUntracked();
          ogre_entity = scene.createEntity(entity.unique_name);
        }
        catch (Ogre::InvalidParametersException& e)
        {
          ROS_ERROR("Could not convert mesh resource '%s'. It might be an empty mesh: %s", model_name.c_str(), e.what());
        }
        catch (Ogre::Exception& e)
        {
          ROS_ERROR("Could not load model '%s': %s", model_name.c_str(), e.what());
        }
      }
      else
      {
        ogre_entity = createEntityForMeshData(scene, entity_container, mesh.getVertices(), mesh.getFaces());
      }

      break;
    }
    case tesseract_geometry::GeometryType::CONVEX_MESH:
    {
      const auto& mesh = static_cast<const tesseract_geometry::ConvexMesh&>(geometry);

      if (mesh.getResource() && mesh.getResource()->isFile() && is_visual)
      {
        std::string model_name = "file://" + mesh.getResource()->getFilePath();

        const Eigen::Vector3d& mesh_scale = mesh.getScale();
        ogre_scale = Ogre::Vector3(
            static_cast<float>(mesh_scale.x()), static_cast<float>(mesh_scale.y()), static_cast<float>(mesh_scale.z()));

        try
        {
          rviz::loadMeshFromResource(model_name);
          ogre_entity = scene.createEntity(model_name);
        }
        catch (Ogre::InvalidParametersException& e)
        {
          ROS_ERROR("Could not convert mesh resource '%s'. It might be an empty mesh: %s", model_name.c_str(), e.what());
        }
        catch (Ogre::Exception& e)
        {
          ROS_ERROR("Could not load model '%s': %s", model_name.c_str(), e.what());
        }
      }
      else
      {
        ogre_entity = createEntityForMeshData(scene, entity_container, mesh.getVertices(), mesh.getFaces());
      }
      break;
    }
    case tesseract_geometry::GeometryType::OCTREE:
    {
      std::size_t max_octree_depth = 0;
      double color_factor = 0.8;
      OctreeVoxelRenderMode octree_voxel_rendering = OCTOMAP_OCCUPIED_VOXELS;
      OctreeVoxelColorMode octree_color_mode = OCTOMAP_Z_AXIS_COLOR;
      std::size_t octree_depth;
      auto entity = entity_container.addUntracked();
      Ogre::SceneNode* offset_node = scene.createSceneNode(entity.unique_name);
      std::vector<OctreeDataContainer>* octree_objects;

      const auto& octomap = static_cast<const tesseract_geometry::Octree&>(geometry);
      const std::shared_ptr<const octomap::OcTree>& octree = octomap.getOctree();

      if (!max_octree_depth)
        octree_depth = octree->getTreeDepth();
      else
        octree_depth = std::min(max_octree_depth, static_cast<size_t>(octree->getTreeDepth()));

      std::vector<std::vector<rviz::PointCloud::Point>> pointBuf;
      pointBuf.resize(octree_depth);

      // get dimensions of octree
      double minX, minY, minZ, maxX, maxY, maxZ;
      octree->getMetricMin(minX, minY, minZ);
      octree->getMetricMax(maxX, maxY, maxZ);

      auto render_mode_mask = static_cast<unsigned int>(octree_voxel_rendering);

      size_t pointCount = 0;
      {
        // traverse all leafs in the tree:
        for (octomap::OcTree::iterator it = octree->begin(static_cast<unsigned char>(octree_depth)),
                                       end = octree->end();
             it != end;
             ++it)
        {
          bool display_voxel = false;

          // the left part evaluates to 1 for free voxels and 2 for occupied voxels
          if ((static_cast<unsigned>(octree->isNodeOccupied(*it)) + 1) & render_mode_mask)
          {
            // Disabling because it does not work correctly with a pruned octomap
            display_voxel = true;

            //            // check if current voxel has neighbors on all sides -> no need to be
            //            // displayed
            //            bool allNeighborsFound = true;

            //            octomap::OcTreeKey key;
            //            octomap::OcTreeKey nKey = it.getKey();

            //            for (key[2] = static_cast<octomap::key_type>(nKey[2] - 1);
            //                 allNeighborsFound && key[2] <= static_cast<octomap::key_type>(nKey[2] + 1);
            //                 ++key[2])
            //            {
            //              for (key[1] = static_cast<octomap::key_type>(nKey[1] - 1);
            //                   allNeighborsFound && key[1] <= static_cast<octomap::key_type>(nKey[1] + 1);
            //                   ++key[1])
            //              {
            //                for (key[0] = static_cast<octomap::key_type>(nKey[0] - 1);
            //                     allNeighborsFound && key[0] <= static_cast<octomap::key_type>(nKey[0] + 1);
            //                     ++key[0])
            //                {
            //                  if (key != nKey)
            //                  {
            //                    octomap::OcTreeNode* node = octree->search(key);

            //                    // the left part evaluates to 1 for free voxels and 2 for
            //                    // occupied voxels
            //                    if (!(node && (static_cast<unsigned>(octree->isNodeOccupied(node)) + 1) &
            //                    render_mode_mask))
            //                    {
            //                      // we do not have a neighbor => break!
            //                      allNeighborsFound = false;
            //                    }
            //                  }
            //                }
            //              }
            //            }

            //            display_voxel |= !allNeighborsFound;
          }

          if (display_voxel)
          {
            rviz::PointCloud::Point newPoint;

            newPoint.position.x = static_cast<float>(it.getX());
            newPoint.position.y = static_cast<float>(it.getY());
            newPoint.position.z = static_cast<float>(it.getZ());

            float cell_probability;

            switch (octree_color_mode)
            {
              case OCTOMAP_Z_AXIS_COLOR:
                setOctomapColor(static_cast<double>(newPoint.position.z), minZ, maxZ, color_factor, &newPoint);
                break;
              case OCTOMAP_PROBABLILTY_COLOR:
                cell_probability = static_cast<float>(it->getOccupancy());
                newPoint.setColor((1.0f - cell_probability), cell_probability, 0.0f);
                break;
              default:
                break;
            }

            // push to point vectors
            unsigned int depth = it.getDepth();
            pointBuf[depth - 1].push_back(newPoint);

            ++pointCount;
          }
        }
      }

      for (unsigned i = 0; i < octree_depth; ++i)
      {
        OctreeDataContainer data;
        data.size = static_cast<float>(octree->getNodeSize(static_cast<unsigned>(i + 1)));
        data.points = std::vector<rviz::PointCloud::Point>(pointBuf[i]);
        data.point_cloud = createPointCloud(std::move(pointBuf[i]), entity_container, data.size, octomap.getSubType());
        data.shape_type = octomap.getSubType();

        offset_node->attachObject(data.point_cloud);
        octree_objects->push_back(data);
      }

      offset_node->setScale(ogre_scale);
      offset_node->setPosition(offset_position);
      offset_node->setOrientation(offset_orientation);

      return offset_node;
    }
    default:
      ROS_WARN("Unsupported geometry type for element: %d", geometry.getType());
      break;
  }

  if (ogre_entity)
  {
    auto entity = entity_container.addUntracked();
    Ogre::SceneNode* offset_node = scene.createSceneNode(entity.unique_name);
    std::vector<Ogre::Entity*>* meshes;

    offset_node->attachObject(ogre_entity);
    offset_node->setScale(ogre_scale);
    offset_node->setPosition(offset_position);
    offset_node->setOrientation(offset_orientation);

    for (uint32_t i = 0; i < ogre_entity->getNumSubEntities(); ++i)
    {
      auto ogre_material = loadMaterial(scene, material);

      // Assign materials only if the submesh does not have one already

      Ogre::SubEntity* sub = ogre_entity->getSubEntity(i);
      const std::string& material_name = sub->getMaterialName();

      if (material_name == "BaseWhite" || material_name == "BaseWhiteNoLighting")
      {
        sub->setMaterialName(ogre_material->getName());
      }
      else
      {
        sub->setMaterialName(ogre_material->getName());
      }
    }

    meshes->push_back(ogre_entity);
    return offset_node;
  }

  return nullptr;



//  ignition::rendering::MaterialPtr ign_material = loadMaterial(scene, material);
//  switch (geometry.getType())
//  {
//    case tesseract_geometry::GeometryType::BOX:
//    {
//      auto gv_id = static_cast<unsigned>(entity_container.createEntityID());
//      ignition::rendering::VisualPtr box = scene.CreateVisual(gv_id);
//      box->SetLocalPose(ignition::math::eigen3::convert(local_pose));
//      box->AddGeometry(scene.CreateBox());

//      const auto& shape = static_cast<const tesseract_geometry::Box&>(geometry);
//      box->Scale(shape.getX() * scale.x(), shape.getY() * scale.y(), shape.getZ() * scale.z());
//      box->SetMaterial(ign_material);
//      return box;
//    }
//    case tesseract_geometry::GeometryType::SPHERE:
//    {
//      auto gv_id = static_cast<unsigned>(entity_container.createEntityID());
//      ignition::rendering::VisualPtr sphere = scene.CreateVisual(gv_id);
//      sphere->SetLocalPose(ignition::math::eigen3::convert(local_pose));
//      sphere->AddGeometry(scene.CreateSphere());

//      const auto& shape = static_cast<const tesseract_geometry::Sphere&>(geometry);
//      sphere->Scale(shape.getRadius() * scale.x(), shape.getRadius() * scale.y(), shape.getRadius() * scale.z());
//      sphere->SetMaterial(ign_material);
//      return sphere;
//    }
//    case tesseract_geometry::GeometryType::CYLINDER:
//    {
//      auto gv_id = static_cast<unsigned>(entity_container.createEntityID());
//      ignition::rendering::VisualPtr cylinder = scene.CreateVisual(gv_id);
//      cylinder->SetLocalPose(ignition::math::eigen3::convert(local_pose));
//      cylinder->AddGeometry(scene.CreateCylinder());

//      const auto& shape = static_cast<const tesseract_geometry::Cylinder&>(geometry);
//      cylinder->Scale(shape.getRadius() * scale.x(), shape.getRadius() * scale.y(), shape.getLength() * scale.z());
//      cylinder->SetMaterial(ign_material);
//      return cylinder;
//    }
//    case tesseract_geometry::GeometryType::CONE:
//    {
//      auto gv_id = static_cast<unsigned>(entity_container.createEntityID());
//      ignition::rendering::VisualPtr cone = scene.CreateVisual(gv_id);
//      cone->SetLocalPose(ignition::math::eigen3::convert(local_pose));
//      cone->AddGeometry(scene.CreateCone());

//      const auto& shape = static_cast<const tesseract_geometry::Cone&>(geometry);
//      cone->Scale(shape.getRadius() * scale.x(), shape.getRadius() * scale.y(), shape.getLength() * scale.z());
//      cone->SetMaterial(ign_material);
//      return cone;
//    }
//    case tesseract_geometry::GeometryType::CAPSULE:
//    {
//      return nullptr;
//    }
//    case tesseract_geometry::GeometryType::MESH:
//    {
//      const auto& shape = static_cast<const tesseract_geometry::Mesh&>(geometry);
//      auto resource = shape.getResource();
//      if (resource)
//      {
//        auto gv_id = static_cast<unsigned>(entity_container.createEntityID());
//        ignition::rendering::VisualPtr mesh = scene.CreateVisual(gv_id);
//        mesh->SetLocalPose(ignition::math::eigen3::convert(local_pose));

//        ignition::rendering::MeshDescriptor descriptor;
//        descriptor.meshName = resource->getFilePath();
//        ignition::common::MeshManager* mesh_manager = ignition::common::MeshManager::Instance();
//        descriptor.mesh = mesh_manager->Load(descriptor.meshName);
//        ignition::rendering::MeshPtr mesh_geom = scene.CreateMesh(descriptor);

//        if (!isMeshWithColor(resource->getFilePath()))
//          mesh_geom->SetMaterial(ign_material);

//        mesh->AddGeometry(mesh_geom);
//        return mesh;
//      }

//      assert(false);
//      return nullptr;
//    }
//    case tesseract_geometry::GeometryType::CONVEX_MESH:
//    {
//      const auto& shape = static_cast<const tesseract_geometry::ConvexMesh&>(geometry);
//      auto resource = shape.getResource();
//      if (resource)
//      {
//        auto gv_id = static_cast<unsigned>(entity_container.createEntityID());
//        ignition::rendering::VisualPtr mesh = scene.CreateVisual(gv_id);
//        mesh->SetLocalPose(ignition::math::eigen3::convert(local_pose));

//        ignition::rendering::MeshDescriptor descriptor;
//        descriptor.meshName = resource->getFilePath();
//        ignition::common::MeshManager* mesh_manager = ignition::common::MeshManager::Instance();
//        descriptor.mesh = mesh_manager->Load(descriptor.meshName);
//        ignition::rendering::MeshPtr mesh_geom = scene.CreateMesh(descriptor);

//        if (!isMeshWithColor(resource->getFilePath()))
//          mesh_geom->SetMaterial(ign_material);

//        mesh->AddGeometry(mesh_geom);
//        return mesh;
//      }

//      assert(false);
//      return nullptr;
//    }
//    case tesseract_geometry::GeometryType::OCTREE:
//    {
//      const auto& shape = static_cast<const tesseract_geometry::Octree&>(geometry);

//      // TODO: Need to implement
//      assert(false);
//      return nullptr;
//    }
//    default:
//    {
////      CONSOLE_BRIDGE_logError("This geometric shape type (%d) is not supported",
////                              static_cast<int>(geometry->getType()));
//     return nullptr;
//    }
//  }
}

Ogre::MaterialPtr loadMaterial(Ogre::SceneManager& scene,
                               const tesseract_scene_graph::Material::ConstPtr& material)
{
  Ogre::MaterialPtr mat = Ogre::MaterialManager::getSingleton().create(
      material_name_generator.generate(), Ogre::ResourceGroupManager::DEFAULT_RESOURCE_GROUP_NAME);
  mat->setReceiveShadows(true);
  mat->getTechnique(0)->setLightingEnabled(true);

  if (material == nullptr)
  {
    const Eigen::Vector4d& rgba = tesseract_scene_graph::DEFAULT_TESSERACT_MATERIAL->color;
    const Eigen::Vector4f& col = material->color.cast<float>();
    mat->getTechnique(0)->setAmbient(col(0) * 0.5f, col(1) * 0.5f, col(2) * 0.5f);
    mat->getTechnique(0)->setDiffuse(col(0), col(1), col(2), col(3));
    return mat;
  }

  if (material->texture_filename.empty())
  {
    const Eigen::Vector4d& rgba = material->color;
    const Eigen::Vector4f& col = material->color.cast<float>();
    mat->getTechnique(0)->setAmbient(col(0) * 0.5f, col(1) * 0.5f, col(2) * 0.5f);
    mat->getTechnique(0)->setDiffuse(col(0), col(1), col(2), col(3));

//    material_alpha_ = col(3);
    return mat;
  }
  else
  {
    std::string filename = material->texture_filename;
    if (!Ogre::TextureManager::getSingleton().resourceExists(filename))
    {
      resource_retriever::Retriever retriever;
      resource_retriever::MemoryResource res;
      try
      {
        res = retriever.get(filename);
      }
      catch (resource_retriever::Exception& e)
      {
        ROS_ERROR("%s", e.what());
      }

      if (res.size != 0)
      {
        Ogre::DataStreamPtr stream(new Ogre::MemoryDataStream(res.data.get(), res.size));
        Ogre::Image image;
        std::string extension = tesseract_common::fs::extension(tesseract_common::fs::path(filename));

        if (extension[0] == '.')
        {
          extension = extension.substr(1, extension.size() - 1);
        }

        try
        {
          image.load(stream, extension);
          Ogre::TextureManager::getSingleton().loadImage(
              filename, Ogre::ResourceGroupManager::DEFAULT_RESOURCE_GROUP_NAME, image);
        }
        catch (Ogre::Exception& e)
        {
          ROS_ERROR("Could not load texture [%s]: %s", filename.c_str(), e.what());
        }
      }
    }

    Ogre::Pass* pass = mat->getTechnique(0)->getPass(0);
    Ogre::TextureUnitState* tex_unit = pass->createTextureUnitState();

    tex_unit->setTextureName(filename);
  }

  return Ogre::MaterialPtr();
}



void setOctomapColor(double z_pos, double min_z, double max_z, double color_factor, rviz::PointCloud::Point* point)
{
  int i;
  float m, n, f;

  float s = 1.0f;
  float v = 1.0f;

  float h = static_cast<float>((1.0 - std::min(std::max((z_pos - min_z) / (max_z - min_z), 0.0), 1.0)) * color_factor);

  h -= static_cast<float>(floor(static_cast<double>(h)));
  h *= 6;
  i = static_cast<int>(floor(static_cast<double>(h)));
  f = h - static_cast<float>(i);
  if (!(i & 1))
    f = 1 - f;  // if i is even
  m = v * (1 - s);
  n = v * (1 - s * f);

  switch (i)
  {
    case 6:
    case 0:
      point->setColor(v, n, m);
      break;
    case 1:
      point->setColor(n, v, m);
      break;
    case 2:
      point->setColor(m, v, n);
      break;
    case 3:
      point->setColor(m, n, v);
      break;
    case 4:
      point->setColor(n, m, v);
      break;
    case 5:
      point->setColor(v, m, n);
      break;
    default:
      point->setColor(1.0f, 0.5f, 0.5f);
      break;
  }
}


}
