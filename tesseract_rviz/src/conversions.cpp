/**
 * @file conversions.cpp
 * @brief Common conversion used
 *
 * @author Levi Armstrong
 * @date Dec 18, 2017
 * @version TODO
 * @bug No known bugs
 *
 * @copyright Copyright (c) 2017, Southwest Research Institute
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
#include <tesseract_rviz/conversions.h>

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
#include <ros/package.h>
#include <boost/algorithm/string.hpp>

#include <rviz/load_resource.h>
#include <rviz/mesh_loader.h>
#include <rviz/ogre_helpers/axes.h>
#include <rviz/ogre_helpers/mesh_shape.h>
#include <rviz/ogre_helpers/object.h>
#include <rviz/ogre_helpers/shape.h>
#include <rviz/ogre_helpers/point_cloud.h>

#include <tesseract_common/filesystem.h>
#include <tesseract_common/resource_locator.h>
#include <tesseract_geometry/geometries.h>

const std::string USER_VISIBILITY = "user_visibility";

namespace tesseract_rviz
{
static Ogre::NameGenerator material_name_generator("tesseract::material::");

void toEigen(Eigen::Isometry3d& transform, const Ogre::Vector3& position, const Ogre::Quaternion& orientation)
{
  transform.linear() = Eigen::Quaterniond(orientation.w, orientation.x, orientation.y, orientation.z).matrix();
  transform.translation() = Eigen::Vector3d(position.x, position.y, position.z);
}

void toOgre(Ogre::Vector3& position, Ogre::Quaternion& orientation, const Eigen::Isometry3d& transform)
{
  Eigen::Vector3f robot_visual_position = transform.translation().cast<float>();
  Eigen::Quaternionf robot_visual_orientation(transform.rotation().cast<float>());
  position = Ogre::Vector3(robot_visual_position.x(), robot_visual_position.y(), robot_visual_position.z());
  orientation = Ogre::Quaternion(robot_visual_orientation.w(),
                                 robot_visual_orientation.x(),
                                 robot_visual_orientation.y(),
                                 robot_visual_orientation.z());
}

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

void addOgreResourceLocation()
{
  std::string tesseract_rviz_path = ros::package::getPath("tesseract_rviz");
  Ogre::ResourceGroupManager::getSingleton().addResourceLocation(
      tesseract_rviz_path + "/ogre_media/models", "FileSystem", "tesseract_rviz");
}

std::string getEnvNamespaceFromTopic(const std::string& topic)
{
  std::vector<std::string> tokens;
  boost::split(tokens, topic, boost::is_any_of("/"), boost::token_compress_on);
  tokens.erase(std::remove_if(tokens.begin(), tokens.end(), [](const std::string& token) { return token.empty(); }),
               tokens.end());
  if (!tokens.empty())
    return tokens.at(0);

  return std::string();
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

  auto entity = entity_container.addUntrackedEntity(tesseract_gui::EntityContainer::RESOURCE_NS);
  std::string mesh_name = entity.unique_name + "::mesh";
  Ogre::MeshPtr ogre_mesh = object->convertToMesh(mesh_name, Ogre::ResourceGroupManager::DEFAULT_RESOURCE_GROUP_NAME);
  ogre_mesh->buildEdgeList();

  Ogre::Entity* ogre_entity = scene.createEntity(entity.unique_name, mesh_name);

  delete object;

  return ogre_entity;
}

std::shared_ptr<rviz::PointCloud> createPointCloud(std::vector<rviz::PointCloud::Point>&& points,
                                                   tesseract_gui::EntityContainer& entity_container,
                                                   float size,
                                                   tesseract_geometry::OctreeSubType subtype)
{
  auto entity = entity_container.addUntrackedEntity(tesseract_gui::EntityContainer::RESOURCE_NS);
  auto cloud = std::make_shared<rviz::PointCloud>();
  cloud->clear();
  cloud->setName(entity.unique_name);

  float new_size = size;
  if (subtype == tesseract_geometry::OctreeSubType::BOX)
  {
    cloud->setRenderMode(rviz::PointCloud::RM_BOXES);
  }
  else if (subtype == tesseract_geometry::OctreeSubType::SPHERE_INSIDE)
  {
    cloud->setRenderMode(rviz::PointCloud::RM_SPHERES);
  }
  else if (subtype == tesseract_geometry::OctreeSubType::SPHERE_OUTSIDE)
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
                                        Ogre::SceneNode& parent_node,
                                        tesseract_gui::EntityManager& entity_manager,
                                        const tesseract_scene_graph::SceneGraph& scene_graph,
                                        const std::string& prefix)
{
  std::vector<std::string> link_names;
  if (prefix.empty())
  {
    for (const auto& link : scene_graph.getLinks())
    {
      auto entity_container = entity_manager.getEntityContainer(link->getName());
      parent_node.addChild(loadLink(scene, *entity_container, *link));
      link_names.push_back(link->getName());
    }
  }
  else
  {
    for (const auto& link : scene_graph.getLinks())
    {
      auto clone_link = link->clone(prefix + link->getName());
      auto entity_container = entity_manager.getEntityContainer(clone_link.getName());
      parent_node.addChild(loadLink(scene, *entity_container, clone_link));
      link_names.push_back(clone_link.getName());
    }
  }
  return link_names;
}

Ogre::SceneNode* loadLink(Ogre::SceneManager& scene,
                          tesseract_gui::EntityContainer& entity_container,
                          const tesseract_scene_graph::Link& link,
                          tesseract_scene_graph::Material::ConstPtr visual_material_override,
                          tesseract_scene_graph::Material::ConstPtr collision_material_override)
{
  auto entity = entity_container.addTrackedEntity(tesseract_gui::EntityContainer::VISUAL_NS, link.getName());
  Ogre::SceneNode* scene_node = scene.createSceneNode(entity.unique_name);
  scene_node->getUserObjectBindings().setUserAny(USER_VISIBILITY, Ogre::Any(true));

  Ogre::SceneNode* visuals_scene_node = loadLinkVisuals(scene, entity_container, link, visual_material_override);
  visuals_scene_node->getUserObjectBindings().setUserAny(USER_VISIBILITY, Ogre::Any(true));
  scene_node->addChild(visuals_scene_node);

  Ogre::SceneNode* collisions_scene_node =
      loadLinkCollisions(scene, entity_container, link, collision_material_override);
  collisions_scene_node->getUserObjectBindings().setUserAny(USER_VISIBILITY, Ogre::Any(false));
  scene_node->addChild(collisions_scene_node);

  if (!link.visual.empty() || !link.collision.empty())
  {
    Ogre::AxisAlignedBox aabb = getAABB(*scene_node, false);
    if (aabb.isFinite())
    {
      Ogre::SceneNode* wirebox_scene_node = loadLinkWireBox(scene, entity_container, link, aabb);
      wirebox_scene_node->getUserObjectBindings().setUserAny(USER_VISIBILITY, Ogre::Any(false));
      scene_node->addChild(wirebox_scene_node);
    }
  }

  Ogre::SceneNode* axis_scene_node = loadLinkAxis(scene, entity_container, link);
  axis_scene_node->getUserObjectBindings().setUserAny(USER_VISIBILITY, Ogre::Any(false));
  scene_node->addChild(axis_scene_node);

  return scene_node;
}

Ogre::SceneNode* loadLinkVisuals(Ogre::SceneManager& scene,
                                 tesseract_gui::EntityContainer& entity_container,
                                 const tesseract_scene_graph::Link& link,
                                 tesseract_scene_graph::Material::ConstPtr material_override)
{
  std::string name = link.getName() + "::Visuals";
  auto entity = entity_container.addTrackedEntity(tesseract_gui::EntityContainer::VISUAL_NS, name);

  Ogre::SceneNode* scene_node = scene.createSceneNode(entity.unique_name);

  std::vector<tesseract_scene_graph::Visual::Ptr>::const_iterator vi;
  for (vi = link.visual.begin(); vi != link.visual.end(); vi++)
  {
    tesseract_scene_graph::Visual::Ptr visual = *vi;
    if (visual && visual->geometry)
    {
      Ogre::MaterialPtr material;
      if (material_override == nullptr)
        material = loadLinkMaterial(scene, link, visual->material->getName());
      else
        material = loadMaterial(material_override);

      Ogre::SceneNode* geom_scene_node = loadLinkGeometry(
          scene, entity_container, *visual->geometry, Eigen::Vector3d::Ones(), visual->origin, material, true);
      scene_node->addChild(geom_scene_node);
    }
  }

  return scene_node;
}

Ogre::SceneNode* loadLinkCollisions(Ogre::SceneManager& scene,
                                    tesseract_gui::EntityContainer& entity_container,
                                    const tesseract_scene_graph::Link& link,
                                    tesseract_scene_graph::Material::ConstPtr material_override)
{
  std::string name = link.getName() + "::Collisions";
  auto entity = entity_container.addTrackedEntity(tesseract_gui::EntityContainer::VISUAL_NS, name);

  Ogre::SceneNode* scene_node = scene.createSceneNode(entity.unique_name);

  std::vector<tesseract_scene_graph::Collision::Ptr>::const_iterator vi;
  for (vi = link.collision.begin(); vi != link.collision.end(); vi++)
  {
    tesseract_scene_graph::Collision::Ptr collision = *vi;
    if (collision && collision->geometry)
    {
      Ogre::MaterialPtr material;
      if (material_override == nullptr)
        material = loadLinkMaterial(scene, link, "");
      else
        material = loadMaterial(material_override);

      Ogre::SceneNode* geom_scene_node = loadLinkGeometry(
          scene, entity_container, *collision->geometry, Eigen::Vector3d::Ones(), collision->origin, material, false);
      scene_node->addChild(geom_scene_node);
    }
  }

  scene_node->setVisible(false, true);
  return scene_node;
}

Ogre::SceneNode* loadLinkAxis(Ogre::SceneManager& scene,
                              tesseract_gui::EntityContainer& entity_container,
                              const tesseract_scene_graph::Link& link)
{
  std::string name = link.getName() + "::Axis";
  auto entity = entity_container.addTrackedEntity(tesseract_gui::EntityContainer::VISUAL_NS, name);

  Ogre::SceneNode* scene_node = scene.createSceneNode(entity.unique_name);

  /** @todo Does this need to be manually deleted */
  auto axis = std::make_shared<rviz::Axes>(&scene, scene_node);
  axis->setScale(Ogre::Vector3(0.1, 0.1, 0.1));
  entity_container.addUntrackedUnmanagedObject(tesseract_gui::EntityContainer::VISUAL_NS, axis);

  scene_node->setVisible(false, true);
  return scene_node;
}

Ogre::SceneNode* loadLinkWireBox(Ogre::SceneManager& scene,
                                 tesseract_gui::EntityContainer& entity_container,
                                 const tesseract_scene_graph::Link& link,
                                 const Ogre::AxisAlignedBox& aabb)
{
  Ogre::MaterialPtr mat = Ogre::MaterialManager::getSingleton().getByName(
      "tesseract::highlight::material", Ogre::ResourceGroupManager::DEFAULT_RESOURCE_GROUP_NAME);
  if (mat.isNull())
  {
    mat = Ogre::MaterialManager::getSingleton().create("tesseract::highlight::material",
                                                       Ogre::ResourceGroupManager::DEFAULT_RESOURCE_GROUP_NAME);
    mat->getTechnique(0)->setLightingEnabled(true);
    mat->setAmbient(1.0, 1.0, 1.0);
    mat->setDiffuse(1.0, 1.0, 1.0, 1.0);
    mat->setSpecular(1.0, 1.0, 1.0, 1.0);
    //    mat->setEmissive(1.0, 1.0, 1.0, 1.0);
  }

  std::string name = link.getName() + "::WireBox";
  auto entity = entity_container.addTrackedEntity(tesseract_gui::EntityContainer::VISUAL_NS, name);
  Ogre::SceneNode* wire_box_node = scene.createSceneNode(entity.unique_name);

  auto untracked_entity = entity_container.addUntrackedEntity(tesseract_gui::EntityContainer::RESOURCE_NS);
  Ogre::ManualObject* wire_box = scene.createManualObject(untracked_entity.unique_name);
  wire_box->clear();
  wire_box->setCastShadows(false);
  wire_box->estimateVertexCount(12);
  std::string mat_name = "RVIZ/Green";
  wire_box->begin(mat_name, Ogre::RenderOperation::OT_LINE_LIST);

  Ogre::Vector3 max = aabb.getMaximum();
  Ogre::Vector3 min = aabb.getMinimum();

  // line 0
  wire_box->position(min.x, min.y, min.z);
  wire_box->position(max.x, min.y, min.z);

  // line 1
  wire_box->position(min.x, min.y, min.z);
  wire_box->position(min.x, min.y, max.z);

  // line 2
  wire_box->position(min.x, min.y, min.z);
  wire_box->position(min.x, max.y, min.z);

  // line 3
  wire_box->position(min.x, max.y, min.z);
  wire_box->position(min.x, max.y, max.z);

  // line 4
  wire_box->position(min.x, max.y, min.z);
  wire_box->position(max.x, max.y, min.z);

  // line 5
  wire_box->position(max.x, min.y, min.z);
  wire_box->position(max.x, min.y, max.z);

  // line 6
  wire_box->position(max.x, min.y, min.z);
  wire_box->position(max.x, max.y, min.z);

  // line 7
  wire_box->position(min.x, max.y, max.z);
  wire_box->position(max.x, max.y, max.z);

  // line 8
  wire_box->position(min.x, max.y, max.z);
  wire_box->position(min.x, min.y, max.z);

  // line 9
  wire_box->position(max.x, max.y, min.z);
  wire_box->position(max.x, max.y, max.z);

  // line 10
  wire_box->position(max.x, min.y, max.z);
  wire_box->position(max.x, max.y, max.z);

  // line 11
  wire_box->position(min.x, min.y, max.z);
  wire_box->position(max.x, min.y, max.z);

  wire_box->end();

  wire_box_node->attachObject(wire_box);
  wire_box_node->setVisible(false);
  return wire_box_node;
}

Ogre::Entity* loadMesh(Ogre::SceneManager& scene,
                       Ogre::Vector3& ogre_scale,
                       tesseract_gui::EntityContainer& entity_container,
                       const tesseract_geometry::PolygonMesh& mesh,
                       bool is_visual)
{
  if (mesh.getResource() && mesh.getResource()->isFile() && is_visual)
  {
    std::string model_name = "file://" + mesh.getResource()->getFilePath();

    const Eigen::Vector3d& mesh_scale = mesh.getScale();
    ogre_scale = Ogre::Vector3(
        static_cast<float>(mesh_scale.x()), static_cast<float>(mesh_scale.y()), static_cast<float>(mesh_scale.z()));

    try
    {
      rviz::loadMeshFromResource(model_name);
      auto entity = entity_container.addUntrackedEntity(tesseract_gui::EntityContainer::RESOURCE_NS);
      return scene.createEntity(entity.unique_name, model_name);
    }
    catch (Ogre::InvalidParametersException& e)
    {
      ROS_ERROR("Could not convert mesh resource '%s'. It might be an empty mesh: %s", model_name.c_str(), e.what());
    }
    catch (Ogre::Exception& e)
    {
      ROS_ERROR("Could not load model '%s': %s", model_name.c_str(), e.what());
    }

    return nullptr;
  }

  return createEntityForMeshData(scene, entity_container, mesh.getVertices(), mesh.getFaces());
}

Ogre::SceneNode* loadLinkGeometry(Ogre::SceneManager& scene,
                                  tesseract_gui::EntityContainer& entity_container,
                                  const tesseract_geometry::Geometry& geometry,
                                  const Eigen::Vector3d& scale,
                                  const Eigen::Isometry3d& local_pose,
                                  const Ogre::MaterialPtr& material,
                                  bool is_visual)
{
  std::vector<Ogre::Entity*> ogre_entity;
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
      auto entity = entity_container.addUntrackedEntity(tesseract_gui::EntityContainer::RESOURCE_NS);
      ogre_entity.push_back(scene.createEntity(entity.unique_name, "tesseract_sphere.mesh"));
      float diameter = static_cast<float>(sphere.getRadius()) * 2.0f;
      ogre_scale = Ogre::Vector3(diameter, diameter, diameter);
      break;
    }
    case tesseract_geometry::GeometryType::BOX:
    {
      const auto& box = static_cast<const tesseract_geometry::Box&>(geometry);
      auto entity = entity_container.addUntrackedEntity(tesseract_gui::EntityContainer::RESOURCE_NS);
      ogre_entity.push_back(scene.createEntity(entity.unique_name, "tesseract_cube.mesh"));
      ogre_scale =
          Ogre::Vector3(static_cast<float>(box.getX()), static_cast<float>(box.getY()), static_cast<float>(box.getZ()));
      break;
    }
    case tesseract_geometry::GeometryType::CYLINDER:
    {
      const auto& cylinder = static_cast<const tesseract_geometry::Cylinder&>(geometry);
      auto entity = entity_container.addUntrackedEntity(tesseract_gui::EntityContainer::RESOURCE_NS);
      ogre_entity.push_back(scene.createEntity(entity.unique_name, "tesseract_cylinder.mesh"));
      ogre_scale = Ogre::Vector3(static_cast<float>(cylinder.getRadius() * 2),
                                 static_cast<float>(cylinder.getRadius() * 2),
                                 static_cast<float>(cylinder.getLength()));
      break;
    }
    case tesseract_geometry::GeometryType::CONE:
    {
      const auto& cone = static_cast<const tesseract_geometry::Cone&>(geometry);
      auto entity = entity_container.addUntrackedEntity(tesseract_gui::EntityContainer::RESOURCE_NS);
      ogre_entity.push_back(scene.createEntity(entity.unique_name, "tesseract_cone.mesh"));
      ogre_scale = Ogre::Vector3(static_cast<float>(cone.getRadius() * 2),
                                 static_cast<float>(cone.getRadius() * 2),
                                 static_cast<float>(cone.getLength()));
      break;
    }
    case tesseract_geometry::GeometryType::CAPSULE:
    {
      const auto& capsule = static_cast<const tesseract_geometry::Capsule&>(geometry);
      auto entity = entity_container.addUntrackedEntity(tesseract_gui::EntityContainer::RESOURCE_NS);
      ogre_entity.push_back(scene.createEntity(entity.unique_name, "tesseract_capsule.mesh"));
      ogre_scale = Ogre::Vector3(static_cast<float>(capsule.getRadius() * 2),
                                 static_cast<float>(capsule.getRadius() * 2),
                                 static_cast<float>((0.5 * capsule.getLength()) + capsule.getRadius()));
      break;
    }
    case tesseract_geometry::GeometryType::MESH:
    case tesseract_geometry::GeometryType::CONVEX_MESH:
    {
      const auto& mesh = static_cast<const tesseract_geometry::PolygonMesh&>(geometry);
      ogre_entity.push_back(loadMesh(scene, ogre_scale, entity_container, mesh, is_visual));
      break;
    }
    case tesseract_geometry::GeometryType::COMPOUND_MESH:
    {
      const auto& compound_mesh = static_cast<const tesseract_geometry::CompoundMesh&>(geometry);
      if (compound_mesh.getResource() && compound_mesh.getResource()->isFile() && is_visual)
      {
        std::string model_name = "file://" + compound_mesh.getResource()->getFilePath();

        const Eigen::Vector3d& mesh_scale = compound_mesh.getScale();
        ogre_scale = Ogre::Vector3(
            static_cast<float>(mesh_scale.x()), static_cast<float>(mesh_scale.y()), static_cast<float>(mesh_scale.z()));

        try
        {
          rviz::loadMeshFromResource(model_name);
          auto entity = entity_container.addUntrackedEntity(tesseract_gui::EntityContainer::RESOURCE_NS);
          ogre_entity.push_back(scene.createEntity(entity.unique_name, model_name));
        }
        catch (Ogre::InvalidParametersException& e)
        {
          ROS_ERROR(
              "Could not convert mesh resource '%s'. It might be an empty mesh: %s", model_name.c_str(), e.what());
        }
        catch (Ogre::Exception& e)
        {
          ROS_ERROR("Could not load model '%s': %s", model_name.c_str(), e.what());
        }
      }
      else
      {
        for (const auto& mesh : compound_mesh.getMeshes())
          ogre_entity.push_back(
              createEntityForMeshData(scene, entity_container, mesh->getVertices(), mesh->getFaces()));
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
      auto entity = entity_container.addUntrackedEntity(tesseract_gui::EntityContainer::RESOURCE_NS);
      Ogre::SceneNode* offset_node = scene.createSceneNode(entity.unique_name);

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

        offset_node->attachObject(data.point_cloud.get());
        entity_container.addUntrackedUnmanagedObject(tesseract_gui::EntityContainer::VISUAL_NS, data.point_cloud);
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

  if (!ogre_entity.empty())
  {
    auto entity = entity_container.addUntrackedEntity(tesseract_gui::EntityContainer::VISUAL_NS);
    Ogre::SceneNode* offset_node = scene.createSceneNode(entity.unique_name);

    for (auto* obj : ogre_entity)
      offset_node->attachObject(obj);

    offset_node->setScale(ogre_scale);
    offset_node->setPosition(offset_position);
    offset_node->setOrientation(offset_orientation);

    for (auto* obj : ogre_entity)
    {
      for (uint32_t i = 0; i < obj->getNumSubEntities(); ++i)
      {
        // Assign materials only if the submesh does not have one already
        Ogre::SubEntity* sub = obj->getSubEntity(i);
        const std::string& material_name = sub->getMaterialName();

        if (material_name == "BaseWhite" || material_name == "BaseWhiteNoLighting")
        {
          std::string cloned_name = material_name_generator.generate();
          material->clone(cloned_name);
          sub->setMaterialName(cloned_name);
        }
        else
        {
          std::string cloned_name = material_name_generator.generate();
          sub->getMaterial()->clone(cloned_name);
          sub->setMaterialName(cloned_name);
        }
      }
    }

    return offset_node;
  }

  return nullptr;
}

Ogre::MaterialPtr loadLinkMaterial(Ogre::SceneManager& scene,
                                   const tesseract_scene_graph::Link& link,
                                   const std::string& material_name)
{
  if (link.visual.empty() || !link.visual[0]->material)
  {
    return Ogre::MaterialManager::getSingleton().getByName("RVIZ/ShadedRed");
  }

  Ogre::MaterialPtr mat = Ogre::MaterialManager::getSingleton().create(
      material_name_generator.generate(), Ogre::ResourceGroupManager::DEFAULT_RESOURCE_GROUP_NAME);
  mat->getTechnique(0)->setLightingEnabled(true);

  tesseract_scene_graph::Visual::Ptr visual = nullptr;
  std::vector<tesseract_scene_graph::Visual::Ptr>::const_iterator vi;
  for (vi = link.visual.begin(); vi != link.visual.end(); vi++)
  {
    if ((*vi) && (*vi)->material != nullptr && (*vi)->material->getName() == material_name)
    {
      visual = *vi;
      break;
    }
  }

  if (visual == nullptr)
  {
    visual = link.visual[0];  // if link does not have material, use default one
  }

  if (visual->material->texture_filename.empty())
  {
    const Eigen::Vector4f& col = visual->material->color.cast<float>();
    mat->getTechnique(0)->setAmbient(col(0) * 0.5f, col(1) * 0.5f, col(2) * 0.5f);
    mat->getTechnique(0)->setDiffuse(col(0), col(1), col(2), col(3));

    //    material_alpha_ = col(3);
  }
  else
  {
    std::string filename = visual->material->texture_filename;
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

  // Set alpha parameter
  Ogre::ColourValue color = mat->getTechnique(0)->getPass(0)->getDiffuse();
  //  color.a = alpha_ * material_alpha_ * link_alpha;
  color.a = visual->material->color(3);
  mat->setDiffuse(color);

  if (color.a < 0.9998)
  {
    mat->setSceneBlending(Ogre::SBT_TRANSPARENT_ALPHA);
    mat->setDepthWriteEnabled(false);
  }
  else
  {
    mat->setSceneBlending(Ogre::SBT_REPLACE);
    mat->setDepthWriteEnabled(true);
  }

  return mat;
}

Ogre::MaterialPtr loadMaterial(const tesseract_scene_graph::Material::ConstPtr& material)
{
  Ogre::MaterialPtr mat = Ogre::MaterialManager::getSingleton().create(
      material_name_generator.generate(), Ogre::ResourceGroupManager::DEFAULT_RESOURCE_GROUP_NAME);
  mat->getTechnique(0)->setLightingEnabled(true);

  const Eigen::Vector4f& col = material->color.cast<float>();
  mat->getTechnique(0)->setAmbient(col(0) * 0.5f, col(1) * 0.5f, col(2) * 0.5f);
  mat->getTechnique(0)->setDiffuse(col(0), col(1), col(2), col(3));

  // Set alpha parameter
  Ogre::ColourValue color = mat->getTechnique(0)->getPass(0)->getDiffuse();
  color.a = material->color(3);
  mat->setDiffuse(color);

  if (color.a < 0.9998)
  {
    mat->setSceneBlending(Ogre::SBT_TRANSPARENT_ALPHA);
    mat->setDepthWriteEnabled(false);
  }
  else
  {
    mat->setSceneBlending(Ogre::SBT_REPLACE);
    mat->setDepthWriteEnabled(true);
  }

  return mat;
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

void getAABBRecursive(Ogre::AxisAlignedBox& aabb,
                      Ogre::SceneNode& scene_node,
                      Ogre::Matrix4 parent_pose,
                      bool visible_only)
{
  scene_node._updateBounds();
  scene_node._update(false, true);

  Ogre::Matrix4 local_transform(scene_node.getOrientation());
  local_transform.setTrans(scene_node.getPosition());

  Ogre::Matrix4 transform = parent_pose * local_transform;

  for (int i = 0; i < scene_node.numAttachedObjects(); i++)
  {
    Ogre::MovableObject* obj = scene_node.getAttachedObject(i);

    if (!visible_only || obj->isVisible())
    {
      Ogre::AxisAlignedBox box = obj->getBoundingBox();

      // Ogre does not return a valid bounding box for lights.
      if (obj->getMovableType() == Ogre::LightFactory::FACTORY_TYPE_NAME)
      {
        box.setMinimum(Ogre::Vector3(-0.5, -0.5, -0.5));
        box.setMaximum(Ogre::Vector3(0.5, 0.5, 0.5));
      }

      box.transform(transform);
      aabb.merge(box);
    }
  }

  auto num_children = scene_node.numChildren();
  if (num_children == 0)
    return;

  for (unsigned short i = 0; i < num_children; ++i)
  {
    auto* child = dynamic_cast<Ogre::SceneNode*>(scene_node.getChild(i));
    getAABBRecursive(aabb, *child, transform, visible_only);
  }
}

Ogre::AxisAlignedBox getAABB(Ogre::SceneNode& scene_node, bool visible_only)
{
  Ogre::AxisAlignedBox aabb;
  getAABBRecursive(aabb, scene_node, Ogre::Matrix4::IDENTITY, visible_only);

  if (aabb.isFinite())
  {
    Ogre::Vector3 scale(1.15, 1.15, 1.15);
    Ogre::Vector3 center = aabb.getCenter();
    Ogre::Vector3 max = aabb.getMaximum();
    Ogre::Vector3 min = aabb.getMinimum();

    aabb.setMaximum(center + (scale * (max - center)));
    aabb.setMinimum(center + (scale * (min - center)));
  }

  return aabb;
}
}  // namespace tesseract_rviz
