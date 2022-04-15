#include <tesseract_rviz/environment_plugin/environment_visual_properties.h>
#include <tesseract_rviz/environment_plugin/ros_environment_widget.h>

#include <rviz/display.h>
#include <rviz/properties/bool_property.h>
#include <rviz/properties/float_property.h>

namespace tesseract_rviz
{
struct EnvironmentVisualPropertiesPrivate
{
  rviz::Display* parent;
  rviz::Property* main_property;
  tesseract_rviz::ROSEnvironmentWidget* widget;

  rviz::FloatProperty* scene_alpha_property;
  rviz::BoolProperty* scene_visual_visible;
  rviz::BoolProperty* scene_collision_visible;
  rviz::BoolProperty* scene_links_visible;
  rviz::BoolProperty* scene_wirebox_visible;
};

EnvironmentVisualProperties::EnvironmentVisualProperties(rviz::Display* parent, rviz::Property* main_property)
  : data_(std::make_unique<EnvironmentVisualPropertiesPrivate>())
{
  data_->parent = parent;

  data_->main_property = main_property;
  if (data_->main_property == nullptr)
    data_->main_property = data_->parent;

  data_->scene_alpha_property = new rviz::FloatProperty("Alpha",
                                                        1.0f,
                                                        "Specifies the alpha for the links with geometry",
                                                        data_->main_property,
                                                        SLOT(onSceneAlphaChanged()),
                                                        this);
  data_->scene_alpha_property->setMin(0.0);
  data_->scene_alpha_property->setMax(1.0);

  data_->scene_visual_visible = new rviz::BoolProperty("Show Visual",
                                                       true,
                                                       "Whether to display the visual representation of the "
                                                       "environment.",
                                                       data_->main_property,
                                                       SLOT(onSceneVisualVisibleChanged()),
                                                       this);

  data_->scene_collision_visible = new rviz::BoolProperty("Show Collision",
                                                          false,
                                                          "Whether to display the collision representation of the "
                                                          "environment.",
                                                          data_->main_property,
                                                          SLOT(onSceneCollisionVisibleChanged()),
                                                          this);

  data_->scene_wirebox_visible = new rviz::BoolProperty("Show Wire Box",
                                                        false,
                                                        "Whether to display the wire box representation of the "
                                                        "environment.",
                                                        data_->main_property,
                                                        SLOT(onSceneWireBoxVisibleChanged()),
                                                        this);

  data_->scene_links_visible = new rviz::BoolProperty("Show All Links",
                                                      true,
                                                      "Toggle all links visibility on or off.",
                                                      data_->main_property,
                                                      SLOT(onSceneLinkVisibleChanged()),
                                                      this);
}

EnvironmentVisualProperties::~EnvironmentVisualProperties() = default;

void EnvironmentVisualProperties::onInitialize(ROSEnvironmentWidget* widget) { data_->widget = widget; }

void EnvironmentVisualProperties::load(const rviz::Config& config)
{
  float alpha{ 0 };
  if (config.mapGetFloat("tesseract::SceneAlpha", &alpha))
    data_->scene_alpha_property->setValue(alpha);

  bool link_visible{ true };
  if (config.mapGetBool("tesseract::SceneLinkVisible", &link_visible))
    data_->scene_links_visible->setBool(link_visible);

  bool visual_visible{ true };
  if (config.mapGetBool("tesseract::SceneVisualVisible", &visual_visible))
    data_->scene_visual_visible->setBool(visual_visible);

  bool collision_visible{ true };
  if (config.mapGetBool("tesseract::SceneCollisionVisible", &collision_visible))
    data_->scene_collision_visible->setBool(collision_visible);

  bool wirebox_visible{ true };
  if (config.mapGetBool("tesseract::SceneWireBoxVisible", &wirebox_visible))
    data_->scene_wirebox_visible->setBool(wirebox_visible);
}

void EnvironmentVisualProperties::save(rviz::Config config) const
{
  config.mapSetValue("tesseract::SceneAlpha", data_->scene_alpha_property->getFloat());
  config.mapSetValue("tesseract::SceneLinkVisible", data_->scene_links_visible->getBool());
  config.mapSetValue("tesseract::SceneVisualVisible", data_->scene_visual_visible->getBool());
  config.mapSetValue("tesseract::SceneCollisionVisible", data_->scene_collision_visible->getBool());
  config.mapSetValue("tesseract::SceneWireBoxVisible", data_->scene_wirebox_visible->getBool());
}

void EnvironmentVisualProperties::onSceneAlphaChanged() {}

void EnvironmentVisualProperties::onSceneVisualVisibleChanged()
{
  if (data_->widget == nullptr)
    return;

  std::vector<std::string> link_names = data_->widget->environment().getLinkNames();
  for (const auto& link_name : link_names)
    data_->widget->onLinkVisualVisibleChanged(link_name, data_->scene_visual_visible->getBool());
}

void EnvironmentVisualProperties::onSceneCollisionVisibleChanged()
{
  if (data_->widget == nullptr)
    return;

  std::vector<std::string> link_names = data_->widget->environment().getLinkNames();
  for (const auto& link_name : link_names)
    data_->widget->onLinkCollisionVisibleChanged(link_name, data_->scene_collision_visible->getBool());
}

void EnvironmentVisualProperties::onSceneWireBoxVisibleChanged()
{
  if (data_->widget == nullptr)
    return;

  std::vector<std::string> link_names;
  if (data_->scene_wirebox_visible->getBool())
    link_names = data_->widget->environment().getLinkNames();

  data_->widget->onSelectedLinksChanged(link_names);
}

void EnvironmentVisualProperties::onSceneLinkVisibleChanged()
{
  if (data_->widget == nullptr)
    return;

  std::vector<std::string> link_names = data_->widget->environment().getLinkNames();
  for (const auto& link_name : link_names)
    data_->widget->onLinkVisibleChanged(link_name, data_->scene_links_visible->getBool());

  if (data_->scene_links_visible->getBool())
  {
    onSceneVisualVisibleChanged();
    onSceneCollisionVisibleChanged();
    onSceneWireBoxVisibleChanged();
  }
}

}  // namespace tesseract_rviz
