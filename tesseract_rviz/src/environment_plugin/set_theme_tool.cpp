#include <tesseract_rviz/environment_plugin/set_theme_tool.h>
#include <tesseract_widgets/common/theme_utils.h>
#include <tesseract_widgets/common/icon_utils.h>

#include <rviz/properties/enum_property.h>

#include <QString>
#include <QApplication>

namespace tesseract_rviz
{
struct SetThemeToolImpl
{
  QString theme{ "default" };
  rviz::EnumProperty* theme_property;
};

SetThemeTool::SetThemeTool() : data_(std::make_unique<SetThemeToolImpl>())
{
  data_->theme_property = new rviz::EnumProperty("Theme",
                                                 "default",
                                                 "Set the rviz application theme (Dark, Light or Default)",
                                                 nullptr,
                                                 SLOT(onThemeChanged()),
                                                 this);

  data_->theme_property->addOptionStd("Default", 0);
  data_->theme_property->addOptionStd("Light", 1);
  data_->theme_property->addOptionStd("Dark", 2);

  getPropertyContainer()->addChild(data_->theme_property);
}
SetThemeTool::~SetThemeTool() = default;

void SetThemeTool::onInitialize()
{
  setIcon(tesseract_gui::icons::getTesseractIcon());
  onThemeChanged();
}

void SetThemeTool::activate() {}

void SetThemeTool::deactivate() {}

void SetThemeTool::load(const rviz::Config& config)
{
  rviz::Tool::load(config);

  QString theme;
  if (config.mapGetString("tesseract::SetThemeTool", &theme))
    data_->theme_property->setString(theme);
}

void SetThemeTool::save(rviz::Config config) const
{
  config.mapSetValue("tesseract::SetThemeTool", data_->theme_property->getString());
  rviz::Tool::save(config);
}

void SetThemeTool::onThemeChanged()
{
  if (data_->theme_property->getOptionInt() == 0)
  {
    qApp->setStyleSheet("");
    setName("Theme (Default)");
  }
  else if (data_->theme_property->getOptionInt() == 1)
  {
    qApp->setStyleSheet(tesseract_gui::themes::getLightTheme());
    setName("Theme (Light)");
  }
  else if (data_->theme_property->getOptionInt() == 2)
  {
    qApp->setStyleSheet(tesseract_gui::themes::getDarkTheme());
    setName("Theme (Dark)");
  }
}

}  // namespace tesseract_rviz
