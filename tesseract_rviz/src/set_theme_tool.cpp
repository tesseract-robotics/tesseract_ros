#include <tesseract_rviz/set_theme_tool.h>

#include <tesseract_qt/common/icon_utils.h>
#include <tesseract_qt/common/theme_utils.h>

#include <rviz/display_context.h>
#include <rviz/window_manager_interface.h>

#include <QApplication>
#include <QMainWindow>
#include <QToolBar>
#include <QMenu>
#include <QToolButton>
#include <QSettings>

namespace tesseract_rviz
{
struct SetThemeToolPrivate
{
  int theme_index{ 0 };

  QToolBar* toolbar;
  QMenu* menu;
  QToolButton* tool_button;

  QAction* default_theme{ nullptr };
  QAction* dark_theme{ nullptr };
  QAction* light_theme{ nullptr };
};

SetThemeTool::SetThemeTool() : data_(std::make_unique<SetThemeToolPrivate>()) {}

SetThemeTool::~SetThemeTool()
{
  QSettings ms;
  ms.beginGroup("SetThemeTool");
  ms.setValue("theme", data_->theme_index);
  ms.endGroup();
}

bool SetThemeTool::isInitialized() const { return (data_->toolbar != nullptr); }

void SetThemeTool::initialized(rviz::DisplayContext* context)
{
  if (!isInitialized())
  {
    auto* main_window = dynamic_cast<QMainWindow*>(context->getWindowManager());
    data_->toolbar = main_window->findChild<QToolBar*>("Tools");
    if (data_->toolbar != nullptr)
    {
      data_->toolbar->addSeparator();

      data_->menu = new QMenu(data_->toolbar);  // NOLINT
      data_->default_theme = data_->menu->addAction("Default", this, SLOT(onThemeDefaultSelected()));
      data_->dark_theme = data_->menu->addAction("Dark", this, SLOT(onThemeDarkSelected()));
      data_->light_theme = data_->menu->addAction("Light", this, SLOT(onThemeLightSelected()));

      data_->tool_button = new QToolButton();  // NOLINT
      data_->tool_button->setMenu(data_->menu);
      data_->tool_button->setToolButtonStyle(Qt::ToolButtonTextBesideIcon);
      data_->tool_button->setPopupMode(QToolButton::InstantPopup);
      data_->tool_button->setToolTip("Set the application theme.");
      data_->tool_button->setIcon(tesseract_gui::icons::getTesseractIcon());
      data_->toolbar->addWidget(data_->tool_button);

      QSettings ms;
      ms.beginGroup("SetThemeTool");
      data_->theme_index = ms.value("theme", 0).toInt();
      ms.endGroup();

      if (data_->theme_index == 0)
        onThemeDefaultSelected();
      else if (data_->theme_index == 1)
        onThemeDarkSelected();
      else if (data_->theme_index == 2)
        onThemeLightSelected();
    }
  }
}

std::shared_ptr<SetThemeTool> SetThemeTool::instance()
{
  static std::shared_ptr<SetThemeTool> singleton = nullptr;
  if (singleton == nullptr)
    singleton = std::make_shared<SetThemeTool>();

  return singleton;
}

void SetThemeTool::onThemeDefaultSelected()
{
  qApp->setStyleSheet("");
  data_->tool_button->setText("Theme (Default)");
  data_->theme_index = 0;
}

void SetThemeTool::onThemeDarkSelected()
{
  qApp->setStyleSheet(tesseract_gui::themes::getDarkTheme());
  data_->tool_button->setText("Theme (Dark)");
  data_->theme_index = 1;
}

void SetThemeTool::onThemeLightSelected()
{
  qApp->setStyleSheet(tesseract_gui::themes::getLightTheme());
  data_->tool_button->setText("Theme (Light)");
  data_->theme_index = 2;
}

}  // namespace tesseract_rviz
