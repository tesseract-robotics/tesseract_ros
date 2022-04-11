#ifndef TESSERACT_RVIZ_ENVIRONMENT_PANEL_H
#define TESSERACT_RVIZ_ENVIRONMENT_PANEL_H

#include <rviz/panel.h>
#include <tesseract_environment/environment.h>

namespace tesseract_rviz
{
struct EnvironmentPanelPrivate;

class EnvironmentPanel : public rviz::Panel
{
  Q_OBJECT

public:
  using Ptr = std::shared_ptr<EnvironmentPanel>;
  using ConstPtr = std::shared_ptr<const EnvironmentPanel>;

  EnvironmentPanel(QWidget* parent = nullptr);
  ~EnvironmentPanel() override;

  void onInitialize() override;

  void load( const rviz::Config& config ) override;
  void save( rviz::Config config ) const override;

private Q_SLOTS:
  void onDisplayModeChanged(int index);
  void onEnvironmentTopicChanged(const QString &text);
  void onJointStateTopicChanged(const QString &text);

  void onEnvironmentSet(const tesseract_environment::Environment& env);
  void onEnvironmentChanged(const tesseract_environment::Environment& env);
  void onEnvironmentCurrentStateChanged(const tesseract_environment::Environment& env);
  void onLinkVisibleChanged(const std::string& link_name, bool visible);
  void onLinkCollisionVisibleChanged(const std::string& link_name, bool visible);
  void onLinkVisualVisibleChanged(const std::string& link_name, bool visible);
  void onSelectedLinksChanged(const std::vector<std::string>& selected_links);
  void onRender();

protected:
  std::unique_ptr<EnvironmentPanelPrivate> data_;
};

}

#endif // TESSERACT_RVIZ_ENVIRONMENT_PANEL_H
