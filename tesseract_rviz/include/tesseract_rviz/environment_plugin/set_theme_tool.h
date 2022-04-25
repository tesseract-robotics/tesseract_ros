#ifndef TESSERACT_RVIZ_SET_THEME_TOOL_H
#define TESSERACT_RVIZ_SET_THEME_TOOL_H

#include <rviz/tool.h>

namespace tesseract_rviz
{
struct SetThemeToolImpl;

class SetThemeTool : public rviz::Tool
{
  Q_OBJECT
public:
  SetThemeTool();
  ~SetThemeTool() override;

  void onInitialize() override;

  void activate() override;
  void deactivate() override;

  void load(const rviz::Config& config) override;
  void save(rviz::Config config) const override;

private Q_SLOTS:
  void onThemeChanged();

protected:
  std::unique_ptr<SetThemeToolImpl> data_;
};
}  // namespace tesseract_rviz

#endif  // TESSERACT_RVIZ_SET_THEME_TOOL_H
