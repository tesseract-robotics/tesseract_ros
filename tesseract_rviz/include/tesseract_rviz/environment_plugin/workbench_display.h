#ifndef TESSERACT_RVIZ_WORKBENCH_DISPLAY_H
#define TESSERACT_RVIZ_WORKBENCH_DISPLAY_H

#include <rviz/display.h>

namespace tesseract_rviz
{
struct WorkbenchDisplayPrivate;

class WorkbenchDisplay : public rviz::Display
{
  Q_OBJECT
public:
  WorkbenchDisplay();
  ~WorkbenchDisplay() override;

  void update(float wall_dt, float ros_dt) override;
  void reset() override;

  void load(const rviz::Config& config) override;
  void save(rviz::Config config) const override;

public Q_SLOTS:
  void onEnableChanged() override;

protected:
  // overrides from Display
  void onInitialize() override;
  void onEnable() override;
  void onDisable() override;

  std::unique_ptr<WorkbenchDisplayPrivate> data_;
};
}  // namespace tesseract_rviz

#endif  // TESSERACT_RVIZ_WORKBENCH_DISPLAY_H
