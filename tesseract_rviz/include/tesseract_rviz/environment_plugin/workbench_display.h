#ifndef TESSERACT_RVIZ_WORKBENCH_DISPLAY_H
#define TESSERACT_RVIZ_WORKBENCH_DISPLAY_H

#include <rviz/display.h>
#include <tesseract_environment/environment.h>
#include <tesseract_widgets/common/entity_container.h>

namespace tesseract_common
{
class JointTrajectorySet;
class JointState;
}  // namespace tesseract_common

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

private Q_SLOTS:
  void onConfigureJointTrajectorySet(const QString& uuid,
                                     const tesseract_common::JointTrajectorySet& joint_trajectory_set);
  void onJointTrajectorySetRemoved(const QString& uuid);

  void onJointTrajectorySetState(const tesseract_common::JointState& state);

protected:
  // overrides from Display
  void onInitialize() override;

  std::unique_ptr<WorkbenchDisplayPrivate> data_;
};
}  // namespace tesseract_rviz

#endif  // TESSERACT_RVIZ_WORKBENCH_DISPLAY_H
