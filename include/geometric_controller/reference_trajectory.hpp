#ifndef GEOMETRIC_CONTROLLER_REFERENCE_TRAJECTORY_HPP_
#define GEOMETRIC_CONTROLLER_REFERENCE_TRAJECTORY_HPP_

#include <array>
#include <string>
#include <vector>

namespace geometric_controller
{

using Vector3 = std::array<double, 3>;

struct TrajectoryParameters
{
  std::string traj_name{"figure8_horizontal"};
  bool trajectory_yaw_lock{false};
  double trajectory_yaw_fixed{0.0};
  double origin_x{0.0};
  double origin_y{0.0};
  double omega_value{0.5};
  double phase_shift{0.0};
  double path_preview_cycles{10.0};

  double figure8_horizontal_Ax{2.0};
  double figure8_horizontal_Ay{2.0};
  double figure8_horizontal_Hc{3.0};
  double figure8_horizontal_theta0{0.0};

  double figure8_vertical_Ay{2.0};
  double figure8_vertical_Az{2.0};
  double figure8_vertical_Hc{3.0};
  double figure8_vertical_theta0{-0.7853981633974483};

  double helix_flip_Ay{2.0};
  double helix_flip_Az{2.0};
  double helix_flip_Hc{3.0};
  double helix_flip_Vx{0.30};
  double helix_flip_theta0{0.0};

  double helix_flip_y_Ax{2.0};
  double helix_flip_y_Az{2.0};
  double helix_flip_y_Hc{3.0};
  double helix_flip_y_Vy{0.30};
  double helix_flip_y_theta0{0.0};

  double flip_loop_sine_Ay{2.0};
  double flip_loop_sine_Az{2.0};
  double flip_loop_sine_Hc{3.0};
  double flip_loop_sine_Vx{0.0};
  double flip_loop_sine_theta0{0.0};

  double fast_circle_Ax{3.0};
  double fast_circle_Ay{3.0};
  double fast_circle_Hc{3.0};
  double fast_circle_theta0{0.0};
};

struct TrajectorySample
{
  Vector3 position{0.0, 0.0, 0.0};
  Vector3 velocity{0.0, 0.0, 0.0};
  Vector3 acceleration{0.0, 0.0, 0.0};
  Vector3 jerk{0.0, 0.0, 0.0};
  Vector3 snap{0.0, 0.0, 0.0};
  double yaw{0.0};
  double yaw_rate{0.0};
  double yaw_acceleration{0.0};
};

class ReferenceTrajectory
{
public:
  ReferenceTrajectory() = default;
  explicit ReferenceTrajectory(const TrajectoryParameters & parameters);

  void setParameters(const TrajectoryParameters & parameters);
  const TrajectoryParameters & parameters() const;
  TrajectorySample sample(double time_s) const;
  double theta(double time_s) const;
  double nominalPeriod() const;
  double previewDuration() const;

private:
  struct ThetaState
  {
    double theta{0.0};
    double theta_dot{0.0};
    double theta_ddot{0.0};
    double theta_3{0.0};
    double theta_4{0.0};
  };

  struct ScalarState
  {
    double p{0.0};
    double v{0.0};
    double a{0.0};
    double j{0.0};
    double s{0.0};
  };

  TrajectorySample figureEightHorizontal(double time_s, const ThetaState & theta) const;
  TrajectorySample figureEightVertical(double time_s, const ThetaState & theta) const;
  TrajectorySample helixFlip(double time_s, const ThetaState & theta) const;
  TrajectorySample helixFlipY(double time_s, const ThetaState & theta) const;
  TrajectorySample flipLoopSine(double time_s, const ThetaState & theta) const;
  TrajectorySample fastCircle(double time_s, const ThetaState & theta) const;
  ThetaState thetaState(double time_s) const;
  double theta0ForType() const;
  static ScalarState trigDerivatives(
    double amplitude, double theta, double theta_dot, double theta_ddot, double theta_3,
    double theta_4, bool sine);
  void applyYaw(TrajectorySample & sample) const;

  TrajectoryParameters parameters_;
};

std::string normalizeTrajectoryType(const std::string & type);
std::string trajectoryTypeNameFromId(int type_id);
int trajectoryTypeIdFromName(const std::string & type);
bool isSupportedTrajectoryType(const std::string & type);
std::vector<std::string> supportedTrajectoryTypes();

}  // namespace geometric_controller

#endif  // GEOMETRIC_CONTROLLER_REFERENCE_TRAJECTORY_HPP_
