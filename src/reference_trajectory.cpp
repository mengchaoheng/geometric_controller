// Copyright 2026 Chaoheng Meng
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include "geometric_controller/reference_trajectory.hpp"

#include <algorithm>
#include <cmath>
#include <cctype>
#include <limits>

namespace geometric_controller
{
namespace
{

constexpr double kPi = 3.14159265358979323846;

std::string normalizeName(std::string value)
{
  std::transform(value.begin(), value.end(), value.begin(), [](unsigned char c) {
      return static_cast<char>(std::tolower(c));
  });
  std::replace(value.begin(), value.end(), '-', '_');
  return value;
}

double positiveOr(double value, double fallback)
{
  if (!std::isfinite(value) || value <= 0.0) {
    return fallback;
  }
  return value;
}

double wrapPi(double value)
{
  while (value > kPi) {
    value -= 2.0 * kPi;
  }
  while (value < -kPi) {
    value += 2.0 * kPi;
  }
  return value;
}

}  // namespace

ReferenceTrajectory::ReferenceTrajectory(const TrajectoryParameters & parameters)
: parameters_(parameters)
{
}

void ReferenceTrajectory::setParameters(const TrajectoryParameters & parameters)
{
  parameters_ = parameters;
  parameters_.traj_name = normalizeTrajectoryType(parameters_.traj_name);
  parameters_.path_preview_cycles = positiveOr(parameters_.path_preview_cycles, 1.0);
}

const TrajectoryParameters & ReferenceTrajectory::parameters() const
{
  return parameters_;
}

TrajectorySample ReferenceTrajectory::sample(double time_s) const
{
  const auto type = normalizeTrajectoryType(parameters_.traj_name);
  const auto theta = thetaState(time_s);

  if (type == "figure8_vertical") {
    return figureEightVertical(time_s, theta);
  }
  if (type == "helix_flip") {
    return helixFlip(time_s, theta);
  }
  if (type == "helix_flip_y") {
    return helixFlipY(time_s, theta);
  }
  if (type == "flip_loop_sine") {
    return flipLoopSine(time_s, theta);
  }
  if (type == "fast_circle") {
    return fastCircle(time_s, theta);
  }
  return figureEightHorizontal(time_s, theta);
}

double ReferenceTrajectory::theta(double time_s) const
{
  return thetaState(time_s).theta;
}

double ReferenceTrajectory::nominalPeriod() const
{
  const double omega = positiveOr(parameters_.omega_value, 1.0);
  return 2.0 * kPi / omega;
}

double ReferenceTrajectory::previewDuration() const
{
  const auto type = normalizeTrajectoryType(parameters_.traj_name);
  double duration = nominalPeriod();
  if (type == "helix_flip" || type == "helix_flip_y" || type == "flip_loop_sine") {
    duration *= positiveOr(parameters_.path_preview_cycles, 10.0);
  }
  return duration;
}

ReferenceTrajectory::ThetaState ReferenceTrajectory::thetaState(double time_s) const
{
  const double t = std::max(0.0, time_s);
  const double omega_value = positiveOr(parameters_.omega_value, 1.0);

  ThetaState out;
  out.theta = theta0ForType() + parameters_.phase_shift + omega_value * t;
  out.theta_dot = omega_value;
  return out;
}

double ReferenceTrajectory::theta0ForType() const
{
  const auto type = normalizeTrajectoryType(parameters_.traj_name);
  if (type == "figure8_vertical") {
    return parameters_.figure8_vertical_theta0;
  }
  if (type == "helix_flip") {
    return parameters_.helix_flip_theta0;
  }
  if (type == "helix_flip_y") {
    return parameters_.helix_flip_y_theta0;
  }
  if (type == "flip_loop_sine") {
    return parameters_.flip_loop_sine_theta0;
  }
  if (type == "fast_circle") {
    return parameters_.fast_circle_theta0;
  }
  return parameters_.figure8_horizontal_theta0;
}

ReferenceTrajectory::ScalarState ReferenceTrajectory::trigDerivatives(
  double amplitude, double theta, double theta_dot, double theta_ddot, double theta_3,
  double theta_4, bool sine)
{
  const double s = std::sin(theta);
  const double c = std::cos(theta);
  ScalarState out;

  if (sine) {
    out.p = amplitude * s;
    out.v = amplitude * c * theta_dot;
    out.a = amplitude * (c * theta_ddot - s * theta_dot * theta_dot);
    out.j = amplitude * (c * theta_3 - 3.0 * s * theta_dot * theta_ddot -
      c * std::pow(theta_dot, 3.0));
    out.s = amplitude * (c * theta_4 - 4.0 * s * theta_dot * theta_3 -
      3.0 * s * theta_ddot * theta_ddot -
      6.0 * c * theta_dot * theta_dot * theta_ddot +
      s * std::pow(theta_dot, 4.0));
  } else {
    out.p = amplitude * c;
    out.v = -amplitude * s * theta_dot;
    out.a = amplitude * (-c * theta_dot * theta_dot - s * theta_ddot);
    out.j = amplitude * (s * std::pow(theta_dot, 3.0) -
      3.0 * c * theta_dot * theta_ddot - s * theta_3);
    out.s = amplitude * (c * std::pow(theta_dot, 4.0) +
      6.0 * s * theta_dot * theta_dot * theta_ddot -
      3.0 * c * theta_ddot * theta_ddot -
      4.0 * c * theta_dot * theta_3 - s * theta_4);
  }

  return out;
}

TrajectorySample ReferenceTrajectory::figureEightHorizontal(
  double /*time_s*/, const ThetaState & theta) const
{
  const auto x = trigDerivatives(
    parameters_.figure8_horizontal_Ax, theta.theta, theta.theta_dot, theta.theta_ddot,
    theta.theta_3, theta.theta_4, true);
  const auto y = trigDerivatives(
    parameters_.figure8_horizontal_Ay, 2.0 * theta.theta, 2.0 * theta.theta_dot,
    2.0 * theta.theta_ddot, 2.0 * theta.theta_3, 2.0 * theta.theta_4, true);

  TrajectorySample sample;
  sample.position = {parameters_.origin_x + x.p, parameters_.origin_y + y.p,
    -parameters_.figure8_horizontal_Hc};
  sample.velocity = {x.v, y.v, 0.0};
  sample.acceleration = {x.a, y.a, 0.0};
  sample.jerk = {x.j, y.j, 0.0};
  sample.snap = {x.s, y.s, 0.0};
  applyYaw(sample);
  return sample;
}

TrajectorySample ReferenceTrajectory::figureEightVertical(
  double /*time_s*/, const ThetaState & theta) const
{
  const auto y = trigDerivatives(
    parameters_.figure8_vertical_Ay, theta.theta, theta.theta_dot, theta.theta_ddot,
    theta.theta_3, theta.theta_4, true);
  const auto z = trigDerivatives(
    -parameters_.figure8_vertical_Az, 2.0 * theta.theta, 2.0 * theta.theta_dot,
    2.0 * theta.theta_ddot, 2.0 * theta.theta_3, 2.0 * theta.theta_4, true);

  TrajectorySample sample;
  sample.position = {parameters_.origin_x, parameters_.origin_y + y.p,
    -parameters_.figure8_vertical_Hc + z.p};
  sample.velocity = {0.0, y.v, z.v};
  sample.acceleration = {0.0, y.a, z.a};
  sample.jerk = {0.0, y.j, z.j};
  sample.snap = {0.0, y.s, z.s};
  applyYaw(sample);
  return sample;
}

TrajectorySample ReferenceTrajectory::helixFlip(double time_s, const ThetaState & theta) const
{
  const auto y = trigDerivatives(
    parameters_.helix_flip_Ay, theta.theta, theta.theta_dot, theta.theta_ddot,
    theta.theta_3, theta.theta_4, true);
  const auto z = trigDerivatives(
    parameters_.helix_flip_Az, theta.theta, theta.theta_dot, theta.theta_ddot,
    theta.theta_3, theta.theta_4, false);
  const double t = std::max(0.0, time_s);

  TrajectorySample sample;
  sample.position = {parameters_.origin_x + parameters_.helix_flip_Vx * t,
    parameters_.origin_y + y.p, -parameters_.helix_flip_Hc + z.p};
  sample.velocity = {parameters_.helix_flip_Vx, y.v, z.v};
  sample.acceleration = {0.0, y.a, z.a};
  sample.jerk = {0.0, y.j, z.j};
  sample.snap = {0.0, y.s, z.s};
  applyYaw(sample);
  return sample;
}

TrajectorySample ReferenceTrajectory::helixFlipY(double time_s, const ThetaState & theta) const
{
  const auto x = trigDerivatives(
    parameters_.helix_flip_y_Ax, theta.theta, theta.theta_dot, theta.theta_ddot,
    theta.theta_3, theta.theta_4, true);
  const auto z = trigDerivatives(
    parameters_.helix_flip_y_Az, theta.theta, theta.theta_dot, theta.theta_ddot,
    theta.theta_3, theta.theta_4, false);
  const double t = std::max(0.0, time_s);

  TrajectorySample sample;
  sample.position = {parameters_.origin_x + x.p,
    parameters_.origin_y + parameters_.helix_flip_y_Vy * t,
    -parameters_.helix_flip_y_Hc + z.p};
  sample.velocity = {x.v, parameters_.helix_flip_y_Vy, z.v};
  sample.acceleration = {x.a, 0.0, z.a};
  sample.jerk = {x.j, 0.0, z.j};
  sample.snap = {x.s, 0.0, z.s};
  applyYaw(sample);
  return sample;
}

TrajectorySample ReferenceTrajectory::flipLoopSine(double time_s, const ThetaState & theta) const
{
  const auto y = trigDerivatives(
    parameters_.flip_loop_sine_Ay, theta.theta, theta.theta_dot, theta.theta_ddot,
    theta.theta_3, theta.theta_4, true);
  const auto z = trigDerivatives(
    parameters_.flip_loop_sine_Az, theta.theta, theta.theta_dot, theta.theta_ddot,
    theta.theta_3, theta.theta_4, false);
  const double t = std::max(0.0, time_s);

  TrajectorySample sample;
  sample.position = {parameters_.origin_x + parameters_.flip_loop_sine_Vx * t,
    parameters_.origin_y + y.p, -parameters_.flip_loop_sine_Hc + z.p};
  sample.velocity = {parameters_.flip_loop_sine_Vx, y.v, z.v};
  sample.acceleration = {0.0, y.a, z.a};
  sample.jerk = {0.0, y.j, z.j};
  sample.snap = {0.0, y.s, z.s};
  applyYaw(sample);
  return sample;
}

TrajectorySample ReferenceTrajectory::fastCircle(double /*time_s*/, const ThetaState & theta) const
{
  const auto x = trigDerivatives(
    parameters_.fast_circle_Ax, theta.theta, theta.theta_dot, theta.theta_ddot,
    theta.theta_3, theta.theta_4, false);
  const auto y = trigDerivatives(
    parameters_.fast_circle_Ay, theta.theta, theta.theta_dot, theta.theta_ddot,
    theta.theta_3, theta.theta_4, true);

  TrajectorySample sample;
  sample.position = {parameters_.origin_x + x.p, parameters_.origin_y + y.p,
    -parameters_.fast_circle_Hc};
  sample.velocity = {x.v, y.v, 0.0};
  sample.acceleration = {x.a, y.a, 0.0};
  sample.jerk = {x.j, y.j, 0.0};
  sample.snap = {x.s, y.s, 0.0};
  applyYaw(sample);
  return sample;
}

void ReferenceTrajectory::applyYaw(TrajectorySample & sample) const
{
  if (parameters_.trajectory_yaw_lock) {
    sample.yaw = wrapPi(parameters_.trajectory_yaw_fixed);
    sample.yaw_rate = 0.0;
    sample.yaw_acceleration = 0.0;
    return;
  }

  const auto type = normalizeTrajectoryType(parameters_.traj_name);
  if (type == "figure8_vertical" || type == "helix_flip" || type == "helix_flip_y" ||
    type == "flip_loop_sine")
  {
    sample.yaw = 0.0;
    sample.yaw_rate = 0.0;
    sample.yaw_acceleration = 0.0;
    return;
  }

  const double speed2 = sample.velocity[0] * sample.velocity[0] +
    sample.velocity[1] * sample.velocity[1];
  if (speed2 < 1e-10) {
    sample.yaw = 0.0;
    sample.yaw_rate = 0.0;
    sample.yaw_acceleration = 0.0;
    return;
  }

  const double num = sample.velocity[0] * sample.acceleration[1] -
    sample.velocity[1] * sample.acceleration[0];
  const double den_dot = 2.0 * (
    sample.velocity[0] * sample.acceleration[0] +
    sample.velocity[1] * sample.acceleration[1]);
  const double num_dot = sample.velocity[0] * sample.jerk[1] -
    sample.velocity[1] * sample.jerk[0];

  sample.yaw = wrapPi(std::atan2(sample.velocity[1], sample.velocity[0]));
  sample.yaw_rate = num / speed2;
  sample.yaw_acceleration = (num_dot * speed2 - num * den_dot) / (speed2 * speed2);
}

std::string normalizeTrajectoryType(const std::string & type)
{
  const auto normalized = normalizeName(type);
  if (normalized == "figure8" || normalized == "figure_8" || normalized == "lemniscate" ||
    normalized == "lamniscate")
  {
    return "figure8_horizontal";
  }
  if (normalized == "circle") {
    return "fast_circle";
  }
  if (normalized == "figure8_horizontal" || normalized == "figure8_vertical" ||
    normalized == "helix_flip" || normalized == "helix_flip_y" ||
    normalized == "flip_loop_sine" || normalized == "fast_circle")
  {
    return normalized;
  }
  return "figure8_horizontal";
}

std::string trajectoryTypeNameFromId(int type_id)
{
  switch (type_id) {
    case 2:
      return "figure8_vertical";
    case 3:
      return "helix_flip";
    case 4:
      return "helix_flip_y";
    case 5:
      return "flip_loop_sine";
    case 6:
      return "fast_circle";
    case 1:
    default:
      return "figure8_horizontal";
  }
}

int trajectoryTypeIdFromName(const std::string & type)
{
  const auto normalized = normalizeTrajectoryType(type);
  if (normalized == "figure8_vertical") {
    return 2;
  }
  if (normalized == "helix_flip") {
    return 3;
  }
  if (normalized == "helix_flip_y") {
    return 4;
  }
  if (normalized == "flip_loop_sine") {
    return 5;
  }
  if (normalized == "fast_circle") {
    return 6;
  }
  return 1;
}

bool isSupportedTrajectoryType(const std::string & type)
{
  const auto normalized = normalizeName(type);
  return normalized == "figure8" || normalized == "figure_8" || normalized == "lemniscate" ||
         normalized == "lamniscate" || normalized == "circle" ||
         normalized == "figure8_horizontal" || normalized == "figure8_vertical" ||
         normalized == "helix_flip" || normalized == "helix_flip_y" ||
         normalized == "flip_loop_sine" || normalized == "fast_circle";
}

std::vector<std::string> supportedTrajectoryTypes()
{
  return {
    "figure8_horizontal",
    "figure8_vertical",
    "helix_flip",
    "helix_flip_y",
    "flip_loop_sine",
    "fast_circle",
  };
}

}  // namespace geometric_controller
