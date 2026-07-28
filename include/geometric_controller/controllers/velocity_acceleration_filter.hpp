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

#ifndef GEOMETRIC_CONTROLLER__CONTROLLERS__VELOCITY_ACCELERATION_FILTER_HPP_
#define GEOMETRIC_CONTROLLER__CONTROLLERS__VELOCITY_ACCELERATION_FILTER_HPP_

#include <Eigen/Dense>

#include <algorithm>
#include <cmath>
#include <limits>

namespace geometric_controller
{

struct VelocityAccelerationFilterParams
{
  double velocity_lpf_hz{0.0};
  double velocity_notch_hz{0.0};
  double velocity_notch_bandwidth_hz{5.0};
  double derivative_lpf_hz{5.0};
};

// Reproduces the mc_pos_control acceleration-feedback chain:
// velocity notch -> velocity alpha LPF -> numerical derivative ->
// derivative alpha LPF. All vectors remain in the input NED frame.
class VelocityAccelerationFilter
{
public:
  void reset()
  {
    initialized_ = false;
    notch_initialized_ = false;
    velocity_state_.setZero();
    derivative_state_.setZero();
    notch_x1_.setZero();
    notch_x2_.setZero();
    notch_y1_.setZero();
    notch_y2_.setZero();
  }

  Eigen::Vector3d update(
    const Eigen::Vector3d & velocity_ned, double dt,
    const VelocityAccelerationFilterParams & params)
  {
    if (!velocity_ned.allFinite() || !std::isfinite(dt) || dt <= 0.0) {
      reset();
      return Eigen::Vector3d::Constant(
        std::numeric_limits<double>::quiet_NaN());
    }

    if (!initialized_) {
      initialized_ = true;
      velocity_state_ = velocity_ned;
      derivative_state_.setZero();
      resetNotch(velocity_ned, dt, params);
      return derivative_state_;
    }

    const Eigen::Vector3d notched = applyNotch(velocity_ned, dt, params);
    const Eigen::Vector3d velocity_previous = velocity_state_;
    velocity_state_ = applyAlpha(
      velocity_state_, notched, dt, params.velocity_lpf_hz);
    const Eigen::Vector3d raw_derivative =
      (velocity_state_ - velocity_previous) / dt;
    derivative_state_ = applyAlpha(
      derivative_state_, raw_derivative, dt, params.derivative_lpf_hz);
    return derivative_state_;
  }

  bool initialized() const {return initialized_;}

private:
  static Eigen::Vector3d applyAlpha(
    const Eigen::Vector3d & state, const Eigen::Vector3d & input,
    double dt, double cutoff_hz)
  {
    if (cutoff_hz <= 0.0) {
      return input;
    }
    const double time_constant = 1.0 / (2.0 * M_PI * cutoff_hz);
    const double alpha = dt / (time_constant + dt);
    return state + alpha * (input - state);
  }

  void resetNotch(
    const Eigen::Vector3d & input, double dt,
    const VelocityAccelerationFilterParams & params)
  {
    configureNotch(dt, params);
    notch_x1_ = input;
    notch_x2_ = input;
    const double dc_gain =
      (notch_b0_ + notch_b1_ + notch_b2_) /
      (1.0 + notch_a1_ + notch_a2_);
    notch_y1_ = dc_gain * input;
    notch_y2_ = dc_gain * input;
    notch_initialized_ = true;
  }

  void configureNotch(
    double dt, const VelocityAccelerationFilterParams & params)
  {
    const double sample_rate_hz = 1.0 / dt;
    if (params.velocity_notch_hz <= 0.0 ||
      params.velocity_notch_bandwidth_hz <= 0.0 ||
      params.velocity_notch_hz >= 0.5 * sample_rate_hz)
    {
      notch_b0_ = 1.0;
      notch_b1_ = 0.0;
      notch_b2_ = 0.0;
      notch_a1_ = 0.0;
      notch_a2_ = 0.0;
      return;
    }

    const double frequency = std::max(
      params.velocity_notch_hz, sample_rate_hz * 0.001);
    const double bandwidth = std::max(
      params.velocity_notch_bandwidth_hz, sample_rate_hz * 0.001);
    const double alpha = std::tan(M_PI * bandwidth / sample_rate_hz);
    const double beta = -std::cos(2.0 * M_PI * frequency / sample_rate_hz);
    const double a0_inverse = 1.0 / (alpha + 1.0);
    notch_b0_ = a0_inverse;
    notch_b1_ = 2.0 * beta * a0_inverse;
    notch_b2_ = a0_inverse;
    notch_a1_ = notch_b1_;
    notch_a2_ = (1.0 - alpha) * a0_inverse;
  }

  Eigen::Vector3d applyNotch(
    const Eigen::Vector3d & input, double dt,
    const VelocityAccelerationFilterParams & params)
  {
    if (!notch_initialized_) {
      resetNotch(input, dt, params);
    } else {
      configureNotch(dt, params);
    }
    const Eigen::Vector3d output =
      notch_b0_ * input + notch_b1_ * notch_x1_ + notch_b2_ * notch_x2_ -
      notch_a1_ * notch_y1_ - notch_a2_ * notch_y2_;
    notch_x2_ = notch_x1_;
    notch_x1_ = input;
    notch_y2_ = notch_y1_;
    notch_y1_ = output;
    return output;
  }

  bool initialized_{false};
  bool notch_initialized_{false};
  Eigen::Vector3d velocity_state_{Eigen::Vector3d::Zero()};
  Eigen::Vector3d derivative_state_{Eigen::Vector3d::Zero()};
  Eigen::Vector3d notch_x1_{Eigen::Vector3d::Zero()};
  Eigen::Vector3d notch_x2_{Eigen::Vector3d::Zero()};
  Eigen::Vector3d notch_y1_{Eigen::Vector3d::Zero()};
  Eigen::Vector3d notch_y2_{Eigen::Vector3d::Zero()};
  double notch_b0_{1.0};
  double notch_b1_{0.0};
  double notch_b2_{0.0};
  double notch_a1_{0.0};
  double notch_a2_{0.0};
};

}  // namespace geometric_controller

#endif  // GEOMETRIC_CONTROLLER__CONTROLLERS__VELOCITY_ACCELERATION_FILTER_HPP_
