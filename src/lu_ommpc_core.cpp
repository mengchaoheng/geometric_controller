// Lu et al. OMMPC core, embedded in geometric_controller.
#include "lu_ommpc/core.hpp"

#include <Eigen/Geometry>

#include <algorithm>
#include <chrono>
#include <cmath>
#include <stdexcept>

namespace lu_ommpc
{
namespace
{

using Clock = std::chrono::steady_clock;

double elapsedUs(const Clock::time_point & start)
{
  return std::chrono::duration<double, std::micro>(Clock::now() - start).count();
}

Eigen::Vector3d normalizedDerivative(
  const Eigen::Vector3d & value, const Eigen::Vector3d & derivative,
  Eigen::Vector3d & normalized)
{
  const double norm = value.norm();
  if (!std::isfinite(norm) || norm < 1e-9) {
    throw std::runtime_error("singular flatness reference vector");
  }
  normalized = value / norm;
  return (Eigen::Matrix3d::Identity() - normalized * normalized.transpose()) * derivative / norm;
}

}  // namespace

Eigen::Matrix3d hat(const Eigen::Vector3d & value)
{
  Eigen::Matrix3d out;
  out << 0.0, -value.z(), value.y(),
    value.z(), 0.0, -value.x(),
    -value.y(), value.x(), 0.0;
  return out;
}

Eigen::Vector3d vee(const Eigen::Matrix3d & value)
{
  return Eigen::Vector3d(value(2, 1), value(0, 2), value(1, 0));
}

Eigen::Matrix3d expSO3(const Eigen::Vector3d & phi)
{
  const double theta_squared = phi.squaredNorm();
  const Eigen::Matrix3d K = hat(phi);
  double a = 0.0;
  double b = 0.0;
  if (theta_squared < 1e-12) {
    a = 1.0 - theta_squared / 6.0 + theta_squared * theta_squared / 120.0;
    b = 0.5 - theta_squared / 24.0 + theta_squared * theta_squared / 720.0;
  } else {
    const double theta = std::sqrt(theta_squared);
    a = std::sin(theta) / theta;
    b = (1.0 - std::cos(theta)) / theta_squared;
  }
  return Eigen::Matrix3d::Identity() + a * K + b * K * K;
}

Eigen::Matrix3d projectSO3(const Eigen::Matrix3d & rotation)
{
  const Eigen::JacobiSVD<Eigen::Matrix3d> svd(
    rotation, Eigen::ComputeFullU | Eigen::ComputeFullV);
  Eigen::Matrix3d correction = Eigen::Matrix3d::Identity();
  correction(2, 2) = (svd.matrixU() * svd.matrixV().transpose()).determinant();
  return svd.matrixU() * correction * svd.matrixV().transpose();
}

Eigen::Vector3d logSO3(const Eigen::Matrix3d & rotation)
{
  Eigen::Quaterniond quaternion(projectSO3(rotation));
  quaternion.normalize();
  // Select a deterministic principal representative; q and -q are identical.
  if (quaternion.w() < 0.0 ||
    (std::abs(quaternion.w()) < 1e-12 &&
    (quaternion.x() < 0.0 ||
    (std::abs(quaternion.x()) < 1e-12 && quaternion.y() < 0.0))))
  {
    quaternion.coeffs() *= -1.0;
  }
  const Eigen::Vector3d vector(quaternion.x(), quaternion.y(), quaternion.z());
  const double vector_norm = vector.norm();
  if (vector_norm < 1e-12) {
    return 2.0 * vector;
  }
  const double angle = 2.0 * std::atan2(vector_norm, quaternion.w());
  return angle * vector / vector_norm;
}

Eigen::Matrix3d leftJacobianSO3(const Eigen::Vector3d & phi)
{
  const double theta = phi.norm();
  const Eigen::Matrix3d K = hat(phi);
  if (theta < 1e-6) {
    return Eigen::Matrix3d::Identity() + 0.5 * K + (1.0 / 6.0) * K * K;
  }
  return Eigen::Matrix3d::Identity() +
         ((1.0 - std::cos(theta)) / (theta * theta)) * K +
         ((theta - std::sin(theta)) / (theta * theta * theta)) * K * K;
}

ReferenceKnot flatnessReference(const FlatOutput & flat, double gravity)
{
  const Eigen::Vector3d e3 = Eigen::Vector3d::UnitZ();
  const Eigen::Vector3d thrust_axis = gravity * e3 - flat.acceleration;
  const Eigen::Vector3d thrust_axis_dot = -flat.jerk;

  Eigen::Vector3d z_body;
  const Eigen::Vector3d z_body_dot = normalizedDerivative(
    thrust_axis, thrust_axis_dot, z_body);

  const Eigen::Vector3d x_heading(std::cos(flat.yaw), std::sin(flat.yaw), 0.0);
  const Eigen::Vector3d x_heading_dot =
    flat.yaw_rate * Eigen::Vector3d(-std::sin(flat.yaw), std::cos(flat.yaw), 0.0);
  Eigen::Vector3d y_body;
  const Eigen::Vector3d y_raw = z_body.cross(x_heading);
  const Eigen::Vector3d y_raw_dot =
    z_body_dot.cross(x_heading) + z_body.cross(x_heading_dot);
  const Eigen::Vector3d y_body_dot = normalizedDerivative(y_raw, y_raw_dot, y_body);
  const Eigen::Vector3d x_body = y_body.cross(z_body);
  const Eigen::Vector3d x_body_dot =
    y_body_dot.cross(z_body) + y_body.cross(z_body_dot);

  ReferenceKnot out;
  out.state.position = flat.position;
  out.state.velocity = flat.velocity;
  out.state.rotation.col(0) = x_body;
  out.state.rotation.col(1) = y_body;
  out.state.rotation.col(2) = z_body;
  Eigen::Matrix3d rotation_dot;
  rotation_dot.col(0) = x_body_dot;
  rotation_dot.col(1) = y_body_dot;
  rotation_dot.col(2) = z_body_dot;
  out.input.thrust_acceleration = thrust_axis.norm();
  out.input.body_rate = vee(out.state.rotation.transpose() * rotation_dot);
  return out;
}

Eigen::Matrix<double, kStateDim, 1> stateError(
  const State & state, const State & reference)
{
  Eigen::Matrix<double, kStateDim, 1> error;
  error.segment<3>(0) = state.position - reference.position;
  error.segment<3>(3) = state.velocity - reference.velocity;
  error.segment<3>(6) = logSO3(reference.rotation.transpose() * state.rotation);
  return error;
}

void linearizeErrorDynamics(
  const ReferenceKnot & reference, double dt,
  Eigen::Matrix<double, kStateDim, kStateDim> & A,
  Eigen::Matrix<double, kStateDim, kInputDim> & B)
{
  const Eigen::Vector3d phi = dt * reference.input.body_rate;
  Eigen::Matrix<double, kStateDim, kStateDim> Gx =
    Eigen::Matrix<double, kStateDim, kStateDim>::Identity();
  Eigen::Matrix<double, kStateDim, kStateDim> Gf =
    Eigen::Matrix<double, kStateDim, kStateDim>::Identity();
  Gx.block<3, 3>(6, 6) = expSO3(-phi);
  Gf.block<3, 3>(6, 6) = leftJacobianSO3(phi).transpose();

  Eigen::Matrix<double, kStateDim, kStateDim> dfdx =
    Eigen::Matrix<double, kStateDim, kStateDim>::Zero();
  Eigen::Matrix<double, kStateDim, kInputDim> dfdu =
    Eigen::Matrix<double, kStateDim, kInputDim>::Zero();
  dfdx.block<3, 3>(0, 3) = Eigen::Matrix3d::Identity();
  dfdx.block<3, 3>(3, 6) = reference.input.thrust_acceleration *
    reference.state.rotation * hat(Eigen::Vector3d::UnitZ());
  dfdu.block<3, 1>(3, 0) = -reference.state.rotation * Eigen::Vector3d::UnitZ();
  dfdu.block<3, 3>(6, 1) = Eigen::Matrix3d::Identity();
  A = Gx + dt * Gf * dfdx;
  B = dt * Gf * dfdu;
}

QPBuilder::QPBuilder(MpcConfig config)
: config_(std::move(config))
{
  if (config_.horizon_steps < 1 || config_.horizon_dt <= 0.0) {
    throw std::invalid_argument("OMMPC horizon and dt must be positive");
  }
  const int N = config_.horizon_steps;
  const int input_stack_size = kInputDim * N;
  state_matrices_.resize(static_cast<std::size_t>(N));
  input_matrices_.resize(static_cast<std::size_t>(N));
  mu_row_.setZero(kStateDim, input_stack_size);
  mu_next_.setZero(kStateDim, input_stack_size);
  weighted_mu_.setZero(kStateDim, input_stack_size);
}

QPProblem QPBuilder::build(
  const State & state, const ReferenceHorizon & reference,
  double * manifold_us, double * linearization_us)
{
  QPProblem problem;
  buildInto(
    state, reference, QPBuildMode::kBoth, problem, manifold_us, linearization_us);
  return problem;
}

void QPBuilder::buildInto(
  const State & state, const ReferenceHorizon & reference, QPBuildMode mode,
  QPProblem & problem, double * manifold_us, double * linearization_us)
{
  const int N = config_.horizon_steps;
  if (static_cast<int>(reference.size()) != N + 1) {
    throw std::invalid_argument("reference horizon must contain N+1 knots");
  }

  auto start = Clock::now();
  const Eigen::Matrix<double, kStateDim, 1> initial_error =
    stateError(state, reference.front().state);
  if (manifold_us != nullptr) {
    *manifold_us = elapsedUs(start);
  }

  start = Clock::now();
  for (int k = 0; k < N; ++k) {
    linearizeErrorDynamics(
      reference[static_cast<std::size_t>(k)], config_.horizon_dt,
      state_matrices_[static_cast<std::size_t>(k)],
      input_matrices_[static_cast<std::size_t>(k)]);
  }
  if (linearization_us != nullptr) {
    *linearization_us = elapsedUs(start);
  }

  const int input_stack_size = kInputDim * N;
  const bool build_condensed = mode != QPBuildMode::kOcp;
  const bool build_ocp = mode != QPBuildMode::kCondensed;

  if (build_condensed) {
    problem.H.resize(input_stack_size, input_stack_size);
    problem.H.setZero();
    problem.g.resize(input_stack_size);
    problem.g.setZero();
    problem.A.resize(input_stack_size, input_stack_size);
    problem.A.setIdentity();
    problem.lower.resize(input_stack_size);
    problem.upper.resize(input_stack_size);

    // Condense stage by stage. This avoids allocating Hx, the full Mu, Qbar,
    // and Rbar, and only retains one 9 x (4N) transition row.
    mu_row_.setZero();
    Eigen::Matrix<double, kStateDim, kStateDim> state_transition =
      Eigen::Matrix<double, kStateDim, kStateDim>::Identity();
    for (int k = 0; k < N; ++k) {
      const auto & A = state_matrices_[static_cast<std::size_t>(k)];
      const int previous_inputs = k * kInputDim;
      if (previous_inputs > 0) {
        mu_next_.leftCols(previous_inputs).noalias() =
          A * mu_row_.leftCols(previous_inputs);
      }
      mu_row_.swap(mu_next_);
      mu_row_.block(0, k * kInputDim, kStateDim, kInputDim) =
        input_matrices_[static_cast<std::size_t>(k)];
      const int active_inputs = (k + 1) * kInputDim;
      state_transition = A * state_transition;
      const auto & stage_weight = (k == N - 1) ? config_.P : config_.Q;
      weighted_mu_.leftCols(active_inputs).noalias() =
        stage_weight * mu_row_.leftCols(active_inputs);
      problem.H.topLeftCorner(active_inputs, active_inputs).noalias() +=
        mu_row_.leftCols(active_inputs).transpose() *
        weighted_mu_.leftCols(active_inputs);
      const Eigen::Matrix<double, kStateDim, 1> weighted_state =
        stage_weight * (state_transition * initial_error);
      problem.g.head(active_inputs).noalias() +=
        mu_row_.leftCols(active_inputs).transpose() * weighted_state;
      problem.H.block<kInputDim, kInputDim>(
        k * kInputDim, k * kInputDim) += config_.R;
    }
    // Symmetrize in place.  The previous Eigen expression created a dynamic
    // temporary matrix on every MPC cycle; the problem is unchanged, but the
    // temporary could add allocator/cache jitter on small ARM systems.
    for (int row = 0; row < input_stack_size; ++row) {
      for (int col = row + 1; col < input_stack_size; ++col) {
        const double symmetric = 0.5 * (problem.H(row, col) + problem.H(col, row));
        problem.H(row, col) = symmetric;
        problem.H(col, row) = symmetric;
      }
    }
    problem.H.diagonal().array() += 1e-9;
  } else {
    problem.H.resize(0, 0);
    problem.g.resize(0);
    problem.A.resize(0, 0);
    problem.lower.resize(0);
    problem.upper.resize(0);
  }

  const Eigen::Vector4d input_min(
    config_.thrust_acceleration_min, -config_.body_rate_max.x(),
    -config_.body_rate_max.y(), -config_.body_rate_max.z());
  const Eigen::Vector4d input_max(
    config_.thrust_acceleration_max, config_.body_rate_max.x(),
    config_.body_rate_max.y(), config_.body_rate_max.z());
  for (int k = 0; k < N; ++k) {
    const Eigen::Vector4d reference_input = reference[static_cast<std::size_t>(k)].input.vector();
    if (build_condensed) {
      problem.lower.segment<kInputDim>(k * kInputDim) = input_min - reference_input;
      problem.upper.segment<kInputDim>(k * kInputDim) = input_max - reference_input;
    }
  }

  if (build_ocp) {
    problem.ocp.x0 = initial_error;
    problem.ocp.Q = config_.Q;
    problem.ocp.P = config_.P;
    problem.ocp.R = config_.R;
    problem.ocp.A.resize(static_cast<std::size_t>(N));
    problem.ocp.B.resize(static_cast<std::size_t>(N));
    problem.ocp.lower_u.resize(static_cast<std::size_t>(N));
    problem.ocp.upper_u.resize(static_cast<std::size_t>(N));
    for (int k = 0; k < N; ++k) {
      const std::size_t index = static_cast<std::size_t>(k);
      problem.ocp.A[index] = state_matrices_[index];
      problem.ocp.B[index] = input_matrices_[index];
      const Eigen::Vector4d reference_input = reference[index].input.vector();
      problem.ocp.lower_u[index] = input_min - reference_input;
      problem.ocp.upper_u[index] = input_max - reference_input;
    }
  } else {
    problem.ocp.A.clear();
    problem.ocp.B.clear();
    problem.ocp.lower_u.clear();
    problem.ocp.upper_u.clear();
  }
}

OMMPCController::OMMPCController(MpcConfig config, const std::string & solver_name)
: config_(std::move(config)), builder_(config_), solver_(makeSolver(solver_name, config_))
{
  if (!solver_) {
    throw std::invalid_argument("unsupported OMMPC solver: " + solver_name);
  }
  if (!config_.dataset_path.empty()) {
    dataset_writer_ = std::make_unique<QPDatasetWriter>(config_.dataset_path);
    if (!dataset_writer_->good()) {
      throw std::runtime_error("cannot create OMMPC dataset: " + config_.dataset_path);
    }
  }
}

Eigen::VectorXd OMMPCController::shiftedSolution(const Eigen::VectorXd & solution) const
{
  const int expected = config_.horizon_steps * kInputDim;
  if (solution.size() != expected) {
    return Eigen::VectorXd();
  }
  Eigen::VectorXd shifted(expected);
  if (expected > kInputDim) {
    shifted.head(expected - kInputDim) = solution.tail(expected - kInputDim);
  }
  shifted.tail(kInputDim) = solution.tail(kInputDim);
  return shifted;
}

MpcResult OMMPCController::solve(const State & state, const ReferenceHorizon & reference)
{
  const auto total_start = Clock::now();
  MpcResult result;
  const auto build_start = Clock::now();
  const QPBuildMode build_mode = (dataset_writer_ || config_.verify_solution) ?
    QPBuildMode::kBoth : solver_->buildMode();
  builder_.buildInto(
    state, reference, build_mode, last_problem_,
    &result.manifold_us, &result.linearization_us);
  result.qp_build_us = elapsedUs(build_start) - result.manifold_us - result.linearization_us;

  last_warm_start_ = config_.warm_start ? shiftedSolution(previous_solution_) : Eigen::VectorXd();
  if (dataset_writer_) {
    QPSnapshot snapshot;
    snapshot.timestamp_us = static_cast<uint64_t>(
      static_cast<double>(dataset_sample_++) * config_.horizon_dt * 1.0e6);
    snapshot.problem = last_problem_;
    snapshot.warm_start = last_warm_start_;
    snapshot.state = state;
    snapshot.reference = reference;
    snapshot.has_mpc_input = true;
    if (!dataset_writer_->write(snapshot)) {
      throw std::runtime_error("failed while writing OMMPC dataset");
    }
  }
  const Eigen::Index decision_size = config_.horizon_steps * kInputDim;
  if (solver_->update(last_problem_)) {
    if (last_warm_start_.size() == decision_size) {
      solver_->warmStart(last_warm_start_);
    } else {
      solver_->clearWarmStart();
    }
    result.solver = solver_->solve();
  } else {
    result.solver.status = SolverStatus::kInvalidProblem;
  }
  result.candidate_status = result.solver.status;
  result.candidate_primal_residual = result.solver.primal_residual;
  auto verify = [this](SolverResult & solved, double * scaled_kkt) {
      if (solved.status != SolverStatus::kSolved ||
        solved.x.size() != config_.horizon_steps * kInputDim || !solved.x.allFinite())
      {
        return false;
      }
      if (!config_.verify_solution) {return true;}
      if (solved.x.size() != last_problem_.g.size()) {return false;}
      solved.objective = qpObjective(last_problem_, solved.x);
      solved.primal_residual = qpPrimalResidual(last_problem_, solved.x);
      solved.kkt_residual = qpBoxKktResidual(last_problem_, solved.x);
      const Eigen::VectorXd gradient = last_problem_.H * solved.x + last_problem_.g;
      const double gradient_scale = std::max(
        1.0, std::max(gradient.lpNorm<Eigen::Infinity>(),
        last_problem_.g.lpNorm<Eigen::Infinity>()));
      *scaled_kkt = solved.kkt_residual / gradient_scale;
      return std::isfinite(solved.primal_residual) && std::isfinite(*scaled_kkt) &&
             solved.primal_residual <= 1e-7 && *scaled_kkt <= 1e-4;
    };
  const bool candidate_acceptable = verify(result.solver, &result.candidate_kkt_scaled);
  result.candidate_primal_residual = result.solver.primal_residual;
  if (candidate_acceptable && result.solver.x.size() >= kInputDim) {
    previous_solution_ = result.solver.x;
    const Eigen::Vector4d command = reference.front().input.vector() +
      result.solver.x.head<kInputDim>();
    result.command.thrust_acceleration = std::clamp(
      command[0], config_.thrust_acceleration_min, config_.thrust_acceleration_max);
    result.command.body_rate = command.tail<3>().cwiseMax(-config_.body_rate_max).cwiseMin(
      config_.body_rate_max);
    result.command_valid = command.allFinite();
  }
  result.total_us = elapsedUs(total_start);
  return result;
}

void OMMPCController::reset()
{
  previous_solution_.resize(0);
  last_warm_start_.resize(0);
  solver_->clearWarmStart();
}

}  // namespace lu_ommpc
