// Structure-exploiting OMMPC solver adapters.
#include "lu_ommpc/solver.hpp"

#include <hpipm_d_ocp_qp.h>
#include <hpipm_d_ocp_qp_dim.h>
#include <hpipm_d_ocp_qp_ipm.h>
#include <hpipm_d_ocp_qp_sol.h>
#include <qpDUNES.h>
#include <tiny_api.hpp>

#include <algorithm>
#include <chrono>
#include <memory>
#include <limits>
#include <string>
#include <vector>

namespace lu_ommpc
{
namespace
{
using Clock = std::chrono::steady_clock;
double elapsedUs(const Clock::time_point & start)
{
  return std::chrono::duration<double, std::micro>(Clock::now() - start).count();
}
void evaluate(const QPProblem & p, SolverResult & r)
{
  if (r.x.size() == p.g.size() && r.x.allFinite()) {
    r.objective = qpObjective(p, r.x);
    r.primal_residual = qpPrimalResidual(p, r.x);
    r.kkt_residual = qpBoxKktResidual(p, r.x);
  } else if (p.ocp.valid() &&
    r.x.size() == static_cast<Eigen::Index>(p.ocp.A.size() * kInputDim) &&
    r.x.allFinite())
  {
    Eigen::Matrix<double, kStateDim, 1> state = p.ocp.x0;
    double objective = 0.0;
    double primal = 0.0;
    for (std::size_t k = 0; k < p.ocp.A.size(); ++k) {
      const auto input = r.x.segment<kInputDim>(static_cast<Eigen::Index>(k * kInputDim));
      objective += 0.5 * state.dot(p.ocp.Q * state) +
        0.5 * input.dot(p.ocp.R * input);
      primal = std::max(primal, (p.ocp.lower_u[k] - input).cwiseMax(0.0).maxCoeff());
      primal = std::max(primal, (input - p.ocp.upper_u[k]).cwiseMax(0.0).maxCoeff());
      state = p.ocp.A[k] * state + p.ocp.B[k] * input;
    }
    objective += 0.5 * state.dot(p.ocp.P * state);
    r.objective = objective;
    r.primal_residual = primal;
    r.kkt_residual = std::numeric_limits<double>::quiet_NaN();
  }
}

class HpipmOcpSolver final : public QPSolver
{
public:
  explicit HpipmOcpSolver(const MpcConfig & config)
  : max_iterations_(config.solver_max_iterations), tolerance_(config.solver_tolerance) {}
  std::string name() const override {return "hpipm_ocp";}
  QPBuildMode buildMode() const override {return QPBuildMode::kOcp;}

  bool update(const QPProblem & problem) override
  {
    const auto start = Clock::now();
    valid_ = problem.ocp.valid();
    if (!valid_) {update_us_ = elapsedUs(start); return false;}
    problem_ = &problem;
    const int N = static_cast<int>(problem.ocp.A.size());
    if (!initialized_ || horizon_ != N) {initialize(N);}
    Eigen::Matrix<double, kStateDim, 1> zx =
      Eigen::Matrix<double, kStateDim, 1>::Zero();
    Eigen::Matrix<double, kInputDim, 1> zu =
      Eigen::Matrix<double, kInputDim, 1>::Zero();
    for (int k = 0; k < N; ++k) {
      d_ocp_qp_set_A(k, const_cast<double *>(problem.ocp.A[k].data()), &qp_);
      d_ocp_qp_set_B(k, const_cast<double *>(problem.ocp.B[k].data()), &qp_);
      d_ocp_qp_set_Q(k, const_cast<double *>(problem.ocp.Q.data()), &qp_);
      d_ocp_qp_set_R(k, const_cast<double *>(problem.ocp.R.data()), &qp_);
      d_ocp_qp_set_q(k, zx.data(), &qp_);
      d_ocp_qp_set_r(k, zu.data(), &qp_);
      d_ocp_qp_set_lbu(k, const_cast<double *>(problem.ocp.lower_u[k].data()), &qp_);
      d_ocp_qp_set_ubu(k, const_cast<double *>(problem.ocp.upper_u[k].data()), &qp_);
    }
    d_ocp_qp_set_Q(N, const_cast<double *>(problem.ocp.P.data()), &qp_);
    d_ocp_qp_set_q(N, zx.data(), &qp_);
    d_ocp_qp_set_lbx(0, const_cast<double *>(problem.ocp.x0.data()), &qp_);
    d_ocp_qp_set_ubx(0, const_cast<double *>(problem.ocp.x0.data()), &qp_);
    update_us_ = elapsedUs(start);
    return initialized_;
  }
  void warmStart(const Eigen::VectorXd & x) override {warm_start_ = x;}
  void clearWarmStart() override {warm_start_.resize(0);}
  SolverResult solve() override
  {
    SolverResult result;
    result.update_us = update_us_;
    if (!valid_ || !initialized_) {return result;}
    int warm = 0;
    if (warm_start_.size() == horizon_ * kInputDim) {
      Eigen::Matrix<double, kStateDim, 1> warm_state = problem_->ocp.x0;
      d_ocp_qp_sol_set_x(0, warm_state.data(), &solution_);
      for (int k = 0; k < horizon_; ++k) {
        d_ocp_qp_sol_set_u(k, warm_start_.data() + k * kInputDim, &solution_);
        warm_state = problem_->ocp.A[k] * warm_state +
          problem_->ocp.B[k] * warm_start_.segment<kInputDim>(k * kInputDim);
        d_ocp_qp_sol_set_x(k + 1, warm_state.data(), &solution_);
      }
      warm = 1;
    }
    d_ocp_qp_ipm_arg_set_warm_start(&warm, &arguments_);
    const auto start = Clock::now();
    d_ocp_qp_ipm_solve(&qp_, &solution_, &arguments_, &workspace_);
    result.solve_us = elapsedUs(start);
    int status = NAN_SOL;
    d_ocp_qp_ipm_get_status(&workspace_, &status);
    d_ocp_qp_ipm_get_iter(&workspace_, &result.iterations);
    result.x.resize(horizon_ * kInputDim);
    for (int k = 0; k < horizon_; ++k) {
      d_ocp_qp_sol_get_u(k, &solution_, result.x.data() + k * kInputDim);
    }
    result.status = status == SUCCESS ? SolverStatus::kSolved :
      (status == MAX_ITER ? SolverStatus::kMaxIterations : SolverStatus::kNumericalFailure);
    evaluate(*problem_, result);
    return result;
  }

private:
  void initialize(int N)
  {
    horizon_ = N;
    nx_.assign(N + 1, kStateDim); nu_.assign(N + 1, kInputDim); nu_.back() = 0;
    nbx_.assign(N + 1, 0); nbx_.front() = kStateDim;
    nbu_.assign(N + 1, kInputDim); nbu_.back() = 0;
    ng_.assign(N + 1, 0); ns_.assign(N + 1, 0);
    dim_memory_.resize(d_ocp_qp_dim_memsize(N));
    d_ocp_qp_dim_create(N, &dimensions_, dim_memory_.data());
    d_ocp_qp_dim_set_all(
      nx_.data(), nu_.data(), nbx_.data(), nbu_.data(), ng_.data(), ns_.data(),
      &dimensions_);
    qp_memory_.resize(d_ocp_qp_memsize(&dimensions_));
    d_ocp_qp_create(&dimensions_, &qp_, qp_memory_.data());
    input_indices_ = {0, 1, 2, 3};
    state_indices_ = {0, 1, 2, 3, 4, 5, 6, 7, 8};
    for (int k = 0; k < N; ++k) {d_ocp_qp_set_idxbu(k, input_indices_.data(), &qp_);}
    d_ocp_qp_set_idxbx(0, state_indices_.data(), &qp_);
    solution_memory_.resize(d_ocp_qp_sol_memsize(&dimensions_));
    d_ocp_qp_sol_create(&dimensions_, &solution_, solution_memory_.data());
    argument_memory_.resize(d_ocp_qp_ipm_arg_memsize(&dimensions_));
    d_ocp_qp_ipm_arg_create(&dimensions_, &arguments_, argument_memory_.data());
    d_ocp_qp_ipm_arg_set_default(SPEED_ABS, &arguments_);
    int iterations = std::max(1, max_iterations_);
    d_ocp_qp_ipm_arg_set_iter_max(&iterations, &arguments_);
    d_ocp_qp_ipm_arg_set_tol_stat(&tolerance_, &arguments_);
    d_ocp_qp_ipm_arg_set_tol_eq(&tolerance_, &arguments_);
    d_ocp_qp_ipm_arg_set_tol_ineq(&tolerance_, &arguments_);
    d_ocp_qp_ipm_arg_set_tol_comp(&tolerance_, &arguments_);
    workspace_memory_.resize(d_ocp_qp_ipm_ws_memsize(&dimensions_, &arguments_));
    d_ocp_qp_ipm_ws_create(&dimensions_, &arguments_, &workspace_, workspace_memory_.data());
    initialized_ = true;
  }
  int max_iterations_, horizon_{0};
  double tolerance_, update_us_{0.0};
  const QPProblem * problem_{nullptr};
  Eigen::VectorXd warm_start_;
  std::vector<int> nx_, nu_, nbx_, nbu_, ng_, ns_, input_indices_, state_indices_;
  std::vector<unsigned char> dim_memory_, qp_memory_, solution_memory_;
  std::vector<unsigned char> argument_memory_, workspace_memory_;
  d_ocp_qp_dim dimensions_{}; d_ocp_qp qp_{}; d_ocp_qp_sol solution_{};
  d_ocp_qp_ipm_arg arguments_{}; d_ocp_qp_ipm_ws workspace_{};
  bool initialized_{false}, valid_{false};
};

class QpDunesSolver final : public QPSolver
{
public:
  explicit QpDunesSolver(const MpcConfig & config)
  : max_iterations_(config.solver_max_iterations), tolerance_(config.solver_tolerance) {}
  ~QpDunesSolver() override {cleanup();}
  std::string name() const override {return "qpdunes";}
  QPBuildMode buildMode() const override {return QPBuildMode::kOcp;}
  bool update(const QPProblem & problem) override
  {
    const auto start = Clock::now();
    valid_ = problem.ocp.valid();
    if (!valid_) {update_us_ = elapsedUs(start); return false;}
    problem_ = &problem; horizon_ = static_cast<int>(problem.ocp.A.size());
    // qpDUNES' in-place LTV data update is effective for the small/medium
    // horizons targeted here. For much larger horizons, rebuild its internal
    // Newton system to avoid stale-factor numerical failures.
    const bool setup_required = !initialized_ || configured_horizon_ != horizon_ || horizon_ > 50;
    if (setup_required) {
      cleanup();
      qpOptions_t options = qpDUNES_setupDefaultOptions();
      options.maxIter = std::max(1, max_iterations_); options.printLevel = 0;
      options.logLevel = QPDUNES_LOG_OFF;
      options.stationarityTolerance = tolerance_; options.equalityTolerance = tolerance_;
      if (qpDUNES_setup(
          &qp_, horizon_, kStateDim, kInputDim, nullptr, &options) != QPDUNES_OK)
      {
        update_us_ = elapsedUs(start); return false;
      }
      initialized_ = true;
      configured_horizon_ = horizon_;
    }
    const double inf = qp_.options.QPDUNES_INFTY;
    x_lower_.setConstant(-inf); x_upper_.setConstant(inf); zero_.setZero();
    q_ = problem.ocp.Q; r_ = problem.ocp.R; p_ = problem.ocp.P;
    for (int k = 0; k < horizon_; ++k) {
      Eigen::Matrix<double, kStateDim, 1> lx = x_lower_, ux = x_upper_;
      if (k == 0) {lx = problem.ocp.x0; ux = problem.ocp.x0;}
      a_ = problem.ocp.A[k]; b_ = problem.ocp.B[k];
      if (setup_required) {
        if (qpDUNES_setupSimpleBoundedInterval(
            &qp_, qp_.intervals[k], q_.data(), r_.data(), nullptr, a_.data(), b_.data(),
            zero_.data(), lx.data(), ux.data(), problem.ocp.lower_u[k].data(),
            problem.ocp.upper_u[k].data()) != QPDUNES_OK) {valid_ = false; break;}
      } else {
        dynamics_.leftCols<kStateDim>() = a_;
        dynamics_.rightCols<kInputDim>() = b_;
        stage_lower_.head<kStateDim>() = lx;
        stage_lower_.tail<kInputDim>() = problem.ocp.lower_u[k];
        stage_upper_.head<kStateDim>() = ux;
        stage_upper_.tail<kInputDim>() = problem.ocp.upper_u[k];
        if (qpDUNES_updateIntervalData(
            &qp_, qp_.intervals[k], nullptr, nullptr, dynamics_.data(), zero_.data(),
            stage_lower_.data(), stage_upper_.data(), nullptr, nullptr, nullptr,
            nullptr) != QPDUNES_OK) {valid_ = false; break;}
      }
    }
    if (valid_) {
      if (setup_required) {
        valid_ = qpDUNES_setupSimpleBoundedInterval(
          &qp_, qp_.intervals[horizon_], p_.data(), nullptr, nullptr, nullptr, nullptr,
          nullptr, x_lower_.data(), x_upper_.data(), nullptr, nullptr) == QPDUNES_OK;
      } else {
        valid_ = qpDUNES_updateIntervalData(
          &qp_, qp_.intervals[horizon_], nullptr, nullptr, nullptr, nullptr,
          x_lower_.data(), x_upper_.data(), nullptr, nullptr, nullptr,
          nullptr) == QPDUNES_OK;
      }
    }
    if (valid_ && setup_required) {
      valid_ = qpDUNES_setupAllLocalQPs(&qp_, QPDUNES_FALSE) == QPDUNES_OK;
    } else if (valid_) {
      // For this rapidly changing LTV horizon, keeping the dual vector at the
      // same stage was consistently worse than a cheap zero-dual restart.
      for (int i = 0; i < horizon_ * kStateDim; ++i) {qp_.lambda.data[i] = 0.0;}
      qpDUNES_indicateDataChange(&qp_);
      valid_ = qpDUNES_setupAllLocalQPs(&qp_, QPDUNES_FALSE) == QPDUNES_OK;
    }
    update_us_ = elapsedUs(start); return valid_;
  }
  void warmStart(const Eigen::VectorXd &) override {}
  void clearWarmStart() override
  {
    if (!initialized_) {return;}
    for (int i = 0; i < horizon_ * kStateDim; ++i) {qp_.lambda.data[i] = 0.0;}
  }
  SolverResult solve() override
  {
    SolverResult result; result.update_us = update_us_;
    if (!valid_ || !initialized_) {return result;}
    const auto start = Clock::now(); const return_t status = qpDUNES_solve(&qp_);
    result.solve_us = elapsedUs(start); result.x.resize(horizon_ * kInputDim);
    for (int k = 0; k < horizon_; ++k) for (int j = 0; j < kInputDim; ++j) {
      result.x[k * kInputDim + j] = qp_.intervals[k]->z.data[kStateDim + j];
    }
    result.status = status == QPDUNES_SUCC_OPTIMAL_SOLUTION_FOUND ? SolverStatus::kSolved :
      (status == QPDUNES_ERR_ITERATION_LIMIT_REACHED ? SolverStatus::kMaxIterations :
      SolverStatus::kNumericalFailure);
    evaluate(*problem_, result); return result;
  }
private:
  void cleanup() {if (initialized_) {qpDUNES_cleanup(&qp_); initialized_ = false;}}
  using RX = Eigen::Matrix<double, kStateDim, kStateDim, Eigen::RowMajor>;
  using RB = Eigen::Matrix<double, kStateDim, kInputDim, Eigen::RowMajor>;
  using RU = Eigen::Matrix<double, kInputDim, kInputDim, Eigen::RowMajor>;
  using RC = Eigen::Matrix<double, kStateDim, kStateDim + kInputDim, Eigen::RowMajor>;
  int max_iterations_, horizon_{0}, configured_horizon_{0};
  double tolerance_, update_us_{0.0};
  const QPProblem * problem_{nullptr}; qpData_t qp_{}; RX a_, q_, p_; RB b_; RU r_;
  Eigen::Matrix<double, kStateDim, 1> x_lower_, x_upper_, zero_;
  RC dynamics_;
  Eigen::Matrix<double, kStateDim + kInputDim, 1> stage_lower_, stage_upper_;
  bool initialized_{false}, valid_{false};
};

class TinyMpcLtiSolver final : public QPSolver
{
public:
  TinyMpcLtiSolver(const MpcConfig & c, bool cache_dynamics)
  : max_iterations_(c.solver_max_iterations), tolerance_(c.solver_tolerance),
    rho_(std::max(1.0e-3, c.admm_rho)), cache_dynamics_(cache_dynamics) {}
  ~TinyMpcLtiSolver() override {cleanup();}
  std::string name() const override
  {
    return cache_dynamics_ ? "tinympc_lti_cached" : "tinympc_lti";
  }
  QPBuildMode buildMode() const override {return QPBuildMode::kOcp;}
  bool update(const QPProblem & p) override
  {
    const auto start = Clock::now(); valid_ = p.ocp.valid();
    if (!valid_) {update_us_ = elapsedUs(start); return false;}
    problem_ = &p; horizon_ = static_cast<int>(p.ocp.A.size());
    const bool setup_required = solver_ == nullptr || !cache_dynamics_ ||
      configured_horizon_ != horizon_;
    if (setup_required) {
      cleanup();
      tinyMatrix affine = tinyMatrix::Zero(kStateDim, 1);
      if (tiny_setup(&solver_, p.ocp.A.front(), p.ocp.B.front(), affine, p.ocp.Q,
          p.ocp.R, rho_, kStateDim, kInputDim, horizon_ + 1, 0) != 0) {
        update_us_ = elapsedUs(start); return false;
      }
      configured_horizon_ = horizon_;
      tiny_set_x_ref(solver_, tinyMatrix::Zero(kStateDim, horizon_ + 1));
      tiny_set_u_ref(solver_, tinyMatrix::Zero(kInputDim, horizon_));
      const double admm_tolerance = std::max(1.0e-4, tolerance_);
      tiny_update_settings(solver_->settings, admm_tolerance, admm_tolerance,
        std::max(1, max_iterations_), 1, 0, 1, 0, 0, 0, 0, 0, 0);
    }
    tinyMatrix xmin = tinyMatrix::Constant(kStateDim, horizon_ + 1, -1.e12);
    tinyMatrix xmax = tinyMatrix::Constant(kStateDim, horizon_ + 1, 1.e12);
    tinyMatrix umin(kInputDim, horizon_), umax(kInputDim, horizon_);
    for (int k = 0; k < horizon_; ++k) {umin.col(k) = p.ocp.lower_u[k]; umax.col(k) = p.ocp.upper_u[k];}
    tiny_set_bound_constraints(solver_, xmin, xmax, umin, umax);
    tiny_set_x0(solver_, p.ocp.x0);
    update_us_ = elapsedUs(start); return true;
  }
  void warmStart(const Eigen::VectorXd & x) override
  {
    if (!solver_ || x.size() != horizon_ * kInputDim) {return;}
    for (int k = 0; k < horizon_; ++k) {
      solver_->work->u.col(k) = x.segment<kInputDim>(k * kInputDim);
      solver_->work->z.col(k) = solver_->work->u.col(k);
      solver_->work->znew.col(k) = solver_->work->u.col(k);
    }
  }
  void clearWarmStart() override
  {
    if (!solver_) {return;}
    solver_->work->x.setZero();
    solver_->work->u.setZero();
    solver_->work->z.setZero();
    solver_->work->znew.setZero();
    solver_->work->y.setZero();
  }
  SolverResult solve() override
  {
    SolverResult result; result.update_us = update_us_;
    if (!valid_ || !solver_) {return result;}
    const auto start = Clock::now(); const int status = tiny_solve(solver_);
    result.solve_us = elapsedUs(start); result.iterations = solver_->solution->iter;
    result.x.resize(horizon_ * kInputDim);
    for (int k = 0; k < horizon_; ++k) result.x.segment<kInputDim>(k*kInputDim) = solver_->solution->u.col(k);
    result.status = status == 0 && solver_->solution->solved ? SolverStatus::kSolved :
      SolverStatus::kMaxIterations;
    evaluate(*problem_, result); return result;
  }
private:
  void cleanup()
  {
    if (!solver_) {return;} delete solver_->solution; delete solver_->cache;
    delete solver_->settings; delete solver_->work; delete solver_; solver_ = nullptr;
  }
  int max_iterations_, horizon_{0}, configured_horizon_{0};
  double tolerance_, rho_, update_us_{0.0};
  const QPProblem * problem_{nullptr}; TinySolver * solver_{nullptr};
  bool valid_{false}, cache_dynamics_{false};
};
}  // namespace

std::unique_ptr<QPSolver> makeStructuredSolver(const std::string & name, const MpcConfig & config)
{
  if (auto generated = makeCvxpygenSolver(name, config)) {return generated;}
  if (name == "hpipm_ocp") {return std::make_unique<HpipmOcpSolver>(config);}
  if (name == "qpdunes") {return std::make_unique<QpDunesSolver>(config);}
  if (name == "tinympc_lti") {return std::make_unique<TinyMpcLtiSolver>(config, false);}
  if (name == "tinympc_lti_cached") {
    return std::make_unique<TinyMpcLtiSolver>(config, true);
  }
  return nullptr;
}
std::vector<std::string> availableStructuredSolvers()
{
  return {"hpipm_ocp", "qpdunes", "tinympc_lti", "tinympc_lti_cached", "cvxpygen_osqp"};
}
}  // namespace lu_ommpc
