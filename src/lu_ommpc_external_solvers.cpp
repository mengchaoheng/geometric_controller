// Mature third-party QP solver adapters for the common OMMPC QPProblem.

#include "lu_ommpc/solver.hpp"

#include <osqp.h>
#ifdef WARM_START
#undef WARM_START
#endif
#include <proxsuite/proxqp/dense/dense.hpp>
#include <qpOASES.hpp>
#include <piqp/piqp.hpp>
#include <qpSWIFT.h>
#include <daqp/constants.h>
#include <daqp/daqp.hpp>
#ifdef MAX_ITER
#undef MAX_ITER
#endif
#include <hpipm_common.h>
#include <hpipm_d_dense_qp.h>
#include <hpipm_d_dense_qp_dim.h>
#include <hpipm_d_dense_qp_ipm.h>
#include <hpipm_d_dense_qp_sol.h>
#include <cQpGenDense.h>

#include <algorithm>
#include <chrono>
#include <memory>
#include <string>
#include <utility>
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

bool validProblem(const QPProblem & problem)
{
  const Eigen::Index n = problem.g.size();
  return n > 0 && problem.H.rows() == n && problem.H.cols() == n &&
         problem.A.cols() == n && problem.A.rows() == problem.lower.size() &&
         problem.lower.size() == problem.upper.size() && problem.H.allFinite() &&
         problem.g.allFinite() && problem.A.allFinite() &&
         problem.lower.allFinite() && problem.upper.allFinite() &&
         (problem.lower.array() <= problem.upper.array()).all();
}

void evaluateResult(const QPProblem & problem, SolverResult & result)
{
  if (result.x.size() != problem.g.size() || !result.x.allFinite()) {
    return;
  }
  result.objective = qpObjective(problem, result.x);
  result.primal_residual = qpPrimalResidual(problem, result.x);
  result.kkt_residual = qpBoxKktResidual(problem, result.x);
}

class OsqpSolver final : public QPSolver
{
public:
  explicit OsqpSolver(const MpcConfig & config)
  : max_iterations_(config.solver_max_iterations), tolerance_(config.solver_tolerance),
    rho_(config.admm_rho) {}

  ~OsqpSolver() override {clearWorkspace();}
  std::string name() const override {return "osqp";}

  bool update(const QPProblem & problem) override
  {
    const auto start = Clock::now();
    valid_ = validProblem(problem) && isIdentityConstraint(problem);
    if (!valid_) {
      update_us_ = elapsedUs(start);
      return false;
    }
    problem_ = problem;
    const c_int n = static_cast<c_int>(problem.g.size());
    const c_int m = static_cast<c_int>(problem.A.rows());
    q_.assign(problem.g.data(), problem.g.data() + n);
    lower_.assign(problem.lower.data(), problem.lower.data() + m);
    upper_.assign(problem.upper.data(), problem.upper.data() + m);

    if (workspace_ != nullptr && n == variables_ && m == constraints_) {
      std::size_t value_index = 0;
      for (c_int column = 0; column < n; ++column) {
        for (c_int row = 0; row <= column; ++row) {
          p_values_[value_index++] = problem.H(row, column);
        }
      }
      valid_ = osqp_update_P(
        workspace_, p_values_.data(), nullptr,
        static_cast<c_int>(p_values_.size())) == 0 &&
        osqp_update_lin_cost(workspace_, q_.data()) == 0 &&
        osqp_update_bounds(workspace_, lower_.data(), upper_.data()) == 0;
      update_us_ = elapsedUs(start);
      return valid_;
    }

    clearWorkspace();
    variables_ = n;
    constraints_ = m;

    p_columns_.resize(static_cast<std::size_t>(n + 1));
    for (c_int column = 0; column < n; ++column) {
      p_columns_[static_cast<std::size_t>(column)] = static_cast<c_int>(p_values_.size());
      for (c_int row = 0; row <= column; ++row) {
        p_rows_.push_back(row);
        p_values_.push_back(problem.H(row, column));
      }
    }
    p_columns_.back() = static_cast<c_int>(p_values_.size());

    a_columns_.resize(static_cast<std::size_t>(n + 1));
    for (c_int column = 0; column < n; ++column) {
      a_columns_[static_cast<std::size_t>(column)] = static_cast<c_int>(a_values_.size());
      a_rows_.push_back(column);
      a_values_.push_back(1.0);
    }
    a_columns_.back() = static_cast<c_int>(a_values_.size());
    p_matrix_ = csc_matrix(
      n, n, static_cast<c_int>(p_values_.size()), p_values_.data(),
      p_rows_.data(), p_columns_.data());
    a_matrix_ = csc_matrix(
      m, n, static_cast<c_int>(a_values_.size()), a_values_.data(),
      a_rows_.data(), a_columns_.data());

    OSQPData data{};
    data.n = n;
    data.m = m;
    data.P = p_matrix_;
    data.A = a_matrix_;
    data.q = q_.data();
    data.l = lower_.data();
    data.u = upper_.data();
    OSQPSettings settings{};
    osqp_set_default_settings(&settings);
    settings.verbose = 0;
    settings.polish = 1;
    settings.warm_start = 1;
    settings.max_iter = std::max(20, max_iterations_);
    settings.eps_abs = tolerance_;
    settings.eps_rel = tolerance_;
    if (rho_ > 0.0) {
      settings.rho = rho_;
    }
    valid_ = osqp_setup(&workspace_, &data, &settings) == 0 && workspace_ != nullptr;
    update_us_ = elapsedUs(start);
    return valid_;
  }

  void warmStart(const Eigen::VectorXd & x) override {warm_start_ = x;}
  void clearWarmStart() override
  {
    warm_start_.resize(0);
    if (workspace_ != nullptr) {
      zero_primal_.assign(static_cast<std::size_t>(variables_), 0.0);
      zero_dual_.assign(static_cast<std::size_t>(constraints_), 0.0);
      osqp_warm_start(workspace_, zero_primal_.data(), zero_dual_.data());
    }
  }

  SolverResult solve() override
  {
    SolverResult result;
    result.update_us = update_us_;
    if (!valid_ || workspace_ == nullptr) {
      result.status = SolverStatus::kInvalidProblem;
      return result;
    }
    if (warm_start_.size() == problem_.g.size()) {
      osqp_warm_start_x(workspace_, warm_start_.data());
    }
    const auto start = Clock::now();
    const c_int exit_flag = osqp_solve(workspace_);
    result.solve_us = elapsedUs(start);
    result.iterations = workspace_->info != nullptr ? workspace_->info->iter : 0;
    if (exit_flag == 0 && workspace_->solution != nullptr && workspace_->solution->x != nullptr) {
      result.x = Eigen::Map<Eigen::VectorXd>(workspace_->solution->x, problem_.g.size());
    }
    const c_int status = workspace_->info != nullptr ? workspace_->info->status_val : 0;
    if (status == OSQP_SOLVED || status == OSQP_SOLVED_INACCURATE) {
      result.status = SolverStatus::kSolved;
    } else if (status == OSQP_MAX_ITER_REACHED) {
      result.status = SolverStatus::kMaxIterations;
    } else {
      result.status = SolverStatus::kNumericalFailure;
    }
    evaluateResult(problem_, result);
    return result;
  }

private:
  void clearWorkspace()
  {
    if (workspace_ != nullptr) {
      osqp_cleanup(workspace_);
      workspace_ = nullptr;
    }
    if (p_matrix_ != nullptr) {
      c_free(p_matrix_);
      p_matrix_ = nullptr;
    }
    if (a_matrix_ != nullptr) {
      c_free(a_matrix_);
      a_matrix_ = nullptr;
    }
    p_values_.clear();
    p_rows_.clear();
    p_columns_.clear();
    a_values_.clear();
    a_rows_.clear();
    a_columns_.clear();
  }

  int max_iterations_;
  double tolerance_;
  double rho_;
  c_int variables_{0};
  c_int constraints_{0};
  QPProblem problem_;
  Eigen::VectorXd warm_start_;
  OSQPWorkspace * workspace_{nullptr};
  csc * p_matrix_{nullptr};
  csc * a_matrix_{nullptr};
  std::vector<c_float> p_values_, a_values_, q_, lower_, upper_;
  std::vector<c_float> zero_primal_, zero_dual_;
  std::vector<c_int> p_rows_, p_columns_, a_rows_, a_columns_;
  double update_us_{0.0};
  bool valid_{false};
};

class QpOasesSolver final : public QPSolver
{
public:
  explicit QpOasesSolver(const MpcConfig & config)
  : max_iterations_(config.solver_max_iterations) {}

  std::string name() const override {return "qpoases";}

  bool update(const QPProblem & problem) override
  {
    const auto start = Clock::now();
    valid_ = validProblem(problem);
    if (valid_) {
      problem_ = &problem;
      hessian_ = problem.H;
      gradient_ = problem.g;
      lower_ = problem.lower;
      upper_ = problem.upper;
      if (!solver_ || solver_->getNV() != problem.g.size() ||
        solver_->getNC() != 0)
      {
        solver_.reset();
        initialized_ = false;
      }
    }
    update_us_ = elapsedUs(start);
    return valid_;
  }

  void warmStart(const Eigen::VectorXd & x) override {warm_start_ = x;}
  void clearWarmStart() override
  {
    warm_start_.resize(0);
    initialized_ = false;
  }

  SolverResult solve() override
  {
    SolverResult result;
    result.update_us = update_us_;
    if (!valid_) {
      result.status = SolverStatus::kInvalidProblem;
      return result;
    }
    const auto start = Clock::now();
    if (!solver_ || !initialized_) {
      solver_ = std::make_unique<qpOASES::SQProblem>(
        static_cast<qpOASES::int_t>(gradient_.size()),
        0);
      qpOASES::Options options;
      options.setToMPC();
      options.printLevel = qpOASES::PL_NONE;
      solver_->setOptions(options);
      solver_->setHessianType(qpOASES::HST_POSDEF);
    }
    qpOASES::int_t iterations = std::max(1, max_iterations_);
    qpOASES::returnValue code;
    if (initialized_) {
      code = solver_->hotstart(
        hessian_.data(), gradient_.data(), nullptr, lower_.data(), upper_.data(),
        nullptr, nullptr, iterations);
    } else {
      code = solver_->init(
        hessian_.data(), gradient_.data(), nullptr, lower_.data(), upper_.data(),
        nullptr, nullptr, iterations);
      initialized_ = code == qpOASES::SUCCESSFUL_RETURN;
    }
    result.solve_us = elapsedUs(start);
    result.iterations = iterations;
    result.x.resize(gradient_.size());
    if (solver_->getPrimalSolution(result.x.data()) != qpOASES::SUCCESSFUL_RETURN) {
      result.x.resize(0);
    }
    result.status = code == qpOASES::SUCCESSFUL_RETURN ? SolverStatus::kSolved :
      (code == qpOASES::RET_MAX_NWSR_REACHED ? SolverStatus::kMaxIterations :
      SolverStatus::kNumericalFailure);
    evaluateResult(*problem_, result);
    return result;
  }

private:
  using RowMajorMatrix = Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor>;
  int max_iterations_;
  const QPProblem * problem_{nullptr};
  RowMajorMatrix hessian_;
  Eigen::VectorXd gradient_, lower_, upper_, warm_start_;
  std::unique_ptr<qpOASES::SQProblem> solver_;
  double update_us_{0.0};
  bool valid_{false};
  bool initialized_{false};
};

class ProxQpSolver final : public QPSolver
{
public:
  explicit ProxQpSolver(const MpcConfig & config)
  : max_iterations_(config.solver_max_iterations), tolerance_(config.solver_tolerance) {}

  std::string name() const override {return "proxqp";}

  bool update(const QPProblem & problem) override
  {
    const auto start = Clock::now();
    valid_ = validProblem(problem);
    if (valid_) {
      problem_ = problem;
      const bool dimensions_changed = !solver_ || variables_ != problem.g.size();
      if (dimensions_changed) {
        solver_ = std::make_unique<proxsuite::proxqp::dense::QP<double>>(
          problem.g.size(), 0, 0, true);
        solver_->settings.verbose = false;
        solver_->settings.eps_abs = tolerance_;
        solver_->settings.eps_rel = tolerance_;
        solver_->settings.max_iter = std::max(1, max_iterations_);
        solver_->settings.initial_guess =
          proxsuite::proxqp::InitialGuessStatus::WARM_START_WITH_PREVIOUS_RESULT;
        solver_->init(
          problem.H, problem.g, proxsuite::nullopt, proxsuite::nullopt,
          proxsuite::nullopt, proxsuite::nullopt, proxsuite::nullopt,
          problem.lower, problem.upper, true);
        variables_ = problem.g.size();
      } else {
        solver_->update(
          problem.H, problem.g, proxsuite::nullopt, proxsuite::nullopt,
          proxsuite::nullopt, proxsuite::nullopt, proxsuite::nullopt,
          problem.lower, problem.upper, false);
      }
    }
    update_us_ = elapsedUs(start);
    return valid_;
  }

  void warmStart(const Eigen::VectorXd & x) override
  {
    warm_start_ = x;
    force_cold_start_ = false;
  }
  void clearWarmStart() override
  {
    warm_start_.resize(0);
    force_cold_start_ = true;
  }

  SolverResult solve() override
  {
    SolverResult result;
    result.update_us = update_us_;
    if (!valid_ || !solver_) {
      result.status = SolverStatus::kInvalidProblem;
      return result;
    }
    const auto start = Clock::now();
    if (warm_start_.size() == problem_.g.size()) {
      solver_->solve(warm_start_, proxsuite::nullopt, proxsuite::nullopt);
    } else {
      solver_->settings.initial_guess = force_cold_start_ ?
        proxsuite::proxqp::InitialGuessStatus::NO_INITIAL_GUESS :
        proxsuite::proxqp::InitialGuessStatus::WARM_START_WITH_PREVIOUS_RESULT;
      solver_->solve();
    }
    force_cold_start_ = false;
    result.solve_us = elapsedUs(start);
    result.x = solver_->results.x;
    result.iterations = static_cast<int>(solver_->results.info.iter);
    const auto status = solver_->results.info.status;
    result.status = status == proxsuite::proxqp::QPSolverOutput::PROXQP_SOLVED ?
      SolverStatus::kSolved :
      (status == proxsuite::proxqp::QPSolverOutput::PROXQP_MAX_ITER_REACHED ?
      SolverStatus::kMaxIterations : SolverStatus::kNumericalFailure);
    evaluateResult(problem_, result);
    return result;
  }

private:
  int max_iterations_;
  double tolerance_;
  Eigen::Index variables_{0};
  QPProblem problem_;
  Eigen::VectorXd warm_start_;
  std::unique_ptr<proxsuite::proxqp::dense::QP<double>> solver_;
  double update_us_{0.0};
  bool valid_{false}, force_cold_start_{false};
};

class DaqpSolver final : public QPSolver
{
public:
  explicit DaqpSolver(const MpcConfig & config)
  : max_iterations_(config.solver_max_iterations), tolerance_(config.solver_tolerance) {}

  std::string name() const override {return "daqp";}

  bool update(const QPProblem & problem) override
  {
    const auto start = Clock::now();
    valid_ = validProblem(problem);
    if (valid_) {
      problem_ = problem;
      constraints_ = problem.A;
      const int n = static_cast<int>(problem.g.size());
      const int m = static_cast<int>(problem.A.rows());
      if (!solver_ || n != variables_ || m != constraints_count_) {
        solver_ = std::make_unique<DAQP>(n, m, m);
        solver_->set_iter_limit(std::max(1, max_iterations_));
        const double daqp_tolerance = std::min(tolerance_, 1.0e-9);
        solver_->set_primal_tol(daqp_tolerance);
        solver_->set_dual_tol(daqp_tolerance);
        solver_->set_eta_prox(0.1 * daqp_tolerance);
        variables_ = n;
        constraints_count_ = m;
      }
      Eigen::VectorXi sense(0);
      Eigen::VectorXi break_points(0);
      valid_ = solver_->update(
        problem.H, problem.g, constraints_, problem.upper, problem.lower,
        sense, break_points) >= 0;
    }
    update_us_ = elapsedUs(start);
    return valid_;
  }

  void warmStart(const Eigen::VectorXd &) override
  {
    if (solver_) {
      solver_->set_warm_start();
    }
  }

  void clearWarmStart() override
  {
    if (solver_) {
      solver_->set_cold_start();
    }
  }

  SolverResult solve() override
  {
    SolverResult result;
    result.update_us = update_us_;
    if (!valid_ || !solver_) {
      result.status = SolverStatus::kInvalidProblem;
      return result;
    }
    const auto start = Clock::now();
    const auto & daqp_result = solver_->solve();
    result.solve_us = elapsedUs(start);
    result.iterations = solver_->get_iterations();
    result.x = daqp_result.get_primal();
    const int status = solver_->get_status();
    result.status = status == DAQP_EXIT_OPTIMAL || status == DAQP_EXIT_SOFT_OPTIMAL ?
      SolverStatus::kSolved :
      (status == DAQP_EXIT_ITERLIMIT ? SolverStatus::kMaxIterations :
      SolverStatus::kNumericalFailure);
    evaluateResult(problem_, result);
    return result;
  }

private:
  using RowMajorMatrix = Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor>;
  int max_iterations_;
  double tolerance_;
  int variables_{0};
  int constraints_count_{0};
  QPProblem problem_;
  RowMajorMatrix constraints_;
  std::unique_ptr<DAQP> solver_;
  double update_us_{0.0};
  bool valid_{false};
};

class PiqpSolver final : public QPSolver
{
public:
  explicit PiqpSolver(const MpcConfig & config)
  : max_iterations_(config.solver_max_iterations), tolerance_(config.solver_tolerance) {}

  std::string name() const override {return "piqp";}

  bool update(const QPProblem & problem) override
  {
    const auto start = Clock::now();
    valid_ = validProblem(problem);
    if (valid_) {
      problem_ = problem;
      const bool dimensions_changed = !solver_ ||
        variables_ != problem.g.size() || constraints_ != problem.A.rows();
      if (dimensions_changed) {
        solver_ = std::make_unique<piqp::DenseSolver<double>>();
        configure();
        solver_->setup(
          problem.H, problem.g, piqp::nullopt, piqp::nullopt,
          piqp::nullopt, piqp::nullopt, piqp::nullopt,
          problem.lower, problem.upper);
        variables_ = problem.g.size();
        constraints_ = problem.A.rows();
      } else {
        solver_->update(
          problem.H, problem.g, piqp::nullopt, piqp::nullopt,
          piqp::nullopt, piqp::nullopt, piqp::nullopt,
          problem.lower, problem.upper);
      }
    }
    update_us_ = elapsedUs(start);
    return valid_;
  }

  void warmStart(const Eigen::VectorXd &) override {}

  // PIQP's public dense API reinitializes its primal/dual iterate in solve();
  // it currently has no explicit primal warm-start setter.
  void clearWarmStart() override {}

  SolverResult solve() override
  {
    SolverResult result;
    result.update_us = update_us_;
    if (!valid_ || !solver_) {
      result.status = SolverStatus::kInvalidProblem;
      return result;
    }
    const auto start = Clock::now();
    const piqp::Status status = solver_->solve();
    result.solve_us = elapsedUs(start);
    result.x = solver_->result().x;
    result.iterations = static_cast<int>(solver_->result().info.iter);
    result.status = status == piqp::PIQP_SOLVED ? SolverStatus::kSolved :
      (status == piqp::PIQP_MAX_ITER_REACHED ? SolverStatus::kMaxIterations :
      SolverStatus::kNumericalFailure);
    evaluateResult(problem_, result);
    return result;
  }

private:
  void configure()
  {
    solver_->settings().verbose = false;
    solver_->settings().compute_timings = false;
    solver_->settings().eps_abs = tolerance_;
    solver_->settings().eps_rel = tolerance_;
    solver_->settings().eps_duality_gap_abs = tolerance_;
    solver_->settings().eps_duality_gap_rel = tolerance_;
    solver_->settings().max_iter = std::max(1, max_iterations_);
  }

  int max_iterations_;
  double tolerance_;
  Eigen::Index variables_{0};
  Eigen::Index constraints_{0};
  QPProblem problem_;
  std::unique_ptr<piqp::DenseSolver<double>> solver_;
  double update_us_{0.0};
  bool valid_{false};
};

class QpSwiftSolver final : public QPSolver
{
public:
  explicit QpSwiftSolver(const MpcConfig & config)
  : max_iterations_(config.solver_max_iterations), tolerance_(config.solver_tolerance) {}

  ~QpSwiftSolver() override {clearWorkspace();}
  std::string name() const override {return "qpswift";}

  bool update(const QPProblem & problem) override
  {
    const auto start = Clock::now();
    clearWorkspace();
    valid_ = validProblem(problem);
    if (valid_) {
      problem_ = problem;
      const Eigen::Index m = problem.A.rows();
      inequalities_.resize(2 * m, problem.A.cols());
      inequalities_.topRows(m) = problem.A;
      inequalities_.bottomRows(m) = -problem.A;
      bounds_.resize(2 * m);
      bounds_.head(m) = problem.upper;
      bounds_.tail(m) = -problem.lower;
      hessian_ = problem.H;
      gradient_ = problem.g;
      workspace_ = QP_SETUP_dense(
        static_cast<qp_int>(problem.g.size()), static_cast<qp_int>(2 * m), 0,
        hessian_.data(), nullptr, inequalities_.data(), gradient_.data(),
        bounds_.data(), nullptr, nullptr, COLUMN_MAJOR_ORDERING);
      valid_ = workspace_ != nullptr;
      if (valid_) {
        workspace_->options->verbose = 0;
        workspace_->options->maxit = std::max(1, max_iterations_);
        workspace_->options->abstol = tolerance_;
        workspace_->options->reltol = tolerance_;
      }
    }
    update_us_ = elapsedUs(start);
    return valid_;
  }

  void warmStart(const Eigen::VectorXd &) override {}
  void clearWarmStart() override {}

  SolverResult solve() override
  {
    SolverResult result;
    result.update_us = update_us_;
    if (!valid_ || workspace_ == nullptr) {
      result.status = SolverStatus::kInvalidProblem;
      return result;
    }
    const auto start = Clock::now();
    const qp_int status = QP_SOLVE(workspace_);
    result.solve_us = elapsedUs(start);
    result.iterations = static_cast<int>(workspace_->stats->IterationCount);
    result.x = Eigen::Map<Eigen::VectorXd>(workspace_->x, problem_.g.size());
    result.status = status == QP_OPTIMAL ? SolverStatus::kSolved :
      (status == QP_MAXIT ? SolverStatus::kMaxIterations :
      SolverStatus::kNumericalFailure);
    evaluateResult(problem_, result);
    return result;
  }

private:
  void clearWorkspace()
  {
    if (workspace_ != nullptr) {
      QP_CLEANUP_dense(workspace_);
      workspace_ = nullptr;
    }
  }

  int max_iterations_;
  double tolerance_;
  QPProblem problem_;
  Eigen::MatrixXd hessian_, inequalities_;
  Eigen::VectorXd gradient_, bounds_;
  QP * workspace_{nullptr};
  double update_us_{0.0};
  bool valid_{false};
};

class HpipmSolver final : public QPSolver
{
public:
  explicit HpipmSolver(const MpcConfig & config)
  : max_iterations_(config.solver_max_iterations), tolerance_(config.solver_tolerance) {}

  std::string name() const override {return "hpipm";}

  bool update(const QPProblem & problem) override
  {
    const auto start = Clock::now();
    valid_ = validProblem(problem);
    if (valid_) {
      problem_ = problem;
      const int n = static_cast<int>(problem.g.size());
      const int m = static_cast<int>(problem.A.rows());
      if (!initialized_ || n != variables_ || m != constraints_) {
        initialize(n, m);
      }
      d_dense_qp_set_H(problem_.H.data(), &qp_);
      d_dense_qp_set_g(problem_.g.data(), &qp_);
      d_dense_qp_set_lb(problem_.lower.data(), &qp_);
      d_dense_qp_set_ub(problem_.upper.data(), &qp_);
    }
    update_us_ = elapsedUs(start);
    return valid_ && initialized_;
  }

  void warmStart(const Eigen::VectorXd & x) override {warm_start_ = x;}
  void clearWarmStart() override {warm_start_.resize(0);}

  SolverResult solve() override
  {
    SolverResult result;
    result.update_us = update_us_;
    if (!valid_ || !initialized_) {
      result.status = SolverStatus::kInvalidProblem;
      return result;
    }
    int warm_mode = 0;
    if (warm_start_.size() == variables_) {
      d_dense_qp_sol_set_v(warm_start_.data(), &solution_);
      warm_mode = 1;
    }
    d_dense_qp_ipm_arg_set_warm_start(&warm_mode, &arguments_);
    const auto start = Clock::now();
    d_dense_qp_ipm_solve(&qp_, &solution_, &arguments_, &workspace_);
    result.solve_us = elapsedUs(start);
    int status = NAN_SOL;
    d_dense_qp_ipm_get_status(&workspace_, &status);
    d_dense_qp_ipm_get_iter(&workspace_, &result.iterations);
    result.x.resize(variables_);
    d_dense_qp_sol_get_v(&solution_, result.x.data());
    result.status = status == SUCCESS ? SolverStatus::kSolved :
      (status == MAX_ITER ? SolverStatus::kMaxIterations :
      SolverStatus::kNumericalFailure);
    evaluateResult(problem_, result);
    return result;
  }

private:
  void initialize(int n, int m)
  {
    variables_ = n;
    constraints_ = m;
    dim_memory_.resize(d_dense_qp_dim_memsize());
    d_dense_qp_dim_create(&dimensions_, dim_memory_.data());
    d_dense_qp_dim_set_nv(n, &dimensions_);
    d_dense_qp_dim_set_ne(0, &dimensions_);
    d_dense_qp_dim_set_nb(n, &dimensions_);
    d_dense_qp_dim_set_ng(0, &dimensions_);
    d_dense_qp_dim_set_ns(0, &dimensions_);

    qp_memory_.resize(d_dense_qp_memsize(&dimensions_));
    d_dense_qp_create(&dimensions_, &qp_, qp_memory_.data());
    box_indices_.resize(static_cast<std::size_t>(n));
    for (int i = 0; i < n; ++i) {
      box_indices_[static_cast<std::size_t>(i)] = i;
    }
    d_dense_qp_set_idxb(box_indices_.data(), &qp_);
    solution_memory_.resize(d_dense_qp_sol_memsize(&dimensions_));
    d_dense_qp_sol_create(&dimensions_, &solution_, solution_memory_.data());
    argument_memory_.resize(d_dense_qp_ipm_arg_memsize(&dimensions_));
    d_dense_qp_ipm_arg_create(&dimensions_, &arguments_, argument_memory_.data());
    d_dense_qp_ipm_arg_set_default(BALANCE, &arguments_);
    int iterations = std::max(1, max_iterations_);
    d_dense_qp_ipm_arg_set_iter_max(&iterations, &arguments_);
    d_dense_qp_ipm_arg_set_tol_stat(&tolerance_, &arguments_);
    d_dense_qp_ipm_arg_set_tol_eq(&tolerance_, &arguments_);
    d_dense_qp_ipm_arg_set_tol_ineq(&tolerance_, &arguments_);
    d_dense_qp_ipm_arg_set_tol_comp(&tolerance_, &arguments_);
    workspace_memory_.resize(d_dense_qp_ipm_ws_memsize(&dimensions_, &arguments_));
    d_dense_qp_ipm_ws_create(
      &dimensions_, &arguments_, &workspace_, workspace_memory_.data());
    initialized_ = true;
  }

  int max_iterations_;
  double tolerance_;
  int variables_{0};
  int constraints_{0};
  QPProblem problem_;
  Eigen::VectorXd warm_start_;
  std::vector<unsigned char> dim_memory_, qp_memory_, solution_memory_;
  std::vector<int> box_indices_;
  std::vector<unsigned char> argument_memory_, workspace_memory_;
  d_dense_qp_dim dimensions_{};
  d_dense_qp qp_{};
  d_dense_qp_sol solution_{};
  d_dense_qp_ipm_arg arguments_{};
  d_dense_qp_ipm_ws workspace_{};
  double update_us_{0.0};
  bool valid_{false};
  bool initialized_{false};
};

class OoqpSolver final : public QPSolver
{
public:
  std::string name() const override {return "ooqp";}

  bool update(const QPProblem & problem) override
  {
    const auto start = Clock::now();
    valid_ = validProblem(problem);
    if (valid_) {
      problem_ = problem;
      hessian_ = problem.H;
      constraints_ = problem.A;
      gradient_ = problem.g;
      lower_ = problem.lower;
      upper_ = problem.upper;
      lower_present_.assign(static_cast<std::size_t>(problem.A.rows()), 1);
      upper_present_.assign(static_cast<std::size_t>(problem.A.rows()), 1);
      variable_lower_.assign(static_cast<std::size_t>(problem.g.size()), 0.0);
      variable_upper_.assign(static_cast<std::size_t>(problem.g.size()), 0.0);
      variable_lower_present_.assign(static_cast<std::size_t>(problem.g.size()), 0);
      variable_upper_present_.assign(static_cast<std::size_t>(problem.g.size()), 0);
    }
    update_us_ = elapsedUs(start);
    return valid_;
  }

  void warmStart(const Eigen::VectorXd &) override {}
  void clearWarmStart() override {}

  SolverResult solve() override
  {
    SolverResult result;
    result.update_us = update_us_;
    if (!valid_) {
      result.status = SolverStatus::kInvalidProblem;
      return result;
    }
    result.x.resize(problem_.g.size());
    Eigen::VectorXd gamma(problem_.g.size()), phi(problem_.g.size());
    Eigen::VectorXd z(problem_.A.rows()), lambda(problem_.A.rows()), pi(problem_.A.rows());
    double objective = 0.0;
    int error = -1;
    const auto start = Clock::now();
    qpsolvede(
      gradient_.data(), static_cast<int>(gradient_.size()), hessian_.data(),
      variable_lower_.data(), variable_lower_present_.data(),
      variable_upper_.data(), variable_upper_present_.data(),
      nullptr, 0, nullptr,
      constraints_.data(), static_cast<int>(constraints_.rows()),
      lower_.data(), lower_present_.data(), upper_.data(), upper_present_.data(),
      result.x.data(), gamma.data(), phi.data(), nullptr,
      z.data(), lambda.data(), pi.data(), &objective, 0, &error);
    result.solve_us = elapsedUs(start);
    result.objective = objective;
    result.status = error == 0 ? SolverStatus::kSolved : SolverStatus::kNumericalFailure;
    evaluateResult(problem_, result);
    return result;
  }

private:
  using RowMajorMatrix = Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor>;
  QPProblem problem_;
  RowMajorMatrix hessian_, constraints_;
  Eigen::VectorXd gradient_, lower_, upper_;
  std::vector<double> variable_lower_, variable_upper_;
  std::vector<char> variable_lower_present_, variable_upper_present_;
  std::vector<char> lower_present_, upper_present_;
  double update_us_{0.0};
  bool valid_{false};
};

}  // namespace

std::unique_ptr<QPSolver> makeExternalSolver(
  const std::string & name, const MpcConfig & config)
{
  if (name == "osqp") {
    return std::make_unique<OsqpSolver>(config);
  }
  if (name == "qpoases") {
    return std::make_unique<QpOasesSolver>(config);
  }
  if (name == "proxqp") {
    return std::make_unique<ProxQpSolver>(config);
  }
  if (name == "daqp") {
    return std::make_unique<DaqpSolver>(config);
  }
  if (name == "piqp") {
    return std::make_unique<PiqpSolver>(config);
  }
  if (name == "qpswift") {
    return std::make_unique<QpSwiftSolver>(config);
  }
  if (name == "hpipm") {
    return std::make_unique<HpipmSolver>(config);
  }
  if (name == "ooqp") {
    return std::make_unique<OoqpSolver>();
  }
  return nullptr;
}

std::vector<std::string> availableExternalSolvers()
{
  return {"osqp", "qpoases", "proxqp", "daqp", "piqp", "qpswift", "hpipm", "ooqp"};
}

}  // namespace lu_ommpc
