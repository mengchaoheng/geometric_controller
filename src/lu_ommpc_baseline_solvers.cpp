// Small set of baseline condensed-QP adapters retained for diagnosis.
// Online flight defaults to qpDUNES; these paths are reference implementations
// for the same condensed problem and are not used as fallbacks.
#include "lu_ommpc/solver.hpp"

#include <daqp/daqp.hpp>
#include <osqp/osqp.h>
#ifdef WARM_START
#undef WARM_START
#endif
#include <qpOASES.hpp>

#include <algorithm>
#include <chrono>
#include <memory>
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

bool validProblem(const QPProblem & problem)
{
  const Eigen::Index n = problem.g.size();
  return n > 0 && problem.H.rows() == n && problem.H.cols() == n &&
         problem.A.cols() == n && problem.A.rows() == problem.lower.size() &&
         problem.lower.size() == problem.upper.size() && problem.H.allFinite() &&
         problem.g.allFinite() && problem.A.allFinite() && problem.lower.allFinite() &&
         problem.upper.allFinite() && (problem.lower.array() <= problem.upper.array()).all();
}

void evaluate(const QPProblem & problem, SolverResult & result)
{
  if (result.x.size() == problem.g.size() && result.x.allFinite()) {
    result.objective = qpObjective(problem, result.x);
    result.primal_residual = qpPrimalResidual(problem, result.x);
    result.kkt_residual = qpBoxKktResidual(problem, result.x);
  }
}

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
      if (!solver_ || solver_->getNV() != problem.g.size() || solver_->getNC() != 0) {
        solver_.reset();
        initialized_ = false;
      }
    }
    update_us_ = elapsedUs(start);
    return valid_;
  }

  void warmStart(const Eigen::VectorXd &) override {}
  void clearWarmStart() override {initialized_ = false;}

  SolverResult solve() override
  {
    SolverResult result;
    result.update_us = update_us_;
    if (!valid_) {
      return result;
    }
    const auto start = Clock::now();
    if (!solver_ || !initialized_) {
      solver_ = std::make_unique<qpOASES::SQProblem>(
        static_cast<qpOASES::int_t>(gradient_.size()), 0);
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
    evaluate(*problem_, result);
    return result;
  }

private:
  int max_iterations_;
  const QPProblem * problem_{nullptr};
  Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor> hessian_;
  Eigen::VectorXd gradient_, lower_, upper_;
  std::unique_ptr<qpOASES::SQProblem> solver_;
  double update_us_{0.0};
  bool valid_{false};
  bool initialized_{false};
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
    if (solver_) {solver_->set_warm_start();}
  }

  void clearWarmStart() override
  {
    if (solver_) {solver_->set_cold_start();}
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
    evaluate(problem_, result);
    return result;
  }

private:
  int max_iterations_;
  double tolerance_;
  int variables_{0};
  int constraints_count_{0};
  QPProblem problem_;
  Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor> constraints_;
  std::unique_ptr<DAQP> solver_;
  double update_us_{0.0};
  bool valid_{false};
};

class OsqpSolver final : public QPSolver
{
public:
  explicit OsqpSolver(const MpcConfig & config)
  : max_iterations_(config.solver_max_iterations), tolerance_(config.solver_tolerance),
    rho_(config.admm_rho) {}
  ~OsqpSolver() override {cleanup();}
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
    cleanup();
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
      n, n, static_cast<c_int>(p_values_.size()), p_values_.data(), p_rows_.data(),
      p_columns_.data());
    a_matrix_ = csc_matrix(
      m, n, static_cast<c_int>(a_values_.size()), a_values_.data(), a_rows_.data(),
      a_columns_.data());
    OSQPData data{};
    data.n = n; data.m = m; data.P = p_matrix_; data.A = a_matrix_;
    data.q = q_.data(); data.l = lower_.data(); data.u = upper_.data();
    OSQPSettings settings{};
    osqp_set_default_settings(&settings);
    settings.verbose = 0;
    settings.polish = 1;
    settings.warm_start = 1;
    settings.max_iter = std::max(20, max_iterations_);
    settings.eps_abs = tolerance_;
    settings.eps_rel = tolerance_;
    settings.rho = rho_ > 0.0 ? rho_ : settings.rho;
    valid_ = osqp_setup(&workspace_, &data, &settings) == 0 && workspace_ != nullptr;
    update_us_ = elapsedUs(start);
    return valid_;
  }

  void warmStart(const Eigen::VectorXd & x) override {warm_start_ = x;}
  void clearWarmStart() override {warm_start_.resize(0);}

  SolverResult solve() override
  {
    SolverResult result;
    result.update_us = update_us_;
    if (!valid_ || workspace_ == nullptr) {
      return result;
    }
    if (warm_start_.size() == problem_.g.size()) {
      osqp_warm_start_x(workspace_, warm_start_.data());
    }
    const auto start = Clock::now();
    const c_int code = osqp_solve(workspace_);
    result.solve_us = elapsedUs(start);
    result.iterations = workspace_->info != nullptr ? workspace_->info->iter : 0;
    if (workspace_->solution != nullptr && workspace_->solution->x != nullptr) {
      result.x = Eigen::Map<Eigen::VectorXd>(workspace_->solution->x, problem_.g.size());
    }
    const c_int status = workspace_->info != nullptr ? workspace_->info->status_val : 0;
    result.status = code == 0 && (status == OSQP_SOLVED || status == OSQP_SOLVED_INACCURATE) ?
      SolverStatus::kSolved : (status == OSQP_MAX_ITER_REACHED ? SolverStatus::kMaxIterations :
      SolverStatus::kNumericalFailure);
    evaluate(problem_, result);
    return result;
  }

private:
  void cleanup()
  {
    if (workspace_ != nullptr) {
      osqp_cleanup(workspace_);
      workspace_ = nullptr;
    }
    p_matrix_ = nullptr;
    a_matrix_ = nullptr;
    p_values_.clear(); p_rows_.clear(); p_columns_.clear();
    a_values_.clear(); a_rows_.clear(); a_columns_.clear();
  }

  int max_iterations_;
  double tolerance_, rho_;
  c_int variables_{0}, constraints_{0};
  QPProblem problem_;
  Eigen::VectorXd warm_start_;
  OSQPWorkspace * workspace_{nullptr};
  csc * p_matrix_{nullptr};
  csc * a_matrix_{nullptr};
  std::vector<c_float> p_values_, a_values_, q_, lower_, upper_;
  std::vector<c_int> p_rows_, p_columns_, a_rows_, a_columns_;
  double update_us_{0.0};
  bool valid_{false};
};

}  // namespace

std::unique_ptr<QPSolver> makeExternalSolver(
  const std::string & name, const MpcConfig & config)
{
  if (name == "qpoases") {
    return std::make_unique<QpOasesSolver>(config);
  }
  if (name == "daqp") {
    return std::make_unique<DaqpSolver>(config);
  }
  if (name == "osqp") {
    return std::make_unique<OsqpSolver>(config);
  }
  return nullptr;
}

std::vector<std::string> availableExternalSolvers()
{
  return {"qpoases", "osqp", "daqp"};
}

}  // namespace lu_ommpc
