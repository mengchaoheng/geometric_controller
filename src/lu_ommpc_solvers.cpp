// Dependency-free baseline solvers and the solver factory.
#include "lu_ommpc/solver.hpp"

#include <Eigen/Cholesky>

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

bool validProblem(const QPProblem & problem)
{
  const Eigen::Index n = problem.g.size();
  return n > 0 && problem.H.rows() == n && problem.H.cols() == n &&
         problem.A.cols() == n && problem.A.rows() == problem.lower.size() &&
         problem.lower.size() == problem.upper.size() && problem.H.allFinite() &&
         problem.g.allFinite() && problem.A.allFinite() &&
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

class DenseBoxActiveSetSolver final : public QPSolver
{
public:
  explicit DenseBoxActiveSetSolver(const MpcConfig & config)
  : max_iterations_(config.solver_max_iterations), tolerance_(config.solver_tolerance) {}

  std::string name() const override {return "dense_box_active_set";}

  bool update(const QPProblem & problem) override
  {
    const auto start = Clock::now();
    valid_ = validProblem(problem) && isIdentityConstraint(problem);
    if (valid_) {
      problem_ = problem;
    }
    update_us_ = elapsedUs(start);
    return valid_;
  }

  void warmStart(const Eigen::VectorXd & x) override {warm_start_ = x;}
  void clearWarmStart() override {warm_start_.resize(0);}

  SolverResult solve() override
  {
    SolverResult result;
    result.update_us = update_us_;
    if (!valid_) {
      result.status = SolverStatus::kInvalidProblem;
      return result;
    }
    const auto start = Clock::now();
    const Eigen::Index n = problem_.g.size();
    Eigen::VectorXd x;
    Eigen::Array<bool, Eigen::Dynamic, 1> active_lower(n);
    Eigen::Array<bool, Eigen::Dynamic, 1> active_upper(n);
    const double active_tolerance = std::max(tolerance_, 1e-9);

    if (warm_start_.size() == n && warm_start_.allFinite()) {
      x = warm_start_.cwiseMax(problem_.lower).cwiseMin(problem_.upper);
      active_lower = (x.array() - problem_.lower.array()).abs() <= active_tolerance;
      active_upper = (x.array() - problem_.upper.array()).abs() <= active_tolerance;
      active_upper = active_upper && !active_lower;
    } else {
      Eigen::LDLT<Eigen::MatrixXd> ldlt(problem_.H);
      if (ldlt.info() != Eigen::Success) {
        result.status = SolverStatus::kNumericalFailure;
        result.solve_us = elapsedUs(start);
        return result;
      }
      const Eigen::VectorXd unconstrained = ldlt.solve(-problem_.g);
      x = unconstrained.cwiseMax(problem_.lower).cwiseMin(problem_.upper);
      active_lower = unconstrained.array() < problem_.lower.array();
      active_upper = unconstrained.array() > problem_.upper.array();
    }

    bool converged = false;
    for (int iteration = 0; iteration < max_iterations_; ++iteration) {
      result.iterations = iteration + 1;
      std::vector<Eigen::Index> active;
      std::vector<Eigen::Index> free;
      active.reserve(static_cast<std::size_t>(n));
      free.reserve(static_cast<std::size_t>(n));
      for (Eigen::Index i = 0; i < n; ++i) {
        if (active_lower[i] || active_upper[i]) {
          active.push_back(i);
        } else {
          free.push_back(i);
        }
      }

      if (!free.empty()) {
        Eigen::MatrixXd Hff(free.size(), free.size());
        Eigen::VectorXd rhs(free.size());
        for (std::size_t row = 0; row < free.size(); ++row) {
          rhs[static_cast<Eigen::Index>(row)] = -problem_.g[free[row]];
          for (std::size_t col = 0; col < free.size(); ++col) {
            Hff(static_cast<Eigen::Index>(row), static_cast<Eigen::Index>(col)) =
              problem_.H(free[row], free[col]);
          }
          for (const Eigen::Index index : active) {
            rhs[static_cast<Eigen::Index>(row)] -= problem_.H(free[row], index) * x[index];
          }
        }
        Eigen::LDLT<Eigen::MatrixXd> ldlt(Hff);
        if (ldlt.info() != Eigen::Success) {
          result.status = SolverStatus::kNumericalFailure;
          result.x = x;
          result.solve_us = elapsedUs(start);
          evaluateResult(problem_, result);
          return result;
        }
        const Eigen::VectorXd free_solution = ldlt.solve(rhs);
        bool added_bound = false;
        for (std::size_t i = 0; i < free.size(); ++i) {
          const Eigen::Index index = free[i];
          const double candidate = free_solution[static_cast<Eigen::Index>(i)];
          if (candidate < problem_.lower[index] - active_tolerance) {
            x[index] = problem_.lower[index];
            active_lower[index] = true;
            added_bound = true;
          } else if (candidate > problem_.upper[index] + active_tolerance) {
            x[index] = problem_.upper[index];
            active_upper[index] = true;
            added_bound = true;
          } else {
            x[index] = std::clamp(candidate, problem_.lower[index], problem_.upper[index]);
          }
        }
        if (added_bound) {
          continue;
        }
      }

      const Eigen::VectorXd gradient = problem_.H * x + problem_.g;
      Eigen::Index release_index = -1;
      double largest_violation = 0.0;
      for (Eigen::Index i = 0; i < n; ++i) {
        const double violation = active_lower[i] ? -gradient[i] :
          (active_upper[i] ? gradient[i] : 0.0);
        if (violation > largest_violation + active_tolerance) {
          largest_violation = violation;
          release_index = i;
        }
      }
      if (release_index >= 0) {
        active_lower[release_index] = false;
        active_upper[release_index] = false;
        continue;
      }
      if (qpBoxKktResidual(problem_, x) <= 10.0 * active_tolerance) {
        converged = true;
        break;
      }
    }

    result.x = x;
    result.status = converged ? SolverStatus::kSolved : SolverStatus::kMaxIterations;
    result.solve_us = elapsedUs(start);
    evaluateResult(problem_, result);
    return result;
  }

private:
  int max_iterations_;
  double tolerance_;
  QPProblem problem_;
  Eigen::VectorXd warm_start_;
  double update_us_{0.0};
  bool valid_{false};
};

class DenseBoxAdmmSolver final : public QPSolver
{
public:
  explicit DenseBoxAdmmSolver(const MpcConfig & config)
  : max_iterations_(std::max(config.solver_max_iterations, 20)),
    tolerance_(config.solver_tolerance), rho_(config.admm_rho) {}

  std::string name() const override {return "dense_box_admm";}

  bool update(const QPProblem & problem) override
  {
    const auto start = Clock::now();
    valid_ = validProblem(problem) && isIdentityConstraint(problem) && rho_ > 0.0;
    if (valid_) {
      problem_ = problem;
      Eigen::MatrixXd system = problem.H;
      system.diagonal().array() += rho_;
      factor_.compute(system);
      valid_ = factor_.info() == Eigen::Success;
    }
    update_us_ = elapsedUs(start);
    return valid_;
  }

  void warmStart(const Eigen::VectorXd & x) override {warm_start_ = x;}
  void clearWarmStart() override {warm_start_.resize(0);}

  SolverResult solve() override
  {
    SolverResult result;
    result.update_us = update_us_;
    if (!valid_) {
      result.status = SolverStatus::kInvalidProblem;
      return result;
    }
    const auto start = Clock::now();
    const Eigen::Index n = problem_.g.size();
    Eigen::VectorXd z = Eigen::VectorXd::Zero(n);
    if (warm_start_.size() == n && warm_start_.allFinite()) {
      z = warm_start_.cwiseMax(problem_.lower).cwiseMin(problem_.upper);
    }
    Eigen::VectorXd x = z;
    Eigen::VectorXd dual = Eigen::VectorXd::Zero(n);
    bool converged = false;
    for (int iteration = 0; iteration < max_iterations_; ++iteration) {
      result.iterations = iteration + 1;
      x = factor_.solve(rho_ * (z - dual) - problem_.g);
      if (factor_.info() != Eigen::Success || !x.allFinite()) {
        result.status = SolverStatus::kNumericalFailure;
        result.x = z;
        result.solve_us = elapsedUs(start);
        evaluateResult(problem_, result);
        return result;
      }
      const Eigen::VectorXd old_z = z;
      z = (x + dual).cwiseMax(problem_.lower).cwiseMin(problem_.upper);
      dual += x - z;
      const double primal = (x - z).lpNorm<Eigen::Infinity>();
      const double dual_residual = (rho_ * (z - old_z)).lpNorm<Eigen::Infinity>();
      if (primal <= tolerance_ && dual_residual <= tolerance_) {
        converged = true;
        break;
      }
    }
    result.x = z;
    result.status = converged ? SolverStatus::kSolved : SolverStatus::kMaxIterations;
    result.solve_us = elapsedUs(start);
    evaluateResult(problem_, result);
    // ADMM residual tolerances differ from the common KKT evaluator. Accept a
    // numerically good result even when the splitting residual stopped first.
    if (result.status == SolverStatus::kMaxIterations &&
      result.kkt_residual <= 10.0 * tolerance_ && result.primal_residual <= tolerance_)
    {
      result.status = SolverStatus::kSolved;
    }
    return result;
  }

private:
  int max_iterations_;
  double tolerance_;
  double rho_;
  QPProblem problem_;
  Eigen::VectorXd warm_start_;
  Eigen::LDLT<Eigen::MatrixXd> factor_;
  double update_us_{0.0};
  bool valid_{false};
};

}  // namespace

std::unique_ptr<QPSolver> makeSolver(const std::string & name, const MpcConfig & config)
{
  if (auto structured = makeStructuredSolver(name, config)) {
    return structured;
  }
  if (auto external = makeExternalSolver(name, config)) {
    return external;
  }
  if (name == "active_set" || name == "dense_box_active_set") {
    return std::make_unique<DenseBoxActiveSetSolver>(config);
  }
  if (name == "admm" || name == "dense_box_admm") {
    return std::make_unique<DenseBoxAdmmSolver>(config);
  }
  throw std::invalid_argument("unknown QP solver: " + name);
}

std::vector<std::string> availableSolvers()
{
  auto solvers = availableExternalSolvers();
  const auto structured = availableStructuredSolvers();
  solvers.insert(solvers.end(), structured.begin(), structured.end());
  return solvers;
}

bool isIdentityConstraint(const QPProblem & problem, double tolerance)
{
  return problem.A.rows() == problem.A.cols() &&
         problem.A.rows() == problem.g.size() &&
         (problem.A - Eigen::MatrixXd::Identity(problem.A.rows(), problem.A.cols()))
         .cwiseAbs().maxCoeff() <= tolerance;
}

double qpObjective(const QPProblem & problem, const Eigen::VectorXd & x)
{
  return 0.5 * x.dot(problem.H * x) + problem.g.dot(x);
}

double qpPrimalResidual(const QPProblem & problem, const Eigen::VectorXd & x)
{
  if (x.size() != problem.A.cols()) {
    return std::numeric_limits<double>::infinity();
  }
  const Eigen::VectorXd value = problem.A * x;
  return std::max(
    (problem.lower - value).cwiseMax(0.0).maxCoeff(),
    (value - problem.upper).cwiseMax(0.0).maxCoeff());
}

double qpBoxKktResidual(const QPProblem & problem, const Eigen::VectorXd & x)
{
  if (!isIdentityConstraint(problem) || x.size() != problem.g.size()) {
    return std::numeric_limits<double>::infinity();
  }
  const Eigen::VectorXd gradient = problem.H * x + problem.g;
  const double lipschitz = std::max(1.0, problem.H.cwiseAbs().rowwise().sum().maxCoeff());
  const double step = 1.0 / lipschitz;
  const Eigen::VectorXd projected = (x - step * gradient).cwiseMax(problem.lower).cwiseMin(
    problem.upper);
  // Gradient mapping is continuous as a variable approaches a bound, unlike
  // classifying active constraints with a hard distance tolerance.
  return ((x - projected) / step).lpNorm<Eigen::Infinity>();
}

}  // namespace lu_ommpc
