// OMMPC solver factory and common QP diagnostics for the five retained paths.
#include "lu_ommpc/solver.hpp"

#include <Eigen/Dense>

#include <algorithm>
#include <limits>
#include <stdexcept>

namespace lu_ommpc
{

std::unique_ptr<QPSolver> makeSolver(const std::string & name, const MpcConfig & config)
{
  if (name == "qpdunes") {
    return makeStructuredSolver(name, config);
  }
  if (name == "hpipm_ocp") {
    return makeStructuredSolver(name, config);
  }
  if (name == "qpoases" || name == "osqp" || name == "daqp") {
    return makeExternalSolver(name, config);
  }
  throw std::invalid_argument(
          "unknown QP solver: " + name +
          "; supported baselines: qpdunes, hpipm_ocp, qpoases, osqp, daqp");
}

std::vector<std::string> availableSolvers()
{
  return {"qpdunes", "hpipm_ocp", "qpoases", "osqp", "daqp"};
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
  return ((x - projected) / step).lpNorm<Eigen::Infinity>();
}

}  // namespace lu_ommpc
