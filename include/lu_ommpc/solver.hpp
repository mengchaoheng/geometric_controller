// Small extensible QP solver interface used by geometric_controller's Lu OMMPC.
#ifndef LU_OMMPC__SOLVER_HPP_
#define LU_OMMPC__SOLVER_HPP_

#include <memory>
#include <string>
#include <vector>

#include "lu_ommpc/types.hpp"

namespace lu_ommpc
{

class QPSolver
{
public:
  virtual ~QPSolver() = default;
  virtual std::string name() const = 0;
  virtual QPBuildMode buildMode() const {return QPBuildMode::kCondensed;}
  virtual bool update(const QPProblem & problem) = 0;
  virtual void warmStart(const Eigen::VectorXd & x) = 0;
  virtual void clearWarmStart() = 0;
  virtual SolverResult solve() = 0;
};

std::unique_ptr<QPSolver> makeSolver(const std::string & name, const MpcConfig & config);
std::vector<std::string> availableSolvers();

std::unique_ptr<QPSolver> makeExternalSolver(
  const std::string & name, const MpcConfig & config);
std::vector<std::string> availableExternalSolvers();

std::unique_ptr<QPSolver> makeStructuredSolver(
  const std::string & name, const MpcConfig & config);

double qpObjective(const QPProblem & problem, const Eigen::VectorXd & x);
double qpPrimalResidual(const QPProblem & problem, const Eigen::VectorXd & x);
double qpBoxKktResidual(const QPProblem & problem, const Eigen::VectorXd & x);
bool isIdentityConstraint(const QPProblem & problem, double tolerance = 1e-12);

}  // namespace lu_ommpc

#endif  // LU_OMMPC__SOLVER_HPP_
