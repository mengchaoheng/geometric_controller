// Adapter for CVXPYgen's fixed-N=8, structure-preserving generated C solver.
#include "lu_ommpc/solver.hpp"

extern "C" {
#include <cpg_solve.h>
}

#include <chrono>
#include <algorithm>
#include <cmath>
#include <cstring>
#include <memory>
#include <string>

namespace lu_ommpc
{
namespace
{
using Clock = std::chrono::steady_clock;
double elapsedUs(const Clock::time_point & start)
{
  return std::chrono::duration<double, std::micro>(Clock::now() - start).count();
}

class CvxpygenOsqpSolver final : public QPSolver
{
public:
  explicit CvxpygenOsqpSolver(const MpcConfig & config)
  : max_iterations_(std::max(4000, config.solver_max_iterations)),
    tolerance_(std::max(1.0e-6, config.solver_tolerance))
  {
    lu__cpg_set_solver_default_settings();
    lu__cpg_set_solver_max_iter(max_iterations_);
    lu__cpg_set_solver_eps_abs(tolerance_);
    lu__cpg_set_solver_eps_rel(tolerance_);
    lu__cpg_set_solver_warm_starting(1);
  }
  std::string name() const override {return "cvxpygen_osqp";}
  QPBuildMode buildMode() const override {return QPBuildMode::kOcp;}

  bool update(const QPProblem & problem) override
  {
    const auto start = Clock::now();
    valid_ = compatible(problem);
    if (!valid_) {update_us_ = elapsedUs(start); return false;}
    problem_ = &problem;
    for (int i = 0; i < kStateDim; ++i) {lu__cpg_update_x0(i, problem.ocp.x0[i]);}
    for (int k = 0; k < 8; ++k) {
      for (int j = 0; j < kInputDim; ++j) {
        const int index = k * kInputDim + j;
        lu__cpg_update_lower_u(index, problem.ocp.lower_u[k][j]);
        lu__cpg_update_upper_u(index, problem.ocp.upper_u[k][j]);
      }
      updateStage(k, problem.ocp.A[k], problem.ocp.B[k]);
    }
    update_us_ = elapsedUs(start);
    return true;
  }
  void warmStart(const Eigen::VectorXd &) override {lu__cpg_set_solver_warm_starting(1);}
  void clearWarmStart() override {lu__cpg_set_solver_warm_starting(0);}
  SolverResult solve() override
  {
    SolverResult result; result.update_us = update_us_;
    if (!valid_) {return result;}
    const auto start = Clock::now(); lu__cpg_solve(); result.solve_us = elapsedUs(start);
    result.iterations = lu__CPG_Result.info->iter;
    result.x = Eigen::Map<Eigen::VectorXd>(lu__CPG_Result.prim->u, 8 * kInputDim);
    const char * status = lu__CPG_Result.info->status;
    result.status = status && std::strncmp(status, "solved", 6) == 0 ?
      SolverStatus::kSolved :
      (result.iterations >= max_iterations_ ? SolverStatus::kMaxIterations :
      SolverStatus::kNumericalFailure);
    if (problem_->H.rows() == result.x.size()) {
      result.objective = qpObjective(*problem_, result.x);
      result.primal_residual = qpPrimalResidual(*problem_, result.x);
      result.kkt_residual = qpBoxKktResidual(*problem_, result.x);
    }
    return result;
  }

private:
  static bool compatible(const QPProblem & p)
  {
    if (!p.ocp.valid() || p.ocp.A.size() != 8U) {return false;}
    MpcConfig defaults;
    return p.ocp.Q.isApprox(defaults.Q, 1e-12) &&
           p.ocp.P.isApprox(defaults.P, 1e-12) &&
           p.ocp.R.isApprox(defaults.R, 1e-12);
  }
  static void updateStage(
    int stage, const Eigen::Matrix<double, kStateDim, kStateDim> & A,
    const Eigen::Matrix<double, kStateDim, kInputDim> & B)
  {
    using Update = void (*)(cpg_int, cpg_float);
    static const Update update_a[8] = {
      lu__cpg_update_A_0, lu__cpg_update_A_1, lu__cpg_update_A_2, lu__cpg_update_A_3,
      lu__cpg_update_A_4, lu__cpg_update_A_5, lu__cpg_update_A_6, lu__cpg_update_A_7};
    static const Update update_b[8] = {
      lu__cpg_update_B_0, lu__cpg_update_B_1, lu__cpg_update_B_2, lu__cpg_update_B_3,
      lu__cpg_update_B_4, lu__cpg_update_B_5, lu__cpg_update_B_6, lu__cpg_update_B_7};
    for (int i = 0; i < A.size(); ++i) {update_a[stage](i, A.data()[i]);}
    for (int i = 0; i < B.size(); ++i) {update_b[stage](i, B.data()[i]);}
  }
  int max_iterations_;
  double tolerance_, update_us_{0.0};
  const QPProblem * problem_{nullptr};
  bool valid_{false};
};
}  // namespace

std::unique_ptr<QPSolver> makeCvxpygenSolver(
  const std::string & name, const MpcConfig & config)
{
  if (name == "cvxpygen_osqp") {return std::make_unique<CvxpygenOsqpSolver>(config);}
  return nullptr;
}
}  // namespace lu_ommpc
