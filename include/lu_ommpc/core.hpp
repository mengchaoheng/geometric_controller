// Lu et al. OMMPC core, embedded in geometric_controller.
#ifndef LU_OMMPC__CORE_HPP_
#define LU_OMMPC__CORE_HPP_

#include <memory>
#include <string>

#include "lu_ommpc/solver.hpp"
#include "lu_ommpc/qp_dataset.hpp"
#include "lu_ommpc/so3.hpp"
#include "lu_ommpc/types.hpp"

namespace lu_ommpc
{

ReferenceKnot flatnessReference(const FlatOutput & flat, double gravity);

void linearizeErrorDynamics(
  const ReferenceKnot & reference, double dt,
  Eigen::Matrix<double, kStateDim, kStateDim> & A,
  Eigen::Matrix<double, kStateDim, kInputDim> & B);

Eigen::Matrix<double, kStateDim, 1> stateError(
  const State & state, const State & reference);

class QPBuilder
{
public:
  explicit QPBuilder(MpcConfig config);
  QPProblem build(
    const State & state, const ReferenceHorizon & reference,
    double * manifold_us = nullptr, double * linearization_us = nullptr);
  void buildInto(
    const State & state, const ReferenceHorizon & reference, QPBuildMode mode,
    QPProblem & problem, double * manifold_us = nullptr,
    double * linearization_us = nullptr);
  const MpcConfig & config() const {return config_;}

private:
  MpcConfig config_;
  std::vector<Eigen::Matrix<double, kStateDim, kStateDim>> state_matrices_;
  std::vector<Eigen::Matrix<double, kStateDim, kInputDim>> input_matrices_;
  Eigen::MatrixXd mu_row_;
  Eigen::MatrixXd mu_next_;
  Eigen::MatrixXd weighted_mu_;
};

class OMMPCController
{
public:
  OMMPCController(MpcConfig config, const std::string & solver_name);
  MpcResult solve(const State & state, const ReferenceHorizon & reference);
  void reset();
  const QPProblem & lastProblem() const {return last_problem_;}
  const Eigen::VectorXd & lastWarmStart() const {return last_warm_start_;}
  std::string solverName() const {return solver_->name();}

private:
  Eigen::VectorXd shiftedSolution(const Eigen::VectorXd & solution) const;

  MpcConfig config_;
  QPBuilder builder_;
  std::unique_ptr<QPSolver> solver_;
  QPProblem last_problem_;
  Eigen::VectorXd previous_solution_;
  Eigen::VectorXd last_warm_start_;
  std::unique_ptr<QPDatasetWriter> dataset_writer_;
  uint64_t dataset_sample_{0};
};

}  // namespace lu_ommpc

#endif  // LU_OMMPC__CORE_HPP_
