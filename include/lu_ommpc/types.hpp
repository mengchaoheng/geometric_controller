// Lu et al. OMMPC core, embedded in geometric_controller.
#ifndef LU_OMMPC__TYPES_HPP_
#define LU_OMMPC__TYPES_HPP_

#include <Eigen/Dense>

#include <cstdint>
#include <limits>
#include <string>
#include <vector>

namespace lu_ommpc
{

constexpr int kStateDim = 9;
constexpr int kInputDim = 4;

enum class QPBuildMode : int
{
  kCondensed = 0,
  kOcp = 1,
  kBoth = 2,
};

struct State
{
  Eigen::Vector3d position{Eigen::Vector3d::Zero()};
  Eigen::Vector3d velocity{Eigen::Vector3d::Zero()};
  Eigen::Matrix3d rotation{Eigen::Matrix3d::Identity()};  // body FRD -> world NED
};

struct Input
{
  double thrust_acceleration{9.81};
  Eigen::Vector3d body_rate{Eigen::Vector3d::Zero()};

  Eigen::Vector4d vector() const
  {
    Eigen::Vector4d out;
    out << thrust_acceleration, body_rate;
    return out;
  }
};

struct FlatOutput
{
  Eigen::Vector3d position{Eigen::Vector3d::Zero()};
  Eigen::Vector3d velocity{Eigen::Vector3d::Zero()};
  Eigen::Vector3d acceleration{Eigen::Vector3d::Zero()};
  Eigen::Vector3d jerk{Eigen::Vector3d::Zero()};
  Eigen::Vector3d snap{Eigen::Vector3d::Zero()};
  double yaw{0.0};
  double yaw_rate{0.0};
  double yaw_acceleration{0.0};
};

struct ReferenceKnot
{
  State state;
  Input input;
};

using ReferenceHorizon = std::vector<ReferenceKnot>;

struct QPProblem
{
  // Standard form: min 0.5*x'*H*x + g'*x, lower <= A*x <= upper.
  Eigen::MatrixXd H;
  Eigen::VectorXd g;
  Eigen::MatrixXd A;
  Eigen::VectorXd lower;
  Eigen::VectorXd upper;

  // Optional uncondensed optimal-control representation of the same QP.
  // Structured solvers consume this directly; dense/sparse QP solvers ignore it.
  struct OcpData
  {
    Eigen::Matrix<double, kStateDim, 1> x0{
      Eigen::Matrix<double, kStateDim, 1>::Zero()};
    std::vector<Eigen::Matrix<double, kStateDim, kStateDim>> A;
    std::vector<Eigen::Matrix<double, kStateDim, kInputDim>> B;
    Eigen::Matrix<double, kStateDim, kStateDim> Q{
      Eigen::Matrix<double, kStateDim, kStateDim>::Zero()};
    Eigen::Matrix<double, kStateDim, kStateDim> P{
      Eigen::Matrix<double, kStateDim, kStateDim>::Zero()};
    Eigen::Matrix<double, kInputDim, kInputDim> R{
      Eigen::Matrix<double, kInputDim, kInputDim>::Zero()};
    std::vector<Eigen::Matrix<double, kInputDim, 1>> lower_u;
    std::vector<Eigen::Matrix<double, kInputDim, 1>> upper_u;

    bool valid() const
    {
      const std::size_t stages = A.size();
      return stages > 0U && B.size() == stages && lower_u.size() == stages &&
             upper_u.size() == stages;
    }
  };
  OcpData ocp;
};

enum class SolverStatus : int
{
  kSolved = 0,
  kMaxIterations = 1,
  kInvalidProblem = 2,
  kNumericalFailure = 3,
};

struct SolverResult
{
  Eigen::VectorXd x;
  SolverStatus status{SolverStatus::kInvalidProblem};
  int iterations{0};
  double objective{std::numeric_limits<double>::quiet_NaN()};
  double primal_residual{std::numeric_limits<double>::infinity()};
  double kkt_residual{std::numeric_limits<double>::infinity()};
  double update_us{0.0};
  double solve_us{0.0};
};

struct MpcConfig
{
  int horizon_steps{20};
  double horizon_dt{0.05};
  Eigen::Matrix<double, kStateDim, kStateDim> Q{
    (Eigen::Matrix<double, kStateDim, 1>() <<
      15000.0, 15000.0, 15000.0, 40.0, 40.0, 40.0, 80.0, 80.0, 80.0).finished().asDiagonal()};
  Eigen::Matrix<double, kStateDim, kStateDim> P{Q};
  Eigen::Matrix<double, kInputDim, kInputDim> R{
    (Eigen::Matrix<double, kInputDim, 1>() << 0.5, 0.6, 0.6, 0.6).finished().asDiagonal()};
  double gravity{9.81};
  double thrust_acceleration_min{0.0};
  double thrust_acceleration_max{30.0};
  Eigen::Vector3d body_rate_max{Eigen::Vector3d::Constant(6.0)};
  // qpDUNES stops early when it converges; keep a generous ceiling so valid
  // online weight/bound changes are not rejected by an artificial limit.
  int solver_max_iterations{4000};
  double solver_tolerance{1e-9};
  double admm_rho{1.0};
  bool warm_start{true};
  // Optional diagnostic rejection against the exact condensed QP. Rejection
  // never invokes another solver. Online flight leaves this disabled so the
  // selected backend is evaluated directly.
  bool verify_solution{false};
  std::string dataset_path;
};

struct MpcResult
{
  Input command;
  SolverResult solver;
  double manifold_us{0.0};
  double linearization_us{0.0};
  double qp_build_us{0.0};
  double total_us{0.0};
  bool fallback_used{false};
  SolverStatus candidate_status{SolverStatus::kInvalidProblem};
  double candidate_primal_residual{std::numeric_limits<double>::infinity()};
  double candidate_kkt_scaled{std::numeric_limits<double>::infinity()};
  bool command_valid{false};
};

}  // namespace lu_ommpc

#endif  // LU_OMMPC__TYPES_HPP_
