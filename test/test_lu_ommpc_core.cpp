// Unit tests for the Lu OMMPC core and the five retained solver paths.
#include <gtest/gtest.h>

#include <cmath>
#include <stdexcept>
#include <utility>

#include "geometric_controller/controllers/lu_ommpc_controller.hpp"
#include "lu_ommpc/core.hpp"

namespace
{

lu_ommpc::ReferenceHorizon hoverHorizon(const lu_ommpc::MpcConfig & config)
{
  lu_ommpc::FlatOutput flat;
  flat.position.z() = -2.0;
  const auto knot = lu_ommpc::flatnessReference(flat, config.gravity);
  return lu_ommpc::ReferenceHorizon(
    static_cast<std::size_t>(config.horizon_steps + 1), knot);
}

lu_ommpc::ReferenceHorizon circleHorizon(const lu_ommpc::MpcConfig & config)
{
  constexpr double radius = 1.2;
  constexpr double omega = 0.7;
  lu_ommpc::ReferenceHorizon horizon;
  horizon.reserve(static_cast<std::size_t>(config.horizon_steps + 1));
  for (int k = 0; k <= config.horizon_steps; ++k) {
    const double phase = omega * k * config.horizon_dt;
    lu_ommpc::FlatOutput flat;
    flat.position = Eigen::Vector3d(radius * std::cos(phase), radius * std::sin(phase), -2.0);
    flat.velocity = Eigen::Vector3d(
      -radius * omega * std::sin(phase), radius * omega * std::cos(phase), 0.0);
    flat.acceleration = Eigen::Vector3d(
      -radius * omega * omega * std::cos(phase),
      -radius * omega * omega * std::sin(phase), 0.0);
    flat.jerk = Eigen::Vector3d(
      radius * omega * omega * omega * std::sin(phase),
      -radius * omega * omega * omega * std::cos(phase), 0.0);
    flat.yaw = phase;
    flat.yaw_rate = omega;
    horizon.push_back(lu_ommpc::flatnessReference(flat, config.gravity));
  }
  return horizon;
}

lu_ommpc::MpcResult solveOcp(const lu_ommpc::MpcConfig & config)
{
  const auto reference = circleHorizon(config);
  auto state = reference.front().state;
  state.position += Eigen::Vector3d(0.04, -0.02, 0.03);
  state.velocity += Eigen::Vector3d(0.01, -0.02, 0.02);
  state.rotation *= lu_ommpc::expSO3(Eigen::Vector3d(0.01, -0.015, 0.005));
  lu_ommpc::OMMPCController controller(config, "qpdunes");
  return controller.solve(state, reference);
}

}  // namespace

TEST(SO3, ExpLogRoundTrip)
{
  const Eigen::Vector3d phi(0.4, -0.2, 0.7);
  EXPECT_TRUE(lu_ommpc::logSO3(lu_ommpc::expSO3(phi)).isApprox(phi, 1e-10));
}

TEST(Flatness, HoverMatchesNedModel)
{
  lu_ommpc::FlatOutput flat;
  flat.position.z() = -2.0;
  const auto reference = lu_ommpc::flatnessReference(flat, 9.81);
  EXPECT_NEAR(reference.input.thrust_acceleration, 9.81, 1e-12);
  EXPECT_TRUE(reference.input.body_rate.isZero(1e-12));
  EXPECT_TRUE(reference.state.rotation.isApprox(Eigen::Matrix3d::Identity(), 1e-12));
}

TEST(Linearization, HasLuQuadrotorBlocks)
{
  const auto reference = lu_ommpc::flatnessReference(lu_ommpc::FlatOutput{}, 9.81);
  Eigen::Matrix<double, lu_ommpc::kStateDim, lu_ommpc::kStateDim> A;
  Eigen::Matrix<double, lu_ommpc::kStateDim, lu_ommpc::kInputDim> B;
  lu_ommpc::linearizeErrorDynamics(reference, 0.01, A, B);
  EXPECT_TRUE((A.block<3, 3>(0, 3).isApprox(0.01 * Eigen::Matrix3d::Identity())));
  EXPECT_TRUE((B.block<3, 1>(3, 0).isApprox(-0.01 * Eigen::Vector3d::UnitZ())));
  EXPECT_TRUE((B.block<3, 3>(6, 1).isApprox(0.01 * Eigen::Matrix3d::Identity())));
}

TEST(Solvers, RetainedBackendsAreExposed)
{
  EXPECT_EQ(lu_ommpc::availableSolvers(), std::vector<std::string>(
    {"qpdunes", "hpipm_ocp", "qpoases", "osqp", "daqp"}));
  lu_ommpc::MpcConfig config;
  EXPECT_NO_THROW(lu_ommpc::makeSolver("qpdunes", config));
  EXPECT_NO_THROW(lu_ommpc::makeSolver("hpipm_ocp", config));
  EXPECT_NO_THROW(lu_ommpc::makeSolver("qpoases", config));
  EXPECT_NO_THROW(lu_ommpc::makeSolver("osqp", config));
  EXPECT_NO_THROW(lu_ommpc::makeSolver("daqp", config));
  EXPECT_THROW(lu_ommpc::makeSolver("tinympc_lti", config), std::invalid_argument);
}

TEST(QPBuilder, CondensedAndOcpRepresentationsAreDistinct)
{
  lu_ommpc::MpcConfig config;
  const auto reference = hoverHorizon(config);
  lu_ommpc::QPBuilder builder(config);
  lu_ommpc::QPProblem problem;
  builder.buildInto(reference.front().state, reference, lu_ommpc::QPBuildMode::kCondensed, problem);
  EXPECT_EQ(problem.H.rows(), config.horizon_steps * lu_ommpc::kInputDim);
  EXPECT_FALSE(problem.ocp.valid());
  builder.buildInto(reference.front().state, reference, lu_ommpc::QPBuildMode::kOcp, problem);
  EXPECT_EQ(problem.H.rows(), 0);
  EXPECT_TRUE(problem.ocp.valid());
  EXPECT_EQ(problem.ocp.A.size(), static_cast<std::size_t>(config.horizon_steps));
  EXPECT_EQ(problem.ocp.lower_u.size(), problem.ocp.A.size());
}

TEST(StructuredSolver, QpDunesConsumesRawOcpAndReturnsFeasibleCommand)
{
  lu_ommpc::MpcConfig config;
  config.horizon_steps = 20;
  config.horizon_dt = 0.05;
  const auto result = solveOcp(config);
  ASSERT_TRUE(result.command_valid);
  EXPECT_FALSE(result.fallback_used);
  EXPECT_EQ(result.solver.status, lu_ommpc::SolverStatus::kSolved);
  EXPECT_EQ(result.solver.x.size(), config.horizon_steps * lu_ommpc::kInputDim);
  EXPECT_TRUE(result.command.vector().allFinite());
  EXPECT_LE(result.solver.primal_residual, 1e-8);
  EXPECT_GE(result.command.thrust_acceleration, config.thrust_acceleration_min - 1e-9);
  EXPECT_LE(result.command.thrust_acceleration, config.thrust_acceleration_max + 1e-9);
  EXPECT_TRUE((result.command.body_rate.array().abs() <=
    config.body_rate_max.array() + 1e-8).all());
}

TEST(StructuredSolver, HpipmConsumesRawOcpAndReturnsFeasibleCommand)
{
  lu_ommpc::MpcConfig config;
  config.horizon_steps = 8;
  config.horizon_dt = 0.05;
  const auto reference = circleHorizon(config);
  const auto state = reference.front().state;
  lu_ommpc::QPBuilder builder(config);
  lu_ommpc::QPProblem problem;
  builder.buildInto(state, reference, lu_ommpc::QPBuildMode::kOcp, problem);

  auto solver = lu_ommpc::makeSolver("hpipm_ocp", config);
  ASSERT_NE(solver, nullptr);
  ASSERT_TRUE(solver->update(problem));
  const auto result = solver->solve();
  EXPECT_EQ(result.status, lu_ommpc::SolverStatus::kSolved);
  EXPECT_EQ(result.x.size(), config.horizon_steps * lu_ommpc::kInputDim);
  EXPECT_TRUE(result.x.allFinite());
  EXPECT_LE(result.primal_residual, 1e-7);
}

TEST(BaselineSolvers, CondensedQpOasesOsqpAndDaqpSolveTheSameProblem)
{
  lu_ommpc::MpcConfig config;
  config.horizon_steps = 8;
  config.horizon_dt = 0.05;
  const auto reference = hoverHorizon(config);
  lu_ommpc::QPBuilder builder(config);
  lu_ommpc::QPProblem problem;
  builder.buildInto(
    reference.front().state, reference, lu_ommpc::QPBuildMode::kCondensed, problem);
  for (const auto & name : {
      std::string("qpoases"), std::string("osqp"), std::string("daqp")}) {
    auto solver = lu_ommpc::makeSolver(name, config);
    ASSERT_TRUE(solver->update(problem)) << name;
    const auto result = solver->solve();
    ASSERT_EQ(result.status, lu_ommpc::SolverStatus::kSolved) << name;
    EXPECT_EQ(result.x.size(), config.horizon_steps * lu_ommpc::kInputDim) << name;
    EXPECT_LE(result.primal_residual, 2e-5) << name;
    EXPECT_TRUE(result.x.allFinite()) << name;
  }
}

TEST(ClosedLoop, QpDunesPreservesNedCommandDirections)
{
  lu_ommpc::MpcConfig config;
  const auto reference = hoverHorizon(config);
  lu_ommpc::State state = reference.front().state;
  state.position.z() = 0.0;
  auto altitude = lu_ommpc::OMMPCController(config, "qpdunes").solve(state, reference);
  ASSERT_TRUE(altitude.command_valid);
  EXPECT_GT(altitude.command.thrust_acceleration, config.gravity);

  state = reference.front().state;
  state.position.x() = 0.1;
  auto north = lu_ommpc::OMMPCController(config, "qpdunes").solve(state, reference);
  ASSERT_TRUE(north.command_valid);
  EXPECT_GT(north.command.body_rate.y(), 0.0);

  state = reference.front().state;
  state.position.y() = 0.1;
  auto east = lu_ommpc::OMMPCController(config, "qpdunes").solve(state, reference);
  ASSERT_TRUE(east.command_valid);
  EXPECT_LT(east.command.body_rate.x(), 0.0);
}

TEST(ControllerAdapter, UsesQpDunesOnEveryFeedbackEvent)
{
  geometric_controller::VehicleState state;
  state.position.z() = -2.0;
  geometric_controller::FlatReference reference;
  reference.position.z() = -2.0;
  geometric_controller::ControllerParams params;
  params.ommpc_solver = "qpdunes";
  geometric_controller::LuOMMPCController controller;
  controller.reset(state);
  for (int sample = 0; sample < 10; ++sample) {
    const auto command = controller.update(state, reference, params, 0.01);
    EXPECT_TRUE(command.valid);
    EXPECT_TRUE(command.body_rate_control);
    EXPECT_FALSE(command.solver_fallback_used);
    EXPECT_TRUE(command.desired_body_rate.allFinite());
  }
}

TEST(ControllerAdapter, RebuildsSafelyWhenSupportedParametersChange)
{
  geometric_controller::VehicleState state;
  state.position = Eigen::Vector3d(0.1, -0.08, -2.1);
  geometric_controller::FlatReference reference;
  reference.position.z() = -2.0;
  geometric_controller::ControllerParams params;
  params.ommpc_solver = "qpdunes";
  geometric_controller::LuOMMPCController controller;
  controller.reset(state);
  const std::vector<std::pair<int, double>> cases{{20, 0.05}, {20, 0.03}, {25, 0.025}};
  for (const auto & item : cases) {
    params.ommpc_horizon_steps = item.first;
    params.ommpc_horizon_dt = item.second;
    params.ommpc_position_weight_scale = 0.9;
    params.ommpc_velocity_weight_scale = 1.1;
    params.ommpc_attitude_weight_scale = 0.95;
    params.ommpc_input_weight_scale = 1.2;
    params.ommpc_body_rate_max = Eigen::Vector3d(5.0, 5.0, 4.0);
    const auto command = controller.update(state, reference, params, 0.01);
    EXPECT_TRUE(command.valid) << "N=" << item.first << " dt=" << item.second;
    EXPECT_FALSE(command.solver_fallback_used);
    EXPECT_TRUE(command.desired_body_rate.allFinite());
    EXPECT_TRUE(std::isfinite(command.collective_thrust));
  }
}

TEST(ClosedLoop, IdealRatePlantRemainsFiniteWithQpDunes)
{
  lu_ommpc::MpcConfig config;
  const auto reference = hoverHorizon(config);
  lu_ommpc::State state = reference.front().state;
  state.position += Eigen::Vector3d(0.02, -0.01, 0.02);
  state.rotation = lu_ommpc::expSO3(Eigen::Vector3d(0.003, -0.002, 0.001));
  lu_ommpc::OMMPCController controller(config, "qpdunes");
  constexpr double plant_dt = 0.01;
  for (int k = 0; k < 200; ++k) {
    const auto result = controller.solve(state, reference);
    ASSERT_TRUE(result.command_valid) << "step=" << k;
    ASSERT_FALSE(result.fallback_used) << "step=" << k;
    const Eigen::Vector3d acceleration = config.gravity * Eigen::Vector3d::UnitZ() -
      result.command.thrust_acceleration * state.rotation * Eigen::Vector3d::UnitZ();
    state.position += plant_dt * state.velocity + 0.5 * plant_dt * plant_dt * acceleration;
    state.velocity += plant_dt * acceleration;
    state.rotation = state.rotation * lu_ommpc::expSO3(plant_dt * result.command.body_rate);
  }
  EXPECT_TRUE(state.position.allFinite());
  EXPECT_TRUE(state.velocity.allFinite());
  EXPECT_TRUE(state.rotation.allFinite());
}
