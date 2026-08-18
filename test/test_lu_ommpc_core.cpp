// Unit tests for the embedded Lu OMMPC core.
#include <gtest/gtest.h>

#include "lu_ommpc/core.hpp"
#include "geometric_controller/controllers/lu_ommpc_controller.hpp"

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
    const double t = k * config.horizon_dt;
    const double phase = omega * t;
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
  lu_ommpc::FlatOutput flat;
  const auto reference = lu_ommpc::flatnessReference(flat, 9.81);
  Eigen::Matrix<double, lu_ommpc::kStateDim, lu_ommpc::kStateDim> A;
  Eigen::Matrix<double, lu_ommpc::kStateDim, lu_ommpc::kInputDim> B;
  lu_ommpc::linearizeErrorDynamics(reference, 0.01, A, B);
  EXPECT_TRUE((A.block<3, 3>(0, 3).isApprox(0.01 * Eigen::Matrix3d::Identity())));
  EXPECT_TRUE((B.block<3, 1>(3, 0).isApprox(-0.01 * Eigen::Vector3d::UnitZ())));
  EXPECT_TRUE((B.block<3, 3>(6, 1).isApprox(0.01 * Eigen::Matrix3d::Identity())));
}

TEST(Solvers, AgreeOnBoxConstrainedOptimum)
{
  lu_ommpc::QPProblem problem;
  problem.H = (Eigen::Vector3d(2.0, 4.0, 8.0)).asDiagonal();
  problem.g = Eigen::Vector3d(-4.0, 8.0, -4.0);
  problem.A = Eigen::Matrix3d::Identity();
  problem.lower = Eigen::Vector3d(-1.0, -1.0, -0.25);
  problem.upper = Eigen::Vector3d(1.0, 1.0, 0.25);
  const Eigen::Vector3d expected(1.0, -1.0, 0.25);
  lu_ommpc::MpcConfig config;
  config.solver_max_iterations = 2000;
  config.solver_tolerance = 1e-8;
  for (const auto & name : lu_ommpc::availableExternalSolvers()) {
    auto solver = lu_ommpc::makeSolver(name, config);
    ASSERT_TRUE(solver->update(problem));
    const auto result = solver->solve();
    EXPECT_EQ(result.status, lu_ommpc::SolverStatus::kSolved) << name;
    EXPECT_TRUE(result.x.isApprox(expected, 1e-5)) << name << ": " << result.x.transpose();
    EXPECT_LE(result.primal_residual, 1e-8) << name;
  }
}

TEST(QPBuilder, PaperPresetHasExpectedDimensions)
{
  lu_ommpc::MpcConfig config;
  const auto reference = hoverHorizon(config);
  lu_ommpc::QPBuilder builder(config);
  const auto problem = builder.build(reference.front().state, reference);
  EXPECT_EQ(problem.H.rows(), 32);
  EXPECT_EQ(problem.A.rows(), 32);
  EXPECT_TRUE(problem.H.isApprox(problem.H.transpose(), 1e-12));
  EXPECT_TRUE((problem.lower.array() <= problem.upper.array()).all());
}

TEST(QPBuilder, StageAccumulationMatchesExplicitCondensing)
{
  lu_ommpc::MpcConfig config;
  auto reference = hoverHorizon(config);
  auto state = reference.front().state;
  state.position = Eigen::Vector3d(0.03, -0.01, -1.96);
  state.velocity = Eigen::Vector3d(-0.02, 0.01, 0.04);
  lu_ommpc::QPBuilder builder(config);
  const auto problem = builder.build(state, reference);
  const int N = config.horizon_steps;
  const int nx_stack = N * lu_ommpc::kStateDim;
  const int nu_stack = N * lu_ommpc::kInputDim;
  Eigen::MatrixXd Hx = Eigen::MatrixXd::Zero(nx_stack, lu_ommpc::kStateDim);
  Eigen::MatrixXd Mu = Eigen::MatrixXd::Zero(nx_stack, nu_stack);
  Eigen::MatrixXd MuRow = Eigen::MatrixXd::Zero(lu_ommpc::kStateDim, nu_stack);
  Eigen::Matrix<double, lu_ommpc::kStateDim, lu_ommpc::kStateDim> transition =
    Eigen::Matrix<double, lu_ommpc::kStateDim, lu_ommpc::kStateDim>::Identity();
  for (int k = 0; k < N; ++k) {
    transition = problem.ocp.A[k] * transition;
    MuRow = problem.ocp.A[k] * MuRow;
    MuRow.block(0, k * lu_ommpc::kInputDim, lu_ommpc::kStateDim, lu_ommpc::kInputDim) =
      problem.ocp.B[k];
    Hx.block(k * lu_ommpc::kStateDim, 0, lu_ommpc::kStateDim, lu_ommpc::kStateDim) =
      transition;
    Mu.block(k * lu_ommpc::kStateDim, 0, lu_ommpc::kStateDim, nu_stack) = MuRow;
  }
  Eigen::MatrixXd Qbar = Eigen::MatrixXd::Zero(nx_stack, nx_stack);
  Eigen::MatrixXd Rbar = Eigen::MatrixXd::Zero(nu_stack, nu_stack);
  for (int k = 0; k < N; ++k) {
    Qbar.block<lu_ommpc::kStateDim, lu_ommpc::kStateDim>(
      k * lu_ommpc::kStateDim, k * lu_ommpc::kStateDim) =
      k == N - 1 ? config.P : config.Q;
    Rbar.block<lu_ommpc::kInputDim, lu_ommpc::kInputDim>(
      k * lu_ommpc::kInputDim, k * lu_ommpc::kInputDim) = config.R;
  }
  Eigen::MatrixXd expectedH = Mu.transpose() * Qbar * Mu + Rbar;
  expectedH.diagonal().array() += 1e-9;
  const Eigen::VectorXd expectedG = Mu.transpose() * Qbar * Hx *
    lu_ommpc::stateError(state, reference.front().state);
  EXPECT_TRUE(problem.H.isApprox(expectedH, 1e-10));
  EXPECT_TRUE(problem.g.isApprox(expectedG, 1e-10));
}

TEST(QPBuilder, BuildsOnlyTheRepresentationRequestedBySolver)
{
  lu_ommpc::MpcConfig config;
  const auto reference = hoverHorizon(config);
  lu_ommpc::QPBuilder builder(config);
  lu_ommpc::QPProblem problem;
  builder.buildInto(
    reference.front().state, reference, lu_ommpc::QPBuildMode::kCondensed, problem);
  EXPECT_EQ(problem.H.rows(), config.horizon_steps * lu_ommpc::kInputDim);
  EXPECT_FALSE(problem.ocp.valid());
  builder.buildInto(
    reference.front().state, reference, lu_ommpc::QPBuildMode::kOcp, problem);
  EXPECT_EQ(problem.H.rows(), 0);
  EXPECT_TRUE(problem.ocp.valid());
}

TEST(StructuredSolvers, ExactPathsAgreeWithCondensedQp)
{
  lu_ommpc::MpcConfig config;
  config.solver_max_iterations = 4000;
  config.solver_tolerance = 1e-7;
  const auto reference = hoverHorizon(config);
  auto state = reference.front().state;
  state.position = Eigen::Vector3d(0.04, -0.02, -1.97);
  state.velocity = Eigen::Vector3d(0.01, -0.02, 0.03);
  state.rotation = lu_ommpc::expSO3(Eigen::Vector3d(0.01, -0.015, 0.005));
  lu_ommpc::QPBuilder builder(config);
  const auto problem = builder.build(state, reference);

  auto baseline_solver = lu_ommpc::makeSolver("qpoases", config);
  ASSERT_TRUE(baseline_solver->update(problem));
  const auto baseline = baseline_solver->solve();
  ASSERT_EQ(baseline.status, lu_ommpc::SolverStatus::kSolved);

  for (const std::string name : {"hpipm_ocp", "qpdunes", "cvxpygen_osqp"}) {
    auto solver = lu_ommpc::makeSolver(name, config);
    ASSERT_TRUE(solver->update(problem)) << name;
    const auto result = solver->solve();
    ASSERT_EQ(result.status, lu_ommpc::SolverStatus::kSolved) << name;
    const double tolerance = name == "cvxpygen_osqp" ? 2e-3 : 2e-4;
    EXPECT_TRUE(result.x.isApprox(baseline.x, tolerance))
      << name << " max error=" << (result.x - baseline.x).lpNorm<Eigen::Infinity>();
    EXPECT_LE(result.primal_residual, 2e-5) << name;
  }
}

TEST(StructuredSolvers, TinyMpcPathsAreExplicitlyFeasibleApproximations)
{
  lu_ommpc::MpcConfig config;
  config.solver_max_iterations = 1000;
  const auto reference = hoverHorizon(config);
  auto state = reference.front().state;
  state.position.x() = 0.02;
  lu_ommpc::QPBuilder builder(config);
  const auto problem = builder.build(state, reference);
  for (const std::string name : {"tinympc_lti", "tinympc_lti_cached"}) {
    auto solver = lu_ommpc::makeSolver(name, config);
    ASSERT_TRUE(solver->update(problem)) << name;
    const auto result = solver->solve();
    ASSERT_EQ(result.status, lu_ommpc::SolverStatus::kSolved) << name;
    ASSERT_EQ(result.x.size(), config.horizon_steps * lu_ommpc::kInputDim);
    EXPECT_LE(result.primal_residual, 2e-4) << name;
  }
}

TEST(StructuredSolvers, OneSecondOcpPathsMatchCondensedControl)
{
  lu_ommpc::MpcConfig config;
  config.horizon_steps = 20;
  config.horizon_dt = 0.05;
  config.solver_max_iterations = 2000;
  config.solver_tolerance = 1e-7;
  const auto reference = circleHorizon(config);
  auto state = reference.front().state;
  state.position += Eigen::Vector3d(0.05, -0.03, 0.02);
  state.velocity += Eigen::Vector3d(-0.02, 0.01, 0.03);
  state.rotation *= lu_ommpc::expSO3(Eigen::Vector3d(0.01, -0.02, 0.005));
  lu_ommpc::QPBuilder builder(config);
  const auto problem = builder.build(state, reference);
  auto baseline_solver = lu_ommpc::makeSolver("qpoases", config);
  ASSERT_TRUE(baseline_solver->update(problem));
  const auto baseline = baseline_solver->solve();
  ASSERT_EQ(baseline.status, lu_ommpc::SolverStatus::kSolved);
  for (const std::string name : {"hpipm_ocp", "qpdunes"}) {
    auto solver = lu_ommpc::makeSolver(name, config);
    ASSERT_TRUE(solver->update(problem)) << name;
    const auto result = solver->solve();
    ASSERT_EQ(result.status, lu_ommpc::SolverStatus::kSolved) << name;
    EXPECT_TRUE(result.x.head<lu_ommpc::kInputDim>().isApprox(
      baseline.x.head<lu_ommpc::kInputDim>(), 3e-3)) << name;
    EXPECT_NEAR(result.objective, baseline.objective,
      2e-5 * std::max(1.0, std::abs(baseline.objective))) << name;
    EXPECT_LE(result.primal_residual, 1e-7) << name;
    EXPECT_LE(result.kkt_residual, 2e-2) << name;
  }
}

TEST(ClosedLoop, NedCommandDirectionsAreCorrect)
{
  lu_ommpc::MpcConfig config;
  const auto reference = hoverHorizon(config);

  lu_ommpc::State state = reference.front().state;
  state.position.z() = 0.0;
  lu_ommpc::OMMPCController altitude_controller(config, "active_set");
  const auto altitude = altitude_controller.solve(state, reference);
  ASSERT_TRUE(altitude.command_valid);
  EXPECT_GT(altitude.command.thrust_acceleration, config.gravity);
  EXPECT_NEAR(altitude.command.body_rate.x(), 0.0, 1e-10);
  EXPECT_NEAR(altitude.command.body_rate.y(), 0.0, 1e-10);

  state = reference.front().state;
  state.position.x() = 0.1;
  lu_ommpc::OMMPCController north_controller(config, "active_set");
  const auto north = north_controller.solve(state, reference);
  ASSERT_TRUE(north.command_valid);
  EXPECT_GT(north.command.body_rate.y(), 0.0);

  state = reference.front().state;
  state.position.y() = 0.1;
  lu_ommpc::OMMPCController east_controller(config, "active_set");
  const auto east = east_controller.solve(state, reference);
  ASSERT_TRUE(east.command_valid);
  EXPECT_LT(east.command.body_rate.x(), 0.0);

  state = reference.front().state;
  state.rotation = lu_ommpc::expSO3(Eigen::Vector3d(0.1, 0.0, 0.0));
  lu_ommpc::OMMPCController attitude_controller(config, "active_set");
  const auto attitude = attitude_controller.solve(state, reference);
  ASSERT_TRUE(attitude.command_valid);
  EXPECT_LT(attitude.command.body_rate.x(), 0.0);
}

TEST(ClosedLoop, IdealRatePlantConvergesToHover)
{
  lu_ommpc::MpcConfig config;
  const auto reference = hoverHorizon(config);

  lu_ommpc::State state;
  // Lu linearizes once around the reference and assumes small tracking error;
  // takeoff and large setpoint transitions belong outside this controller.
  state.position = Eigen::Vector3d(0.02, -0.01, -1.98);
  state.rotation = lu_ommpc::expSO3(Eigen::Vector3d(0.003, -0.002, 0.001));
  lu_ommpc::OMMPCController controller(config, "active_set");
  constexpr double dt = 0.01;
  for (int k = 0; k < 1000; ++k) {
    const auto result = controller.solve(state, reference);
    ASSERT_TRUE(result.command_valid);
    const Eigen::Vector3d acceleration = config.gravity * Eigen::Vector3d::UnitZ() -
      result.command.thrust_acceleration * state.rotation * Eigen::Vector3d::UnitZ();
    state.position += dt * state.velocity + 0.5 * dt * dt * acceleration;
    state.velocity += dt * acceleration;
    state.rotation = state.rotation * lu_ommpc::expSO3(dt * result.command.body_rate);
  }
  EXPECT_LT((state.position - reference.front().state.position).norm(), 0.03);
  EXPECT_LT(state.velocity.norm(), 0.03);
  EXPECT_LT(lu_ommpc::logSO3(state.rotation).norm(), 0.02);
}

TEST(ControllerAdapter, SolvesEveryPositionFeedbackEvent)
{
  geometric_controller::VehicleState state;
  state.position.z() = -2.0;
  geometric_controller::FlatReference reference;
  reference.position.z() = -2.0;
  geometric_controller::ControllerParams params;
  params.ommpc_solver = "osqp";

  geometric_controller::LuOMMPCController controller;
  controller.reset(state);
  int solve_count = 0;
  for (int sample = 0; sample < 25; ++sample) {
    const auto command = controller.update(state, reference, params, 0.004);
    solve_count += command.solution_updated ? 1 : 0;
    EXPECT_TRUE(command.body_rate_control);
    EXPECT_TRUE(command.valid);
  }
  EXPECT_EQ(solve_count, 25);
}

TEST(FlightSafety, ExactOnlineSolversRemainEquivalentAfterParameterChanges)
{
  std::vector<lu_ommpc::MpcConfig> configurations(5);
  configurations[0].horizon_steps = 8;
  configurations[0].horizon_dt = 0.05;
  configurations[1].horizon_steps = 8;
  configurations[1].horizon_dt = 0.01;
  configurations[2].horizon_steps = 20;
  configurations[2].horizon_dt = 0.05;
  configurations[3].horizon_steps = 50;
  configurations[3].horizon_dt = 0.02;
  configurations[4].horizon_steps = 100;
  configurations[4].horizon_dt = 0.01;

  // Exercise every online-adjustable part of the mathematical problem, not
  // merely the nominal paper weights.
  for (std::size_t i = 1; i < configurations.size(); ++i) {
    auto & config = configurations[i];
    config.Q.block<3, 3>(0, 0) *= 0.55 + 0.12 * static_cast<double>(i);
    config.Q.block<3, 3>(3, 3) *= 1.55 - 0.11 * static_cast<double>(i);
    config.Q.block<3, 3>(6, 6) *= 0.70 + 0.09 * static_cast<double>(i);
    config.P = config.Q;
    config.R *= 0.75 + 0.28 * static_cast<double>(i);
    config.thrust_acceleration_min = 0.4 * static_cast<double>(i);
    config.thrust_acceleration_max = 30.0 - 1.5 * static_cast<double>(i);
    config.body_rate_max = Eigen::Vector3d(
      5.0 - 0.2 * static_cast<double>(i),
      4.7 - 0.15 * static_cast<double>(i),
      3.8 - 0.1 * static_cast<double>(i));
  }

  const std::vector<std::string> online_solvers{
    "qpoases", "daqp", "hpipm", "piqp", "qpswift", "osqp", "ooqp",
    "hpipm_ocp", "qpdunes"};
  for (auto config : configurations) {
    config.solver_tolerance = 1e-9;
    config.solver_max_iterations = 4000;
    const auto reference = circleHorizon(config);
    lu_ommpc::State state = reference.front().state;
    state.position += Eigen::Vector3d(0.15, -0.11, 0.07);
    state.velocity += Eigen::Vector3d(-0.08, 0.06, -0.03);
    state.rotation *= lu_ommpc::expSO3(Eigen::Vector3d(0.03, -0.02, 0.01));

    auto trusted_config = config;
    trusted_config.verify_solution = false;
    lu_ommpc::OMMPCController trusted(trusted_config, "qpoases");
    const auto expected = trusted.solve(state, reference);
    ASSERT_TRUE(expected.command_valid);

    config.verify_solution = true;
    for (const auto & solver : online_solvers) {
      // The node deliberately does not offer condensed solvers above N=50:
      // correct offline solves are not enough when they miss the 100 Hz input
      // cadence. N=100 is an online-supported structured-solver configuration.
      if (config.horizon_steps > 50 && solver != "hpipm_ocp" && solver != "qpdunes") {
        continue;
      }
      lu_ommpc::OMMPCController controller(config, solver);
      const auto actual = controller.solve(state, reference);
      ASSERT_TRUE(actual.command_valid) << solver << " N=" << config.horizon_steps;
      ASSERT_FALSE(actual.fallback_used) << solver << " N=" << config.horizon_steps;
      ASSERT_EQ(actual.solver.status, lu_ommpc::SolverStatus::kSolved)
        << solver << " N=" << config.horizon_steps;
      ASSERT_EQ(actual.solver.x.size(), expected.solver.x.size())
        << solver << " N=" << config.horizon_steps;
      const double objective_relative = std::abs(
        actual.solver.objective - expected.solver.objective) /
        std::max(1.0, std::abs(expected.solver.objective));
      EXPECT_LE(objective_relative, 1e-7) << solver << " N=" << config.horizon_steps;
      EXPECT_LE(
        (actual.solver.x.head<lu_ommpc::kInputDim>() -
        expected.solver.x.head<lu_ommpc::kInputDim>()).lpNorm<Eigen::Infinity>(), 1e-4)
        << solver << " N=" << config.horizon_steps;
      EXPECT_LE((actual.solver.x - expected.solver.x).lpNorm<Eigen::Infinity>(), 1e-3)
        << solver << " N=" << config.horizon_steps;
      EXPECT_LE(actual.candidate_primal_residual, 1e-7)
        << solver << " N=" << config.horizon_steps;
      EXPECT_LE(actual.candidate_kkt_scaled, 1e-4)
        << solver << " N=" << config.horizon_steps;
    }
  }
}

TEST(FlightSafety, RejectedSolverNeverInvokesFallback)
{
  lu_ommpc::MpcConfig config;
  config.verify_solution = true;
  const auto reference = circleHorizon(config);
  lu_ommpc::State state = reference.front().state;
  state.position += Eigen::Vector3d(0.2, -0.1, 0.05);
  state.rotation *= lu_ommpc::expSO3(Eigen::Vector3d(0.04, -0.03, 0.02));
  lu_ommpc::OMMPCController controller(config, "tinympc_lti");
  const auto result = controller.solve(state, reference);
  EXPECT_FALSE(result.fallback_used);
  EXPECT_FALSE(result.command_valid);
}

TEST(StructuredSolvers, RawOcpControllerDoesNotRequireCondensedVectors)
{
  lu_ommpc::MpcConfig config;
  config.horizon_steps = 20;
  config.horizon_dt = 0.05;
  config.verify_solution = false;
  const auto reference = circleHorizon(config);
  lu_ommpc::State state = reference.front().state;
  state.position += Eigen::Vector3d(0.1, -0.08, 0.04);
  for (const auto * solver : {"hpipm_ocp", "qpdunes"}) {
    lu_ommpc::OMMPCController controller(config, solver);
    const auto result = controller.solve(state, reference);
    EXPECT_TRUE(result.command_valid) << solver;
    EXPECT_FALSE(result.fallback_used) << solver;
    EXPECT_TRUE(result.solver.x.size() == config.horizon_steps * lu_ommpc::kInputDim)
      << solver;
  }
}

TEST(ControllerAdapter, OnlineSolverAndMpcParameterChangesRebuildSafely)
{
  geometric_controller::VehicleState state;
  state.position = Eigen::Vector3d(0.1, -0.08, -2.1);
  geometric_controller::FlatReference reference;
  reference.position.z() = -2.0;
  geometric_controller::ControllerParams params;
  const std::vector<std::string> solvers{
    "qpoases", "daqp", "hpipm", "piqp", "qpswift", "osqp", "ooqp",
    "hpipm_ocp", "qpdunes"};
  geometric_controller::LuOMMPCController controller;
  for (std::size_t index = 0; index < solvers.size(); ++index) {
    params.ommpc_solver = solvers[index];
    params.ommpc_horizon_steps = index % 2U == 0U ? 8 : 20;
    params.ommpc_horizon_dt = index % 2U == 0U ? 0.01 : 0.05;
    params.ommpc_position_weight_scale = 0.7 + 0.05 * static_cast<double>(index);
    params.ommpc_velocity_weight_scale = 1.2;
    params.ommpc_attitude_weight_scale = 0.9;
    params.ommpc_input_weight_scale = 1.4;
    params.ommpc_thrust_acceleration_max = 30.0;
    params.ommpc_body_rate_max = Eigen::Vector3d(4.0, 4.5, 3.5);
    const auto command = controller.update(state, reference, params, 0.01);
    EXPECT_TRUE(command.valid) << solvers[index];
    EXPECT_TRUE(command.desired_body_rate.allFinite()) << solvers[index];
    EXPECT_TRUE(std::isfinite(command.collective_thrust)) << solvers[index];
  }
}

TEST(ClosedLoop, EveryOnlineSolverRunsLongWithoutFallbackAcrossParameterChanges)
{
  const std::vector<std::string> solvers{
    "qpoases", "daqp", "hpipm", "piqp", "qpswift", "osqp", "ooqp",
    "hpipm_ocp", "qpdunes"};
  constexpr double plant_dt = 0.01;
  for (const auto & solver : solvers) {
    geometric_controller::VehicleState state;
    state.position = Eigen::Vector3d(0.02, -0.01, -1.98);
    state.attitude = Eigen::Vector4d(1.0, 0.0, 0.0, 0.0);
    geometric_controller::ControllerParams params;
    params.ommpc_solver = solver;
    params.ommpc_tolerance = 1e-9;
    geometric_controller::LuOMMPCController controller;

    for (int step = 0; step < 600; ++step) {
      if (step == 200) {
        params.ommpc_horizon_steps = 20;
        params.ommpc_horizon_dt = 0.02;
        params.ommpc_position_weight_scale = 0.75;
        params.ommpc_velocity_weight_scale = 1.25;
        params.ommpc_attitude_weight_scale = 0.9;
        params.ommpc_input_weight_scale = 1.5;
        params.ommpc_thrust_acceleration_max = 30.0;
        params.ommpc_body_rate_max = Eigen::Vector3d(4.0, 4.0, 3.0);
      } else if (step == 400) {
        params.ommpc_horizon_steps = 8;
        params.ommpc_horizon_dt = 0.01;
        params.ommpc_position_weight_scale = 1.1;
        params.ommpc_velocity_weight_scale = 0.9;
        params.ommpc_attitude_weight_scale = 1.2;
        params.ommpc_input_weight_scale = 0.8;
      }

      geometric_controller::FlatReference reference;
      reference.position.z() = -2.0;
      geometric_controller::FlatReferenceKnot knot;
      knot.position.z() = -2.0;
      reference.horizon.assign(
        static_cast<std::size_t>(params.ommpc_horizon_steps + 1), knot);
      const auto command = controller.update(state, reference, params, plant_dt);
      ASSERT_TRUE(command.valid) << solver << " step=" << step;
      ASSERT_FALSE(command.solver_fallback_used) << solver << " step=" << step;
      ASSERT_TRUE(command.desired_body_rate.allFinite()) << solver << " step=" << step;
      ASSERT_TRUE(std::isfinite(command.collective_thrust)) << solver << " step=" << step;

      Eigen::Quaterniond attitude(
        state.attitude[0], state.attitude[1], state.attitude[2], state.attitude[3]);
      attitude.normalize();
      const Eigen::Matrix3d rotation = attitude.toRotationMatrix();
      const double thrust_acceleration = command.collective_thrust / params.mass;
      const Eigen::Vector3d acceleration = params.gravity -
        thrust_acceleration * rotation * Eigen::Vector3d::UnitZ();
      state.position += plant_dt * state.velocity +
        0.5 * plant_dt * plant_dt * acceleration;
      state.velocity += plant_dt * acceleration;
      attitude = attitude * Eigen::Quaterniond(
        lu_ommpc::expSO3(plant_dt * command.desired_body_rate));
      attitude.normalize();
      state.attitude = Eigen::Vector4d(
        attitude.w(), attitude.x(), attitude.y(), attitude.z());
    }
    EXPECT_LT((state.position - Eigen::Vector3d(0.0, 0.0, -2.0)).norm(), 0.04) << solver;
    EXPECT_LT(state.velocity.norm(), 0.04) << solver;
  }
}
