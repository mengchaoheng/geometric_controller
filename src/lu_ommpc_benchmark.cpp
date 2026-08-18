// Standalone benchmark executable installed by geometric_controller.
#include <Eigen/Dense>

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <numeric>
#include <stdexcept>
#include <string>
#include <vector>

#include "lu_ommpc/core.hpp"
#include "lu_ommpc/qp_dataset.hpp"

namespace
{

struct Options
{
  std::string solver{"all"};
  std::string preset{"paper"};
  std::string mode{"solver"};
  std::string dataset_in;
  std::string dataset_out;
  std::string csv_path;
  int samples{2000};
  int warmup{200};
  int horizon_steps{-1};
  double horizon_dt{-1.0};
  bool cold_start{false};
};

struct Measurements
{
  std::vector<double> update_us;
  std::vector<double> solve_us;
  std::vector<double> total_us;
  std::vector<double> build_us;
  int failures{0};
  int deadline_misses{0};
  double max_primal{0.0};
  double max_kkt{0.0};
  bool has_primal{false};
  bool has_kkt{false};
};

struct ValidationMeasurements
{
  int count{0};
  int failures{0};
  double max_objective_relative{0.0};
  double max_first_control_inf{0.0};
  double max_solution_inf{0.0};
  double max_primal{0.0};
  double max_kkt{0.0};
  double max_kkt_scaled{0.0};
  bool passed{true};
};

Options parseOptions(int argc, char ** argv)
{
  Options options;
  for (int i = 1; i < argc; ++i) {
    const std::string argument(argv[i]);
    auto value = [&](const std::string & name) {
        if (i + 1 >= argc) {
          throw std::invalid_argument("missing value after " + name);
        }
        return std::string(argv[++i]);
      };
    if (argument == "--solver") {
      options.solver = value(argument);
    } else if (argument == "--preset") {
      options.preset = value(argument);
    } else if (argument == "--mode") {
      options.mode = value(argument);
    } else if (argument == "--samples") {
      options.samples = std::stoi(value(argument));
    } else if (argument == "--warmup") {
      options.warmup = std::stoi(value(argument));
    } else if (argument == "--horizon") {
      options.horizon_steps = std::stoi(value(argument));
    } else if (argument == "--dt") {
      options.horizon_dt = std::stod(value(argument));
    } else if (argument == "--dataset-in") {
      options.dataset_in = value(argument);
    } else if (argument == "--dataset-out") {
      options.dataset_out = value(argument);
    } else if (argument == "--csv") {
      options.csv_path = value(argument);
    } else if (argument == "--cold-start") {
      options.cold_start = true;
    } else if (argument == "--help" || argument == "-h") {
      std::cout <<
        "lu_ommpc_benchmark [--mode solver|mpc|replay|validate] "
        "[--solver all|osqp|qpoases|proxqp|"
        "daqp|piqp|qpswift|hpipm|ooqp|hpipm_ocp|qpdunes|tinympc_lti|"
        "tinympc_lti_cached|cvxpygen_osqp]\n"
        "  [--preset paper|one_second|one_second_100hz|main_comparison] "
        "[--samples N] [--warmup N]\n"
        "  [--horizon N] [--dt SECONDS]\n"
        "  [--cold-start] [--dataset-in FILE] [--dataset-out FILE] [--csv FILE]\n";
      std::exit(0);
    } else {
      throw std::invalid_argument("unknown option: " + argument);
    }
  }
  if (options.samples < 1 || options.warmup < 0 || options.warmup >= options.samples) {
    throw std::invalid_argument("require samples > warmup >= 0");
  }
  if ((options.horizon_steps != -1 && options.horizon_steps <= 0) ||
    (options.horizon_dt != -1.0 && options.horizon_dt <= 0.0))
  {
    throw std::invalid_argument("horizon and dt overrides must be positive");
  }
  return options;
}

lu_ommpc::MpcConfig makeConfig(const Options & options)
{
  lu_ommpc::MpcConfig config;
  if (options.preset == "main_comparison") {
    config.horizon_steps = 20;
    config.horizon_dt = 0.05;
    config.Q.block<3, 3>(6, 6) *= 2.0;
    config.P = config.Q;
    config.R *= 20.0;
  } else if (options.preset == "one_second") {
    config.horizon_steps = 20;
    config.horizon_dt = 0.05;
  } else if (options.preset == "one_second_100hz") {
    config.horizon_steps = 100;
    config.horizon_dt = 0.01;
    config.solver_max_iterations = 1000;
    config.solver_tolerance = 1e-9;
  } else if (options.preset != "paper") {
    throw std::invalid_argument("unknown preset: " + options.preset);
  }
  if (options.horizon_steps > 0) {config.horizon_steps = options.horizon_steps;}
  if (options.horizon_dt > 0.0) {config.horizon_dt = options.horizon_dt;}
  return config;
}

lu_ommpc::ReferenceHorizon makeReferenceHorizon(
  double time_s, const lu_ommpc::MpcConfig & config)
{
  constexpr double radius = 1.3;
  constexpr double omega = 1.2;
  lu_ommpc::ReferenceHorizon horizon;
  horizon.reserve(static_cast<std::size_t>(config.horizon_steps + 1));
  for (int k = 0; k <= config.horizon_steps; ++k) {
    const double t = time_s + static_cast<double>(k) * config.horizon_dt;
    const double phase = omega * t;
    lu_ommpc::FlatOutput flat;
    flat.position = Eigen::Vector3d(
      radius * std::cos(phase), radius * std::sin(phase), -3.0);
    flat.velocity = Eigen::Vector3d(
      -radius * omega * std::sin(phase), radius * omega * std::cos(phase), 0.0);
    flat.acceleration = Eigen::Vector3d(
      -radius * omega * omega * std::cos(phase),
      -radius * omega * omega * std::sin(phase), 0.0);
    flat.jerk = Eigen::Vector3d(
      radius * std::pow(omega, 3) * std::sin(phase),
      -radius * std::pow(omega, 3) * std::cos(phase), 0.0);
    flat.snap = Eigen::Vector3d(
      radius * std::pow(omega, 4) * std::cos(phase),
      radius * std::pow(omega, 4) * std::sin(phase), 0.0);
    flat.yaw = phase + 0.5 * std::acos(-1.0);
    flat.yaw_rate = omega;
    horizon.push_back(lu_ommpc::flatnessReference(flat, config.gravity));
  }
  return horizon;
}

lu_ommpc::State perturbedState(
  const lu_ommpc::ReferenceKnot & reference, double time_s)
{
  lu_ommpc::State state = reference.state;
  state.position += Eigen::Vector3d(
    0.20 * std::sin(0.73 * time_s),
    -0.15 * std::cos(0.51 * time_s),
    0.08 * std::sin(0.37 * time_s));
  state.velocity += Eigen::Vector3d(
    0.12 * std::cos(0.73 * time_s),
    0.08 * std::sin(0.51 * time_s),
    -0.04 * std::cos(0.37 * time_s));
  state.rotation = state.rotation * lu_ommpc::expSO3(Eigen::Vector3d(
      0.04 * std::sin(0.41 * time_s),
      -0.05 * std::cos(0.31 * time_s),
      0.03 * std::sin(0.27 * time_s)));
  return state;
}

std::vector<lu_ommpc::QPSnapshot> makeDataset(
  const Options & options, const lu_ommpc::MpcConfig & config)
{
  std::vector<lu_ommpc::QPSnapshot> snapshots;
  snapshots.reserve(static_cast<std::size_t>(options.samples));
  if (!options.dataset_in.empty()) {
    lu_ommpc::QPDatasetReader reader(options.dataset_in);
    if (!reader.good()) {
      throw std::runtime_error("cannot open QP dataset: " + options.dataset_in);
    }
    lu_ommpc::QPSnapshot snapshot;
    while (static_cast<int>(snapshots.size()) < options.samples && reader.read(snapshot)) {
      snapshots.push_back(snapshot);
    }
    if (static_cast<int>(snapshots.size()) <= options.warmup) {
      throw std::runtime_error("dataset has too few records");
    }
    if (!options.dataset_out.empty()) {
      lu_ommpc::QPDatasetWriter writer(options.dataset_out);
      if (!writer.good()) {
        throw std::runtime_error("cannot create QP dataset: " + options.dataset_out);
      }
      for (const auto & recorded : snapshots) {
        if (!writer.write(recorded)) {
          throw std::runtime_error("failed while copying QP dataset");
        }
      }
    }
    return snapshots;
  }

  lu_ommpc::QPBuilder builder(config);
  for (int i = 0; i < options.samples; ++i) {
    const double time_s = 0.01 * i;
    const auto horizon = makeReferenceHorizon(time_s, config);
    lu_ommpc::QPSnapshot snapshot;
    snapshot.timestamp_us = static_cast<uint64_t>(i) * 10000U;
    snapshot.state = perturbedState(horizon.front(), time_s);
    snapshot.reference = horizon;
    snapshot.has_mpc_input = true;
    snapshot.problem = builder.build(snapshot.state, horizon);
    snapshots.push_back(std::move(snapshot));
  }
  if (!options.dataset_out.empty()) {
    lu_ommpc::QPDatasetWriter writer(options.dataset_out);
    if (!writer.good()) {
      throw std::runtime_error("cannot create QP dataset: " + options.dataset_out);
    }
    for (const auto & snapshot : snapshots) {
      if (!writer.write(snapshot)) {
        throw std::runtime_error("failed while writing QP dataset");
      }
    }
  }
  return snapshots;
}

double percentile(std::vector<double> values, double probability)
{
  if (values.empty()) {
    return std::numeric_limits<double>::quiet_NaN();
  }
  std::sort(values.begin(), values.end());
  const double index = probability * static_cast<double>(values.size() - 1);
  const std::size_t lower = static_cast<std::size_t>(std::floor(index));
  const std::size_t upper = static_cast<std::size_t>(std::ceil(index));
  const double alpha = index - static_cast<double>(lower);
  return (1.0 - alpha) * values[lower] + alpha * values[upper];
}

double mean(const std::vector<double> & values)
{
  return values.empty() ? std::numeric_limits<double>::quiet_NaN() :
         std::accumulate(values.begin(), values.end(), 0.0) / values.size();
}

void updateResiduals(Measurements & measurement, const lu_ommpc::SolverResult & result)
{
  if (std::isfinite(result.primal_residual)) {
    measurement.max_primal = std::max(measurement.max_primal, result.primal_residual);
    measurement.has_primal = true;
  }
  if (std::isfinite(result.kkt_residual)) {
    measurement.max_kkt = std::max(measurement.max_kkt, result.kkt_residual);
    measurement.has_kkt = true;
  }
}

void printStats(const std::string & solver, const Measurements & measurement)
{
  const auto & values = measurement.total_us;
  std::cout << std::fixed << std::setprecision(3)
            << solver
            << " count=" << values.size()
            << " mean_us=" << mean(values)
            << " p50_us=" << percentile(values, 0.50)
            << " p90_us=" << percentile(values, 0.90)
            << " p95_us=" << percentile(values, 0.95)
            << " p99_us=" << percentile(values, 0.99)
            << " p99.9_us=" << percentile(values, 0.999)
            << " max_us=" << (values.empty() ? 0.0 : *std::max_element(values.begin(), values.end()))
            << " update_mean_us=" << mean(measurement.update_us)
            << " solve_mean_us=" << mean(measurement.solve_us)
            << " build_mean_us=" << mean(measurement.build_us)
            << " deadline_miss=" << measurement.deadline_misses
            << " failures=" << measurement.failures
            << " max_primal=" << std::scientific <<
    (measurement.has_primal ? measurement.max_primal : std::numeric_limits<double>::quiet_NaN())
            << " max_kkt=" <<
    (measurement.has_kkt ? measurement.max_kkt : std::numeric_limits<double>::quiet_NaN())
            << std::fixed << '\n';
}

std::vector<std::string> selectedSolvers(const std::string & option)
{
  if (option == "all") {
    return lu_ommpc::availableSolvers();
  }
  return {option};
}

std::vector<std::string> selectedValidationSolvers(const std::string & option)
{
  if (option == "all") {
    // Exact, persistent implementations with credible embedded potential. Approximate LTI
    // substitutions and one-shot reference adapters are intentionally excluded here.
    return {"qpoases", "hpipm_ocp", "qpdunes", "hpipm", "proxqp", "daqp"};
  }
  return {option};
}

Eigen::VectorXd shiftSolution(const Eigen::VectorXd & x)
{
  if (x.size() < lu_ommpc::kInputDim) {
    return Eigen::VectorXd();
  }
  Eigen::VectorXd shifted(x.size());
  shifted.head(x.size() - lu_ommpc::kInputDim) = x.tail(x.size() - lu_ommpc::kInputDim);
  shifted.tail(lu_ommpc::kInputDim) = x.tail(lu_ommpc::kInputDim);
  return shifted;
}

std::vector<lu_ommpc::SolverResult> solveSequence(
  const std::string & solver_name, const Options & options,
  const lu_ommpc::MpcConfig & config,
  const std::vector<lu_ommpc::QPSnapshot> & snapshots)
{
  auto solver = lu_ommpc::makeSolver(solver_name, config);
  std::vector<lu_ommpc::SolverResult> results;
  results.reserve(snapshots.size());
  Eigen::VectorXd previous;
  for (const auto & snapshot : snapshots) {
    lu_ommpc::SolverResult result;
    if (solver->update(snapshot.problem)) {
      if (options.cold_start) {
        solver->clearWarmStart();
      } else {
        const Eigen::VectorXd warm = shiftSolution(previous);
        if (warm.size() == snapshot.problem.g.size()) {
          solver->warmStart(warm);
        } else {
          solver->clearWarmStart();
        }
      }
      result = solver->solve();
      if (result.status == lu_ommpc::SolverStatus::kSolved) {
        previous = result.x;
      }
    }
    results.push_back(std::move(result));
  }
  return results;
}

ValidationMeasurements validateSolver(
  const std::string & solver_name, const Options & options,
  const lu_ommpc::MpcConfig & config,
  const std::vector<lu_ommpc::QPSnapshot> & snapshots,
  const std::vector<lu_ommpc::SolverResult> & references)
{
  constexpr double kObjectiveRelativeLimit = 1e-7;
  constexpr double kFirstControlLimit = 1e-4;
  constexpr double kSolutionLimit = 1e-3;
  constexpr double kPrimalLimit = 1e-7;
  constexpr double kScaledKktLimit = 1e-4;

  const auto candidates = solver_name == "qpoases" ? references :
    solveSequence(solver_name, options, config, snapshots);
  ValidationMeasurements measurement;
  for (std::size_t i = static_cast<std::size_t>(options.warmup); i < snapshots.size(); ++i) {
    const auto & reference = references[i];
    const auto & candidate = candidates[i];
    ++measurement.count;
    if (reference.status != lu_ommpc::SolverStatus::kSolved ||
      candidate.status != lu_ommpc::SolverStatus::kSolved ||
      reference.x.size() != candidate.x.size() || reference.x.size() < lu_ommpc::kInputDim)
    {
      ++measurement.failures;
      continue;
    }
    const auto & problem = snapshots[i].problem;
    const double reference_objective = lu_ommpc::qpObjective(problem, reference.x);
    const double candidate_objective = lu_ommpc::qpObjective(problem, candidate.x);
    const double objective_relative = std::abs(candidate_objective - reference_objective) /
      std::max(1.0, std::abs(reference_objective));
    const double first_control =
      (candidate.x.head(lu_ommpc::kInputDim) -
      reference.x.head(lu_ommpc::kInputDim)).lpNorm<Eigen::Infinity>();
    const double solution = (candidate.x - reference.x).lpNorm<Eigen::Infinity>();
    const double primal = lu_ommpc::qpPrimalResidual(problem, candidate.x);
    const double kkt = lu_ommpc::qpBoxKktResidual(problem, candidate.x);
    const Eigen::VectorXd gradient = problem.H * candidate.x + problem.g;
    const double gradient_scale = std::max(
      1.0, std::max(gradient.lpNorm<Eigen::Infinity>(),
      problem.g.lpNorm<Eigen::Infinity>()));
    const double kkt_scaled = kkt / gradient_scale;
    if (!std::isfinite(objective_relative) || !std::isfinite(first_control) ||
      !std::isfinite(solution) || !std::isfinite(primal) || !std::isfinite(kkt) ||
      !std::isfinite(kkt_scaled))
    {
      ++measurement.failures;
      continue;
    }
    measurement.max_objective_relative =
      std::max(measurement.max_objective_relative, objective_relative);
    measurement.max_first_control_inf =
      std::max(measurement.max_first_control_inf, first_control);
    measurement.max_solution_inf = std::max(measurement.max_solution_inf, solution);
    measurement.max_primal = std::max(measurement.max_primal, primal);
    measurement.max_kkt = std::max(measurement.max_kkt, kkt);
    measurement.max_kkt_scaled = std::max(measurement.max_kkt_scaled, kkt_scaled);
  }
  measurement.passed = measurement.failures == 0 &&
    measurement.max_objective_relative <= kObjectiveRelativeLimit &&
    measurement.max_first_control_inf <= kFirstControlLimit &&
    measurement.max_solution_inf <= kSolutionLimit &&
    measurement.max_primal <= kPrimalLimit &&
    measurement.max_kkt_scaled <= kScaledKktLimit;
  return measurement;
}

void printValidation(const std::string & solver, const ValidationMeasurements & measurement)
{
  std::cout << solver << " validation=" << (measurement.passed ? "PASS" : "FAIL")
            << " count=" << measurement.count << " failures=" << measurement.failures
            << std::scientific << std::setprecision(6)
            << " max_objective_relative=" << measurement.max_objective_relative
            << " max_first_control_inf=" << measurement.max_first_control_inf
            << " max_solution_inf=" << measurement.max_solution_inf
            << " max_primal=" << measurement.max_primal
            << " max_kkt=" << measurement.max_kkt
            << " max_kkt_scaled=" << measurement.max_kkt_scaled << std::fixed << '\n';
}

Measurements runSolverBenchmark(
  const std::string & solver_name, const Options & options,
  const lu_ommpc::MpcConfig & config,
  const std::vector<lu_ommpc::QPSnapshot> & snapshots, std::ofstream * csv)
{
  auto solver = lu_ommpc::makeSolver(solver_name, config);
  Measurements measurements;
  Eigen::VectorXd previous;
  for (std::size_t i = 0; i < snapshots.size(); ++i) {
    const auto & snapshot = snapshots[i];
    if (!solver->update(snapshot.problem)) {
      ++measurements.failures;
      continue;
    }
    if (options.cold_start) {
      solver->clearWarmStart();
    } else if (snapshot.warm_start.size() == snapshot.problem.g.size()) {
      solver->warmStart(snapshot.warm_start);
    } else {
      const Eigen::VectorXd warm = shiftSolution(previous);
      if (warm.size() == snapshot.problem.g.size()) {
        solver->warmStart(warm);
      } else {
        solver->clearWarmStart();
      }
    }
    const auto result = solver->solve();
    if (result.status == lu_ommpc::SolverStatus::kSolved) {
      previous = result.x;
    } else {
      ++measurements.failures;
    }
    if (static_cast<int>(i) >= options.warmup) {
      const double online = result.update_us + result.solve_us;
      measurements.update_us.push_back(result.update_us);
      measurements.solve_us.push_back(result.solve_us);
      measurements.total_us.push_back(online);
      measurements.deadline_misses += online > 10000.0 ? 1 : 0;
      updateResiduals(measurements, result);
      if (csv != nullptr) {
        *csv << solver->name() << ',' << i << ',' << result.update_us << ','
             << result.solve_us << ',' << online << ",0," << result.iterations << ','
             << static_cast<int>(result.status) << ',' << result.objective << ','
             << result.primal_residual << ',' << result.kkt_residual << '\n';
      }
    }
  }
  return measurements;
}

Measurements runMpcBenchmark(
  const std::string & solver_name, const Options & options,
  const lu_ommpc::MpcConfig & config, std::ofstream * csv)
{
  lu_ommpc::OMMPCController controller(config, solver_name);
  Measurements measurements;
  for (int i = 0; i < options.samples; ++i) {
    if (options.cold_start) {
      controller.reset();
    }
    const double time_s = 0.01 * i;
    const auto horizon = makeReferenceHorizon(time_s, config);
    const auto result = controller.solve(perturbedState(horizon.front(), time_s), horizon);
    if (!result.command_valid) {
      ++measurements.failures;
    }
    if (i >= options.warmup) {
      const double build = result.manifold_us + result.linearization_us + result.qp_build_us;
      measurements.update_us.push_back(result.solver.update_us);
      measurements.solve_us.push_back(result.solver.solve_us);
      measurements.build_us.push_back(build);
      measurements.total_us.push_back(result.total_us);
      measurements.deadline_misses += result.total_us > 1.0e6 * config.horizon_dt ? 1 : 0;
      updateResiduals(measurements, result.solver);
      if (csv != nullptr) {
        *csv << controller.solverName() << ',' << i << ',' << result.solver.update_us << ','
             << result.solver.solve_us << ',' << result.total_us << ',' << build << ','
             << result.solver.iterations << ',' << static_cast<int>(result.solver.status) << ','
             << result.solver.objective << ',' << result.solver.primal_residual << ','
             << result.solver.kkt_residual << '\n';
      }
    }
  }
  return measurements;
}

Measurements runReplayBenchmark(
  const std::string & solver_name, const Options & options,
  const lu_ommpc::MpcConfig & config,
  const std::vector<lu_ommpc::QPSnapshot> & snapshots, std::ofstream * csv)
{
  lu_ommpc::OMMPCController controller(config, solver_name);
  Measurements measurements;
  for (std::size_t i = 0; i < snapshots.size(); ++i) {
    if (!snapshots[i].has_mpc_input ||
      snapshots[i].reference.size() != static_cast<std::size_t>(config.horizon_steps + 1))
    {
      throw std::runtime_error("replay dataset does not contain matching state/reference inputs");
    }
    if (options.cold_start) {controller.reset();}
    const auto result = controller.solve(snapshots[i].state, snapshots[i].reference);
    if (!result.command_valid) {++measurements.failures;}
    if (static_cast<int>(i) >= options.warmup) {
      const double build = result.manifold_us + result.linearization_us + result.qp_build_us;
      measurements.update_us.push_back(result.solver.update_us);
      measurements.solve_us.push_back(result.solver.solve_us);
      measurements.build_us.push_back(build);
      measurements.total_us.push_back(result.total_us);
      measurements.deadline_misses += result.total_us > 1.0e6 * config.horizon_dt ? 1 : 0;
      updateResiduals(measurements, result.solver);
      if (csv != nullptr) {
        *csv << controller.solverName() << ',' << i << ',' << result.solver.update_us << ','
             << result.solver.solve_us << ',' << result.total_us << ',' << build << ','
             << result.solver.iterations << ',' << static_cast<int>(result.solver.status) << ','
             << result.solver.objective << ',' << result.solver.primal_residual << ','
             << result.solver.kkt_residual << '\n';
      }
    }
  }
  return measurements;
}

}  // namespace

int main(int argc, char ** argv)
{
  try {
    const Options options = parseOptions(argc, argv);
    const auto config = makeConfig(options);
    std::ofstream csv;
    if (!options.csv_path.empty()) {
      csv.open(options.csv_path, std::ios::trunc);
      if (!csv) {
        throw std::runtime_error("cannot create CSV: " + options.csv_path);
      }
      csv << "solver,sample,update_us,solve_us,total_us,build_us,iterations,status,"
             "objective,primal_residual,kkt_residual\n";
      csv << std::setprecision(15);
    }

    std::vector<lu_ommpc::QPSnapshot> dataset;
    if (options.mode == "solver" || options.mode == "replay" || options.mode == "validate") {
      if (options.mode == "replay" && options.dataset_in.empty()) {
        throw std::invalid_argument("replay mode requires --dataset-in");
      }
      dataset = makeDataset(options, config);
    } else if (options.mode != "mpc") {
      throw std::invalid_argument("mode must be solver, mpc, replay, or validate");
    }

    std::cout << "mode=" << options.mode << " preset=" << options.preset
              << " N=" << config.horizon_steps << " dt=" << config.horizon_dt
              << " start=" << (options.cold_start ? "cold" : "warm") << '\n';
    if (options.mode == "validate") {
      // The reference is deliberately tighter than production. Candidates retain the same
      // settings used by timed MPC runs, so a fast path cannot hide behind extra validation work.
      auto reference_config = config;
      reference_config.solver_tolerance = std::min(reference_config.solver_tolerance, 1e-9);
      reference_config.solver_max_iterations = std::max(reference_config.solver_max_iterations, 2000);
      const auto references = solveSequence("qpoases", options, reference_config, dataset);
      bool all_passed = true;
      for (const auto & solver : selectedValidationSolvers(options.solver)) {
        const auto measurement = validateSolver(solver, options, config, dataset, references);
        printValidation(solver, measurement);
        all_passed = all_passed && measurement.passed;
      }
      return all_passed ? 0 : 2;
    }
    for (const auto & solver : selectedSolvers(options.solver)) {
      const Measurements measurements = options.mode == "solver" ?
        runSolverBenchmark(solver, options, config, dataset, csv ? &csv : nullptr) :
        (options.mode == "replay" ?
        runReplayBenchmark(solver, options, config, dataset, csv ? &csv : nullptr) :
        runMpcBenchmark(solver, options, config, csv ? &csv : nullptr));
      printStats(solver, measurements);
    }
  } catch (const std::exception & error) {
    std::cerr << "benchmark error: " << error.what() << '\n';
    return 1;
  }
  return 0;
}
