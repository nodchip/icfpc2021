#include "stdafx.h"
#include <cmath>
#include <fmt/format.h>
#include <boost/geometry.hpp>
#include <boost/geometry/geometries/point_xy.hpp>
#include <boost/geometry/geometries/polygon.hpp>
#include "contest_types.h"
#include "solver_registry.h"
#include "visual_editor.h"
#include "solver_util.h"
#include "judge.h"
#include "timer.h"

namespace RandomRestartSolver {

std::vector<Point> enumerate_interior_points(const SProblem& problem) {
  integer ymin = INT_MAX, ymax = INT_MIN;
  integer xmin = INT_MAX, xmax = INT_MIN;
  for (auto p : problem.hole_polygon) {
    chmin(xmin, get_x(p));
    chmin(ymin, get_y(p));
    chmax(xmax, get_x(p));
    chmax(ymax, get_y(p));
  }

  std::vector<Point> points;
  for (int y = ymin; y <= ymax; ++y) {
    for (int x = xmin; x <= xmax; ++x) {
      if (contains(problem.hole_polygon, {x, y}) != EContains::EOUT) { // include points on the edge/vertex of the hole.
        points.emplace_back(x, y);
      }
    }
  }

  LOG(INFO) << fmt::format("found {} interior points", points.size());
  return points;
}

class Solver : public SolverBase {
 public:
  SolverOutputs solve(const SolverArguments& args) override {
    auto seed = std::random_device()();
    LOG(INFO) << fmt::format("seed: {}", seed);
    rng.seed(seed);

    constexpr int num_random_restarts = 2;
    const auto interior_points = enumerate_interior_points(*args.problem);

    SSolutionPtr best_solution = args.problem->create_solution();
    integer best_dislikes = std::numeric_limits<integer>::max();

    for (int i = 0; i < num_random_restarts; ++i) {
      LOG(INFO) << fmt::format("solve {}/{}", i, num_random_restarts);
      auto init_pose = args.problem->create_solution();
      if (i % 2 == 0) {
        // random init
        for (auto& v : init_pose->vertices) {
          v = interior_points[std::uniform_int_distribution<size_t>(0ull, interior_points.size())(rng)];
        }
      }

      auto solver = SolverRegistry::getSolver("HopGridAnnealingSolver");
      CHECK(solver);
      SolverArguments sub_args = args;
      sub_args.optional_initial_solution = init_pose;
      sub_args.random_seed = rng();
      Timer t;
      SolverOutputs trial_res = solver->solve(sub_args);

      auto trial_judge = judge(*args.problem, *trial_res.solution);
      LOG(INFO) << fmt::format("solve {}/{} .. elapsed {} ms. is_valid={}, fit_hole={}, DL={}",
        i, num_random_restarts,
        t.elapsed_ms(), trial_judge.is_valid(), trial_judge.fit_in_hole(), trial_judge.dislikes);

      if (trial_judge.is_valid() && trial_judge.dislikes < best_dislikes) {
        const std::string filename = args.problem->problem_id ? fmt::format("{}.bestsofar.pose.json", *args.problem->problem_id) : "bestsofar.pose.json";
        save_solution(args.problem, trial_res.solution, "RandomRestartSolver", filename);

        LOG(INFO) << fmt::format("@@@@@@ trial {} update best {} -> {}. saved: {}", i, best_dislikes, trial_judge.dislikes, filename);
        best_solution = trial_res.solution;
        best_dislikes = trial_judge.dislikes;
      }
    }

    SolverOutputs outputs;
    outputs.solution = best_solution;

    return outputs;
  }

 private:
  std::mt19937 rng;
};

}

REGISTER_SOLVER("RandomRestartSolver", RandomRestartSolver::Solver);
// vim:ts=2 sw=2 sts=2 et ci
