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

namespace FitHoleSolver {

namespace bg = boost::geometry;
using BoostPoint = bg::model::d2::point_xy<double>;
using BoostPolygon = bg::model::polygon<BoostPoint>;
using BoostLinestring = bg::model::linestring<BoostPoint>;

template <typename T>
void shrink(T& point_a, T& point_b) {
  constexpr double eps = 1e-6;
  auto shrink_a = point_a * (1.0 - eps) + point_b * eps;
  auto shrink_b = point_b * (1.0 - eps) + point_a * eps;
  point_a = shrink_a;
  point_b = shrink_b;
}

template <typename T>
BoostPoint ToBoostPoint(const T& point) {
  const auto [x, y] = point;
  return BoostPoint(x, y);
}

template <typename T>
BoostPolygon ToBoostPolygon(const std::vector<T>& points) {
  BoostPolygon polygon;
  for (std::size_t i = 0; i <= points.size(); ++i) {
    polygon.outer().push_back(ToBoostPoint(points[i % points.size()]));
  }
  if (bg::area(polygon) < 0.0) {
    bg::reverse(polygon);
  }
  return polygon;
}

template <typename T, typename U>
double SquaredDistance(const T& vertex0, const U& vertex1) {
  const auto [x0, y0] = vertex0;
  const auto [x1, y1] = vertex1;
  return (x0 - x1) * (x0 - x1) + (y0 - y1) * (y0 - y1);
}

template <typename T>
double SquaredEdgeLength(const T& vertices, const Edge& edge) {
  const auto [a, b] = edge;
  return SquaredDistance(vertices[a], vertices[b]);
}


class Solver : public SolverBase {
 public:
  SolverOutputs solve(const SolverArguments& args) override {
    hole_ = args.problem->hole_polygon;
    vertices_ = args.problem->vertices;
    edges_ = args.problem->edges;
    epsilon_ = args.problem->epsilon;
    hole_polygon_ = ToBoostPolygon(hole_);

    SVisualEditorPtr editor;
    if (args.visualize) {
      editor = std::make_shared<SVisualEditor>(args.problem, "FitHoleSolver", "visualize");
    }

    const int N = vertices_.size();
    auto pose = vertices_;
    double cost = std::numeric_limits<double>::infinity();
    bool found_fit_in_hole = false;
    std::vector<Point> best_feasible_pose;

    SPinnedIndex pinned_index(rng_, N, editor);

    const int num_iters = 10000;
    const double T0 = 1.0e1;
    const double T1 = 1.0e-2;
    double progress = 0.0;

    auto evaluate_and_descide_rollback = [&]() -> bool {
      auto [feasible, updated_cost] = Evaluate(pose);

      auto res = judge(*args.problem, pose);
      if (res.fit_in_hole()) {
        found_fit_in_hole = true;
      }

      const double T = std::pow(T0, 1.0 - progress) * std::pow(T1, progress);
      if (std::uniform_real_distribution(0.0, 1.0)(rng_) < std::exp(-(updated_cost - cost) / T)) {
        cost = updated_cost;
        return false; // accepted
      } else {
        return true; // rejected
      }
    };
    evaluate_and_descide_rollback();

    auto single_small_change = [&] { // ynasu87 original
      const int v = pinned_index.sample_movable_index();
      const int dx = std::uniform_int_distribution(-5, 5)(rng_);
      const int dy = std::uniform_int_distribution(-5, 5)(rng_);
      auto& [x, y] = pose[v];
      x += dx;
      y += dy;
      if (evaluate_and_descide_rollback()) {
        x -= dx;
        y -= dy;
      }
    };

    auto shift = [&] {
      const int dx = std::uniform_int_distribution(-2, 2)(rng_);
      const int dy = std::uniform_int_distribution(-2, 2)(rng_);
      auto pose_bak = pose;
      for (int i : pinned_index.movable_indices) {
        pose[i].first += dx;
        pose[i].second += dy;
      }
      if (evaluate_and_descide_rollback()) {
        pose = pose_bak;
      }
    };

    using Action = std::function<void()>;
    std::vector<std::pair<double, Action>> action_probs = {
      {0.9, single_small_change},
      {0.01, shift},
    };
    {
      // normalize probs
      double p = 0.0;
      for (int i = 0; i < action_probs.size(); ++i) { p += action_probs[i].first; }
      for (int i = 0; i < action_probs.size(); ++i) { action_probs[i].first /= p; }
    }

    for (int iter = 0; iter < num_iters; ++iter) {
      progress = 1.0 * iter / num_iters;

      if (found_fit_in_hole) {
        LOG(INFO) << fmt::format("found fit in hole iter = {}", iter);
        break;
      }

      if (iter < 100) {
        shift();
      } else {
        const double p_action = std::uniform_real_distribution(0.0, 1.0)(rng_);
        double p_accum = 0.0;
        for (int i = 0; i < action_probs.size(); ++i) { 
          p_accum += action_probs[i].first;
          if (p_action < p_accum) {
            action_probs[i].second();
          }
        }
      }

      if (editor && iter % 100 == 0) {
        editor->set_oneshot_custom_stat(fmt::format("iter = {}/{}", iter, num_iters));
        editor->set_pose(args.problem->create_solution(pose));
        if (auto show_result = editor->show(1); show_result.edit_result) {
          pose = show_result.edit_result->pose_after_edit->vertices;
          pinned_index.update_movable_index();
        }
      }

    }

    SolverOutputs outputs;
    outputs.solution = args.problem->create_solution(pose);

    return outputs;
  }

  template <typename P>
  std::tuple<bool, double> Evaluate(const std::vector<P>& pose) const {
    double deformation_cost = 0.0;
    double protrusion_cost = 0.0;
    double dislikes_cost = 0.0;

    const double tolerance = epsilon_ / 1'000'000.0;
    for (const auto& vertex : pose) {
      protrusion_cost += bg::distance(ToBoostPoint(vertex), hole_polygon_);
    }
    for (const auto& edge : edges_) {
      const auto [a, b] = edge;
      Point2d pa = pose[a];
      Point2d pb = pose[b];
      shrink(pa, pb);
      BoostLinestring linestring{ToBoostPoint(pa), ToBoostPoint(pb)};
      std::vector<BoostLinestring> differences;
      bg::difference(linestring, hole_polygon_, differences);
      for (const auto& segment : differences) {
        protrusion_cost += 1.0e1 * bg::length(segment);
      }

      const auto d0 = SquaredEdgeLength(vertices_, edge);
      const auto d1 = SquaredEdgeLength(pose, edge);
      deformation_cost += 1.0e1 * std::max(0.0, std::abs(d1 / d0 - 1.0) - tolerance);
    }

    const bool feasible = deformation_cost + protrusion_cost == 0.0;
    const double cost = deformation_cost + protrusion_cost + dislikes_cost;

    return {feasible, cost};
  }

 private:
  std::mt19937 rng_;
  std::vector<Point> hole_;
  std::vector<Point> vertices_;
  std::vector<Edge> edges_;
  integer epsilon_;
  BoostPolygon hole_polygon_;
};

}

REGISTER_SOLVER("FitHoleSolver", FitHoleSolver::Solver);
// vim:ts=2 sw=2 sts=2 et ci
