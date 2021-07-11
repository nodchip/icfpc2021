#include "stdafx.h"
#include <boost/geometry.hpp>
#include <boost/geometry/geometries/point_xy.hpp>
#include <boost/geometry/geometries/polygon.hpp>
#include "contest_types.h"
#include "solver_registry.h"
#include "visual_editor.h"
#include "judge.h"

namespace ResetTempertureAnnealingSolver {

namespace bg = boost::geometry;
using BoostPoint = bg::model::d2::point_xy<double>;
using BoostPolygon = bg::model::polygon<BoostPoint>;
using BoostLinestring = bg::model::linestring<BoostPoint>;

/// <summary>
/// 評価関数の結果を表す
/// </summary>
struct EvaluationResult {
    /// <summary>
    /// 有効な解の場合true、そうでない場合はfalse
    /// </summary>
    bool feasible;

    /// <summary>
    /// 評価関数の出力したコスト
    /// </summary>
    double cost;

    /// <summary>
    /// 外部にはみ出した部分がある場合
    /// </summary>
    bool has_protrusion;
};

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
      editor = std::make_shared<SVisualEditor>(args.problem, "ResetTempertureAnnealingSolver", "visualize");
    }

    const int N = vertices_.size();
    auto pose = vertices_;
    double cost = std::numeric_limits<double>::infinity();
    double best_feasible_cost = std::numeric_limits<double>::infinity();
    std::vector<Point> best_feasible_pose;

    const int num_iters = 100000;
    // 最後に温度をリセットしたイテレーション
    int last_reset_temperature_iters = 0;
    // 連続でこの回数だけはみ出た部分があった場合、温度をリセットする。
    const int num_consecutive_protrusion_thredshold = 2000;
    // 連続ではみ出た部分があった回数。
    int num_consecutive_protrusion = 0;
    const double T0 = 1.0e1;
    const double T1 = 1.0e-2;
    double progress = 0.0;
    // イテレーション回数
    int iter = 0;

    auto evaluate_and_descide_rollback = [&]() -> bool {
      auto [feasible, updated_cost, has_protrusion] = Evaluate(pose);

      if (has_protrusion) {
          if (++num_consecutive_protrusion >= num_consecutive_protrusion_thredshold) {
              num_consecutive_protrusion = 0;
              last_reset_temperature_iters = iter;
          }
      }
      else {
          num_consecutive_protrusion = 0;
      }

#if 0
      auto judge_valid = judge(*args.problem, pose).is_valid();
      if (feasible != judge_valid) {
        LOG(INFO) << feasible << " " << judge_valid;
        if (editor) {
          editor->set_pose(std::make_shared<SSolution>(pose));
          while (true) {
            int c = editor->show(1);
            if (c == 27) break;
          }
        }
      }
#endif

      if (feasible && updated_cost < best_feasible_cost) {
        best_feasible_cost = updated_cost;
        best_feasible_pose = pose;
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
      const int v = std::uniform_int_distribution(0, N - 1)(rng_);
      const int dx = std::uniform_int_distribution(-1, 1)(rng_);
      const int dy = std::uniform_int_distribution(-1, 1)(rng_);
      auto& [x, y] = pose[v];
      x += dx;
      y += dy;
      if (evaluate_and_descide_rollback()) {
        x -= dx;
        y -= dy;
      }
    };

    auto flip = [&] { // from FlipAnnealingSolver
      const int v0 = std::uniform_int_distribution(0, N - 1)(rng_);
      const int v1 = std::uniform_int_distribution(0, N - 1)(rng_);
      if (v0 == v1) return;
      auto pose_bak = pose;
      auto reflect = [](Point a, Point c, Point v) {
        return v - 2 * double(dot(v - c, a)) / double(dot(a, a)) * a;
      };
      auto diff = pose[v1] - pose[v0];
      Point n = {get_y(diff), -get_x(diff)};
      for (int v2 = 0; v2 < pose.size(); ++v2) {
        if (ccw(pose[v0], pose[v1], pose[v2])) {
          pose[v2] = reflect(n, pose[v0], pose[v2]);
        }
      }
      if (evaluate_and_descide_rollback()) {
        pose = pose_bak;
      }
    };

    integer ymin = INT_MAX, ymax = INT_MIN;
    integer xmin = INT_MAX, xmax = INT_MIN;
    for (auto p : hole_) {
      xmin = std::min(xmin, get_x(p));
      ymin = std::min(ymin, get_y(p));
      xmax = std::max(xmax, get_x(p));
      ymax = std::max(ymax, get_y(p));
    }

    auto edges_cache = edges_from_vertex(*args.problem);
    auto hop_grid = [&] { // jump to a tolerated (by at least one edge) point.
      const int pivot = std::uniform_int_distribution(0, N - 1)(rng_);
      const auto pivot_bak = pose[pivot];
      const auto& edges = edges_cache[pivot];
      std::map<Point, double> good_pos; // position -> number of good grid vote.
      for (auto eid : edges) {
        auto [u, v] = args.problem->edges[eid];
        const int counter_vid = u == pivot ? v : u;
        const auto org_d2 = distance2(args.problem->vertices[pivot], args.problem->vertices[counter_vid]);
        for (int y = ymin; y <= ymax; ++y) {
          for (int x = xmin; x <= xmax; ++x) {
            const auto moved_d2 = distance2({ x, y }, pose[counter_vid]);
            if (tolerate(org_d2, moved_d2, epsilon_)) {
              Point jump {x, y};
              auto it = good_pos.find(jump);
              if (it == good_pos.end()) {
                good_pos.insert(it, {jump, 1.0});
              } else {
                it->second += 1.0;
              }
            }
          }
        }
      }
      // emphasize large votes.
      double total_votes = 0;
      for (auto& [pos, vote] : good_pos) {
        vote = std::pow(vote, 5);
        total_votes += vote;
      }
      const double select_accum_vote = std::uniform_real_distribution<double>(0.0, total_votes)(rng_);
      double accum_vote = 0;
      for (auto& [pos, vote] : good_pos) {
        accum_vote += vote;
        if (select_accum_vote <= accum_vote) {
          pose[pivot] = pos;
          break;
        }
      }
      if (evaluate_and_descide_rollback()) {
        pose[pivot] = pivot_bak;
      }
    };

    using Action = std::function<void()>;
    std::vector<std::pair<double, Action>> action_probs = {
      {0.9, single_small_change},
      {0.01, hop_grid},
      {0.01, flip},
    };
    {
      // normalize probs
      double p = 0.0;
      for (int i = 0; i < action_probs.size(); ++i) { p += action_probs[i].first; }
      for (int i = 0; i < action_probs.size(); ++i) { action_probs[i].first /= p; }
    }

    for (; iter < num_iters; ++iter) {
      progress = 1.0 * (iter - last_reset_temperature_iters) / (num_iters - last_reset_temperature_iters);

      const double p_action = std::uniform_real_distribution(0.0, 1.0)(rng_);
      double p_accum = 0.0;
      for (int i = 0; i < action_probs.size(); ++i) { 
        p_accum += action_probs[i].first;
        if (p_action < p_accum) {
          action_probs[i].second();
        }
      }

      if (editor && iter % 100 == 0) {
        editor->set_pose(std::make_shared<SSolution>(pose));
        int c = editor->show(1);
      }
    }

    SolverOutputs outputs;
    if (best_feasible_pose.empty()) {
      outputs.solution = std::make_shared<SSolution>(pose);
    } else {
      outputs.solution = std::make_shared<SSolution>(best_feasible_pose);
    }
    return outputs;
  }

  template <typename P>
  EvaluationResult Evaluate(const std::vector<P>& pose) const {
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
        protrusion_cost += 1.0e-2 * bg::length(segment);
      }

      const auto d0 = SquaredEdgeLength(vertices_, edge);
      const auto d1 = SquaredEdgeLength(pose, edge);
      deformation_cost += 1.0e1 * std::max(0.0, std::abs(d1 / d0 - 1.0) - tolerance);
    }

    for (const auto h : hole_) {
      double best = std::numeric_limits<double>::infinity();
      for (const auto v : pose) {
        best = std::min(best, SquaredDistance(h, v));
      }
      dislikes_cost += best * 1.0e-2;
    }

    const bool feasible = deformation_cost + protrusion_cost == 0.0;
    const bool has_protrusion = protrusion_cost != 0.0;
    const double cost = deformation_cost + protrusion_cost + dislikes_cost;

    return {feasible, cost, has_protrusion};
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

REGISTER_SOLVER("ResetTempertureAnnealingSolver", ResetTempertureAnnealingSolver::Solver);
// vim:ts=2 sw=2 sts=2 et ci
