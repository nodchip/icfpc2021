#include "stdafx.h"
#include <cmath>
#include <iostream>
#include <fmt/format.h>
#include <boost/geometry.hpp>
#include <boost/geometry/geometries/point_xy.hpp>
#include <boost/geometry/geometries/polygon.hpp>
#include "contest_types.h"
#include "solver_registry.h"
#include "visual_editor.h"
#include "solver_util.h"
#include "judge.h"

namespace HopGridAnnealingSolver2 {

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
      editor = std::make_shared<SVisualEditor>(args.problem, "HopGridAnnealingSolver2", "visualize");
    }

    bool show(true);
    const int N = vertices_.size();
    auto pose = vertices_;
    double cost = std::numeric_limits<double>::infinity();
    double best_feasible_cost = std::numeric_limits<double>::infinity();
    std::vector<Point> best_feasible_pose;

    SPinnedIndex pinned_index(rng_, N, editor);

    const int num_iters = 1000000;
    const double T0 = 1.0e1;
    const double T1 = 1.0e-2;
    double progress = 0.0;

    integer ymin = INT_MAX, ymax = INT_MIN;
    integer xmin = INT_MAX, xmax = INT_MIN;
    for (auto p : hole_) {
      xmin = std::min(xmin, get_x(p));
      ymin = std::min(ymin, get_y(p));
      xmax = std::max(xmax, get_x(p));
      ymax = std::max(ymax, get_y(p));
    }

    std::vector<std::vector<int>> emat(vertices_.size());
    for (const auto &e : edges_) {
      emat[e.first].emplace_back(e.second);
      emat[e.second].emplace_back(e.first);
    }
    neighbors_.resize(vertices_.size());
    for (int i = 0; i < vertices_.size(); ++i) {
      std::set<int> neighbors;
      for (const auto &v : emat[i]) {
        if (v != i) {
          neighbors.emplace(v);
        }
        for (const auto &v2 : emat[v]) {
          if (v2 != i) {
            neighbors.emplace(v2);
          }
          for (const auto &v3 : emat[v2]) {
            if (v3 != i) {
              neighbors.emplace(v3);
            }
          }
        }
      }
      for (auto v : neighbors) {
        neighbors_[i].emplace_back(v);
      }
    }

    // lesser version of tonagi's idea
//    for (auto& p : pose) { p = hole_[0]; }

    auto evaluate_and_descide_rollback = [&]() -> bool {
      auto [feasible, updated_cost, real_cost] = Evaluate(pose, show);

      // tonagi's idea.
      if (false || !best_feasible_pose.empty()) {
        auto res = judge(*args.problem, pose);
        if (!res.fit_in_hole()) {
          feasible = false;
//          updated_cost = DBL_MAX;
        }
      }

#if 0
      auto judge_valid = judge(*args.problem, pose).is_valid();
      if (feasible != judge_valid) {
        LOG(INFO) << feasible << " " << judge_valid;
        if (editor) {
          editor->set_pose(args.problem->create_solution(pose));
          while (true) {
            int c = editor->show(1);
            if (c == 27) break;
          }
        }
      }
#endif

      if (feasible && real_cost < best_feasible_cost) {
        best_feasible_cost = real_cost;
        best_feasible_pose = pose;
        if (editor) editor->set_persistent_custom_stat(fmt::format("best_cost = {}", best_feasible_cost));
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
      const int dx = std::normal_distribution(0.0, 10.0)(rng_);
      const int dy = std::normal_distribution(0.0, 10.0)(rng_);
      auto& [x, y] = pose[v];
      x += dx;
      y += dy;
      if (evaluate_and_descide_rollback()) {
        x -= dx;
        y -= dy;
      }
    };

    auto shift = [&] {
      const int dx = std::uniform_int_distribution(-1, 1)(rng_);
      const int dy = std::uniform_int_distribution(-1, 1)(rng_);
      auto pose_bak = pose;
      for (int i : pinned_index.movable_indices) {
        pose[i].first += dx;
        pose[i].second += dy;
      }
      if (evaluate_and_descide_rollback()) {
        pose = pose_bak;
      }
    };

    auto slight_rotate = [&] {
      const double deg = std::uniform_real_distribution(-180.0, 180.0)(rng_);
      auto pose_bak = pose;

      integer curr_ymin = INT_MAX, curr_ymax = INT_MIN;
      integer curr_xmin = INT_MAX, curr_xmax = INT_MIN;
      for (auto p : hole_) {
        curr_xmin = std::min(curr_xmin, get_x(p));
        curr_ymin = std::min(curr_ymin, get_y(p));
        curr_xmax = std::max(curr_xmax, get_x(p));
        curr_ymax = std::max(curr_ymax, get_y(p));
      }

      const double cx = double(curr_xmin + curr_xmax) / 2;
      const double cy = double(curr_ymin + curr_ymax) / 2;
      const double sin = std::sin(deg * 3.1415 / 180.0);
      const double cos = std::cos(deg * 3.1415 / 180.0);
      for (int i : pinned_index.movable_indices) {
        auto& p = pose[i];
        const double dx = p.first - cx;
        const double dy = p.second - cy;
        p.first  = std::round(dx * cos - dy * sin + cx);
        p.second = std::round(dx * sin + dy * cos + cy);
      }
      if (evaluate_and_descide_rollback()) {
        pose = pose_bak;
      }
    };

    auto flip = [&] { // from FlipAnnealingSolver
      const int v0 = pinned_index.sample_movable_index();
      const int v1 = pinned_index.sample_movable_index();
      if (v0 == v1) return;
      auto pose_bak = pose;
      auto reflect = [](Point a, Point c, Point v) {
        return v - 2 * double(dot(v - c, a)) / double(dot(a, a)) * a;
      };
      auto diff = pose[v1] - pose[v0];
      Point n = {get_y(diff), -get_x(diff)};
      for (int v2 : pinned_index.movable_indices) {
        if (ccw(pose[v0], pose[v1], pose[v2])) {
          pose[v2] = reflect(n, pose[v0], pose[v2]);
        }
      }
      if (evaluate_and_descide_rollback()) {
        pose = pose_bak;
      }
    };

    auto edges_cache = edges_from_vertex(*args.problem);
    auto hop_grid = [&] { // jump to a tolerated (by at least one edge) point.
      const int pivot = pinned_index.sample_movable_index();
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
      {0.98, single_small_change},
      {0.005, slight_rotate},
      {0.005, shift},
      {0.005, hop_grid},
      {0.005, flip},
    };
    {
      // normalize probs
      double p = 0.0;
      for (int i = 0; i < action_probs.size(); ++i) { p += action_probs[i].first; }
      for (int i = 0; i < action_probs.size(); ++i) { action_probs[i].first /= p; }
    }

    for (int iter = 0; iter < num_iters; ++iter) {
      progress = 1.0 * iter / num_iters;

      const double p_action = std::uniform_real_distribution(0.0, 1.0)(rng_);
      double p_accum = 0.0;
      for (int i = 0; i < action_probs.size(); ++i) { 
        p_accum += action_probs[i].first;
        if (p_action < p_accum) {
          action_probs[i].second();
        }
      }

      if (iter % 100 == 0) {
        show = true;
        std::cerr << cost << ", " << best_feasible_cost << std::endl;
      } else {
        show = false;
      }
      if (editor && iter % 100 == 0) {
        editor->set_oneshot_custom_stat(fmt::format("iter = {}/{}", iter, num_iters));
        editor->set_pose(args.problem->create_solution(pose));
        if (auto show_result = editor->show(1); show_result.edit_result) {
          pose = show_result.edit_result->pose_after_edit->vertices;
          auto [feasiblecost, updated_cost, real_cost] = Evaluate(pose, true);
          cost = updated_cost;
          pinned_index.update_movable_index();
        }
      }
    }

    SolverOutputs outputs;
    if (best_feasible_pose.empty()) {
      outputs.solution = args.problem->create_solution(pose);
    } else {
      outputs.solution = args.problem->create_solution(best_feasible_pose);
    }
    return outputs;
  }

  template <typename P>
  std::tuple<bool, double, double> Evaluate(const std::vector<P>& pose, bool show) const {
    double deformation_cost = 0.0;
    double protrusion_cost = 0.0;
    double dislikes_cost = 0.0;
    double neighbor_cost = 0.0;

    const double tolerance = epsilon_ / 1'000'000.0;
    int vidx(0);
    for (const auto& vertex : pose) {
      protrusion_cost += bg::distance(ToBoostPoint(vertex), hole_polygon_);
      for (const auto& v : neighbors_[vidx]) {
        neighbor_cost += 1.0 / (SquaredDistance(vertex, pose[v]) + 1e-6);
      }
/*
      for (int i = 0; i < vertices_.size(); ++i) {
        if (i != vidx) {
          neighbor_cost += 1.0 / (SquaredDistance(vertex, pose[i]) + 1e-6);
        }
      }
*/
      ++vidx;
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
        protrusion_cost += bg::length(segment);
      }

      const auto d0 = SquaredEdgeLength(vertices_, edge);
      const auto d1 = SquaredEdgeLength(pose, edge);
      deformation_cost += 1.0e1 * std::max(0.0, std::abs(d1 / d0 - 1.0) - tolerance);
    }

    double max_best(0.0);
    for (const auto h : hole_) {
      double best = std::numeric_limits<double>::infinity();
      for (const auto v : pose) {
        best = std::min(best, SquaredDistance(h, v));
      }
      max_best = std::max(max_best, best);
      dislikes_cost += best * 1.0e-2;
    }
    dislikes_cost += max_best;

    deformation_cost *= 1e2;
    protrusion_cost *= 1e3;
    dislikes_cost *= 1e-1;

    neighbor_cost *= 1e1;

    if (show) {
      std::cerr << max_best << ", " << deformation_cost << ", " << protrusion_cost << ", " << dislikes_cost << ", " << neighbor_cost << std::endl;
    }
    const bool feasible = deformation_cost + protrusion_cost == 0.0;
    const double cost = deformation_cost + protrusion_cost + dislikes_cost + neighbor_cost;

    return {feasible, cost, max_best};
  }

 private:
  std::mt19937 rng_;
  std::vector<Point> hole_;
  std::vector<Point> vertices_;
  std::vector<Edge> edges_;
  std::vector<std::vector<int>> neighbors_;
  integer epsilon_;
  BoostPolygon hole_polygon_;
};

}

REGISTER_SOLVER("HopGridAnnealingSolver2", HopGridAnnealingSolver2::Solver);
// vim:ts=2 sw=2 sts=2 et ci
