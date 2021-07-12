#include "stdafx.h"

#include <cmath>

#include <boost/geometry.hpp>
#include <boost/geometry/geometries/point_xy.hpp>
#include <boost/geometry/geometries/polygon.hpp>
#include <fmt/format.h>
#include <nlohmann/json.hpp>

#include "contest_types.h"
#include "judge.h"
#include "solver_registry.h"
#include "visual_editor.h"

namespace OptunaAnnealingSolver {

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

      // パラメーターファイルを読み込む
      std::ifstream ifs;
      // 実行バイナリと同じフォルダから読み込む
      ifs.open(args.parameters_file_path);
      CHECK(ifs);
      ifs >> parameters_;

      SVisualEditorPtr editor;
      if (args.visualize) {
        editor = std::make_shared<SVisualEditor>(args.problem, "OptunaAnnealingSolver", "visualize");
      }

      const int N = vertices_.size();
      auto pose = vertices_;
      double cost = std::numeric_limits<double>::infinity();
      double best_feasible_cost = std::numeric_limits<double>::infinity();
      std::vector<Point> best_feasible_pose;

      const int num_iters = 100000;
      const double T0 = parameters_["T0"]; // 10.0 1.0-100.0 log
      const double T1 = parameters_["T1"]; // 0.01 0.001-0.1 log
      double progress = 0.0;

      integer ymin = INT_MAX, ymax = INT_MIN;
      integer xmin = INT_MAX, xmax = INT_MIN;
      for (auto p : hole_) {
        xmin = std::min(xmin, get_x(p));
        ymin = std::min(ymin, get_y(p));
        xmax = std::max(xmax, get_x(p));
        ymax = std::max(ymax, get_y(p));
      }

      // lesser version of tonagi's idea 
      const int initialize_pose_by_hole = parameters_["initialize_pose_by_hole"]; // 0 0-1
      if (initialize_pose_by_hole) {
        for (auto& p : pose) { p = hole_[0]; }
      }

      const int prohibit_unfeasible_after_feasible = parameters_["prohibit_unfeasible_after_feasible"]; // 0 0-1
      auto evaluate_and_descide_rollback = [&]() -> bool {
        auto [feasible, updated_cost] = Evaluate(pose);

        // tonagi's idea.
        if (prohibit_unfeasible_after_feasible || !best_feasible_pose.empty()) {
          auto res = judge(*args.problem, pose);
          if (!res.fit_in_hole()) {
            feasible = false;
            updated_cost = DBL_MAX;
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

        if (feasible && updated_cost < best_feasible_cost) {
          best_feasible_cost = updated_cost;
          best_feasible_pose = pose;
          if (editor) editor->set_persistent_custom_stat(fmt::format("best_cost = {}", best_feasible_cost));
        }
        const double T = std::pow(T0, 1.0 - progress) * std::pow(T1, progress);
        if (std::uniform_real_distribution(0.0, 1.0)(rng_) < std::exp(-(updated_cost - cost) / T)) {
          cost = updated_cost;
          return false; // accepted
        }
        else {
          return true; // rejected
        }
      };
      evaluate_and_descide_rollback();

      const int single_small_change_max_delta = parameters_["single_small_change_max_delta"]; // 1 1-5
      auto single_small_change = [&] { // ynasu87 original
        const int v = std::uniform_int_distribution(0, N - 1)(rng_);
        const int dx = std::uniform_int_distribution(-single_small_change_max_delta, single_small_change_max_delta)(rng_);
        const int dy = std::uniform_int_distribution(-single_small_change_max_delta, single_small_change_max_delta)(rng_);
        auto& [x, y] = pose[v];
        x += dx;
        y += dy;
        if (evaluate_and_descide_rollback()) {
          x -= dx;
          y -= dy;
        }
      };

      const int shift_max_delta = parameters_["shift_max_delta"]; // 1 1-5
      auto shift = [&] {
        const int dx = std::uniform_int_distribution(-shift_max_delta, shift_max_delta)(rng_);
        const int dy = std::uniform_int_distribution(-shift_max_delta, shift_max_delta)(rng_);
        auto pose_bak = pose;
        for (auto& p : pose) {
          p.first += dx;
          p.second += dy;
        }
        if (evaluate_and_descide_rollback()) {
          pose = pose_bak;
        }
      };

      const double slight_rotate_max_deg = parameters_["slight_rotate_max_deg"]; // 2.0 1.0-180.0
      auto slight_rotate = [&] {
        const double deg = std::uniform_real_distribution(-slight_rotate_max_deg, slight_rotate_max_deg)(rng_);
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
        for (auto& p : pose) {
          const double dx = p.first - cx;
          const double dy = p.second - cy;
          p.first = std::round(dx * cos - dy * sin + cx);
          p.second = std::round(dx * sin + dy * cos + cy);
        }
        if (evaluate_and_descide_rollback()) {
          pose = pose_bak;
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
        Point n = { get_y(diff), -get_x(diff) };
        for (int v2 = 0; v2 < pose.size(); ++v2) {
          if (ccw(pose[v0], pose[v1], pose[v2])) {
            pose[v2] = reflect(n, pose[v0], pose[v2]);
          }
        }
        if (evaluate_and_descide_rollback()) {
          pose = pose_bak;
        }
      };

      const double vote_pow = parameters_["vote_pow"]; // 5.0 1.0-5.0
      auto edges_cache = edges_from_vertex(*args.problem);
      std::vector<std::vector<double> > good_pos(ymax - ymin + 1, std::vector<double>(xmax - xmin + 1));
      auto hop_grid = [&] { // jump to a tolerated (by at least one edge) point.
        const int pivot = std::uniform_int_distribution(0, N - 1)(rng_);
        const auto pivot_bak = pose[pivot];
        const auto& edges = edges_cache[pivot];
        for (auto& row : good_pos) {
          fill(row.begin(), row.end(), 0);
        }
        for (auto eid : edges) {
          auto [u, v] = args.problem->edges[eid];
          const int counter_vid = u == pivot ? v : u;
          const auto org_d2 = distance2(args.problem->vertices[pivot], args.problem->vertices[counter_vid]);
          for (int y = ymin; y <= ymax; ++y) {
            for (int x = xmin; x <= xmax; ++x) {
              const auto moved_d2 = distance2({ x, y }, pose[counter_vid]);
              if (tolerate(org_d2, moved_d2, epsilon_)) {
                ++good_pos[y - ymin][x - xmin];
              }
            }
          }
        }
        // emphasize large votes.
        double total_votes = 0;
        for (int y = ymin; y <= ymax; ++y) {
          for (int x = xmin; x <= xmax; ++x) {
            if (good_pos[y - ymin][x - xmin] == 0) {
              continue;
            }
            good_pos[y - ymin][x - xmin] = std::pow(good_pos[y - ymin][x - xmin], vote_pow);
            total_votes += good_pos[y - ymin][x - xmin];
          }
        }
        const double select_accum_vote = std::uniform_real_distribution<double>(0.0, total_votes)(rng_);
        double accum_vote = 0;
        bool found = false;
        for (int y = ymin; !found && y <= ymax; ++y) {
          for (int x = xmin; !found && x <= xmax; ++x) {
            if (good_pos[y - ymin][x - xmin] == 0) {
              continue;
            }
            double adjusted_vote = good_pos[y - ymin][x - xmin];
            accum_vote += adjusted_vote;
            if (select_accum_vote <= accum_vote) {
              pose[pivot] = {x, y};
              found = true;
            }
          }
        }
        if (evaluate_and_descide_rollback()) {
          pose[pivot] = pivot_bak;
        }
      };

      // はみ出している線分を移動させる
      const int slide_protrusion_max_delta = parameters_["slide_protrusion_max_delta"]; // 1 1-5
      auto slide_protrusion = [&] { // from OptunaAnnealingSolver
        bool found = false;
        int vertex_index_backup[2];
        Point vertex_backup[2];
        for (const auto& edge : edges_) {
          const auto [a, b] = edge;
          Point2d pa = pose[a];
          Point2d pb = pose[b];
          shrink(pa, pb);
          const auto boost_point_a = ToBoostPoint(pa);
          const auto boost_point_b = ToBoostPoint(pb);
          BoostLinestring linestring{ boost_point_a, boost_point_b };
          std::vector<BoostLinestring> differences;
          bg::difference(linestring, hole_polygon_, differences);
          if (differences.empty()) {
            continue;
          }

          found = true;
          vertex_index_backup[0] = a;
          vertex_index_backup[1] = b;
          vertex_backup[0] = pose[a];
          vertex_backup[1] = pose[b];

          pose[a].first += std::uniform_int_distribution(-slide_protrusion_max_delta, slide_protrusion_max_delta)(rng_);
          pose[a].second += std::uniform_int_distribution(-slide_protrusion_max_delta, slide_protrusion_max_delta)(rng_);
          pose[b].first += std::uniform_int_distribution(-slide_protrusion_max_delta, slide_protrusion_max_delta)(rng_);
          pose[b].second += std::uniform_int_distribution(-slide_protrusion_max_delta, slide_protrusion_max_delta)(rng_);

          break;
        }

        if (found) {
          if (evaluate_and_descide_rollback()) {
            pose[vertex_index_backup[0]] = vertex_backup[0];
            pose[vertex_index_backup[1]] = vertex_backup[1];
          }
        }
      };

      using Action = std::function<void()>;
      const double single_small_change_probability = parameters_["single_small_change_probability"]; // 0.9 0.0-1.0
      const double slight_rotate_probability = parameters_["slight_rotate_probability"]; // 0.01 0.0-1.0
      const double shift_probability = parameters_["shift_probability"]; // 0.01 0.0-1.0
      const double hop_grid_probability = parameters_["hop_grid_probability"]; // 0.01 0.0-1.0
      const double flip_probability = parameters_["flip_probability"]; // 0.03 0.0-1.0
      const double slide_protrusion_probability = parameters_["slide_protrusion_probability"]; // 0.10 0.0-1.0
      std::vector<std::pair<double, Action>> action_probs = {
        {single_small_change_probability, single_small_change},
        {slight_rotate_probability, slight_rotate},
        {shift_probability, shift},
        {hop_grid_probability, hop_grid},
        {flip_probability, flip},
        {slide_protrusion_probability, slide_protrusion},
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

        if (editor && iter % 100 == 0) {
          editor->set_oneshot_custom_stat(fmt::format("iter = {}/{}", iter, num_iters));
          editor->set_pose(args.problem->create_solution(pose));
          if (auto show_result = editor->show(1); show_result.edit_result) {
            pose = show_result.edit_result->pose_after_edit->vertices;
          }
        }
      }

      SolverOutputs outputs;
      if (best_feasible_pose.empty()) {
        outputs.solution = args.problem->create_solution(pose);
      }
      else {
        outputs.solution = args.problem->create_solution(best_feasible_pose);
      }
      return outputs;
    }

    template <typename P>
    std::tuple<bool, double> Evaluate(const std::vector<P>& pose) const {
      const double protrusion_cost_coefficient0 = parameters_["protrusion_cost_coefficient0"]; // 1.0 0.01-100.0 log
      const double protrusion_cost_coefficient1 = parameters_["protrusion_cost_coefficient1"]; // 0.01 0.01-100.0 log
      const double deformation_cost_coefficient = parameters_["deformation_cost_coefficient"]; // 10.0 0.01-100.0 log
      const double dislikes_cost_coefficient = parameters_["dislikes_cost_coefficient"]; // 0.01 0.01-100.0 log

      double deformation_cost = 0.0;
      double protrusion_cost = 0.0;
      double dislikes_cost = 0.0;

      const double tolerance = epsilon_ / 1'000'000.0;
      for (const auto& vertex : pose) {
        protrusion_cost += protrusion_cost_coefficient0 * bg::distance(ToBoostPoint(vertex), hole_polygon_);
      }
      for (const auto& edge : edges_) {
        const auto [a, b] = edge;
        Point2d pa = pose[a];
        Point2d pb = pose[b];
        shrink(pa, pb);
        BoostLinestring linestring{ ToBoostPoint(pa), ToBoostPoint(pb) };
        std::vector<BoostLinestring> differences;
        bg::difference(linestring, hole_polygon_, differences);
        for (const auto& segment : differences) {
          protrusion_cost += protrusion_cost_coefficient1 * bg::length(segment);
        }

        const auto d0 = SquaredEdgeLength(vertices_, edge);
        const auto d1 = SquaredEdgeLength(pose, edge);
        deformation_cost += deformation_cost_coefficient * std::max(0.0, std::abs(d1 / d0 - 1.0) - tolerance);
      }

      for (const auto h : hole_) {
        double best = std::numeric_limits<double>::infinity();
        for (const auto v : pose) {
          best = std::min(best, SquaredDistance(h, v));
        }
        dislikes_cost += best * dislikes_cost_coefficient;
      }

      const bool feasible = deformation_cost + protrusion_cost == 0.0;
      const double cost = deformation_cost + protrusion_cost + dislikes_cost;

      return { feasible, cost };
    }

  private:
    std::mt19937 rng_;
    std::vector<Point> hole_;
    std::vector<Point> vertices_;
    std::vector<Edge> edges_;
    integer epsilon_;
    BoostPolygon hole_polygon_;
    nlohmann::json parameters_;
  };

}

REGISTER_SOLVER("OptunaAnnealingSolver", OptunaAnnealingSolver::Solver);
// vim:ts=2 sw=2 sts=2 et ci
