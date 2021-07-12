#include "stdafx.h"
#include <cmath>
#include <vector>
#include <stack>
#include <unordered_map>
#include <unordered_set>
#include <iostream>
#include <fmt/format.h>
#include <boost/geometry.hpp>
#include <boost/geometry/geometries/point_xy.hpp>
#include <boost/geometry/geometries/polygon.hpp>
#include "contest_types.h"
#include "solver_registry.h"
#include "solver_util.h"
#include "visual_editor.h"
#include "timer.h"
#include "judge.h"

namespace NNaiveSearchSolver {
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

std::vector<Point> enumerate_interior_points(const SProblem& problem) {
  integer ymin = INT_MAX, ymax = INT_MIN;
  integer xmin = INT_MAX, xmax = INT_MIN;
  for (auto p : problem.hole_polygon) {
    xmin = std::min(xmin, get_x(p));
    ymin = std::min(ymin, get_y(p));
    xmax = std::max(xmax, get_x(p));
    ymax = std::max(ymax, get_y(p));
  }

#if 0
  auto bg_hole_polygon = ToBoostPolygon(problem.hole_polygon);
  std::vector<Point> points;
  for (int y = ymin; y <= ymax; ++y) {
    for (int x = xmin; x <= xmax; ++x) {
      if (bg::within(ToBoostPoint(Point{x, y}), bg_hole_polygon)) { // exclude points on the edge/vertex of the hole.
        points.emplace_back(x, y);
      }
    }
  }
#else
  std::vector<Point> points;
  for (int y = ymin; y <= ymax; ++y) {
    for (int x = xmin; x <= xmax; ++x) {
      if (contains(problem.hole_polygon, {x, y}) != EContains::EOUT) { // include points on the edge/vertex of the hole.
        points.emplace_back(x, y);
      }
    }
  }
#endif
  LOG(INFO) << fmt::format("found {} interior points", points.size());
  return points;
}

class NaiveSearchSolver : public SolverBase {
private:
  std::mt19937 rng;

public:
  NaiveSearchSolver() { }
  SolverOutputs solve(const SolverArguments &args) override {
    SolverOutputs ret;

    // init.
    ret.solution = args.optional_initial_solution ? args.optional_initial_solution : args.problem->create_solution();

    // develop.
    constexpr int check_timeout_every_iter = 100000;
    constexpr int report_every_iter = 100000;
    constexpr int editor_sleep = 1;
    constexpr bool exhaustive_search = true;
    constexpr bool use_cache_for_good_edge = false;
    constexpr bool use_cache_for_movable_edges_with_tolerance = true;
    constexpr bool use_cache_for_infeasible_placement_set = true;
    constexpr size_t infeasible_placement_cache_size_B = 4 * 1024ull * 1024ull * 1024ull;
    const std::optional<int> subsample_roots = std::nullopt;
    SVisualEditorPtr editor;
    if (args.visualize) {
      editor = std::make_shared<SVisualEditor>(args.problem, "NaiveSearchSolver", "visualize");
    }

    // prepare.
    auto interior_points = enumerate_interior_points(*args.problem);
    std::shuffle(interior_points.begin(), interior_points.end(), rng);
    const std::vector<std::vector<int>> edges_from_vertex_cache = edges_from_vertex(*args.problem);
    const int V = args.problem->vertices.size();

    // good edge
    auto bg_hole_polygon = ToBoostPolygon(args.problem->hole_polygon);
    auto _is_good_edge = [&](const Point& p, const Point& q) {
      BoostLinestring linestring{ToBoostPoint(p), ToBoostPoint(q)};
      std::vector<BoostLinestring> differences;
      bg::difference(linestring, bg_hole_polygon, differences);
      return differences.empty();
    };
    struct Line_hash {
      std::size_t operator()(const Line& line) const {
        return std::hash<int>()(line[0].first)
             ^ std::hash<int>()(line[0].second)
             ^ std::hash<int>()(line[1].first)
             ^ std::hash<int>()(line[1].second)
          ;
      }
    };
    std::unordered_set<Line, Line_hash> is_bad_edge_cache; // assume |bad edges| < |good edges|
    if (use_cache_for_good_edge) {
      LOG(INFO) << "bad_edge_cache ON: building";
      Timer t("build is_bad_edge_cache");
      const size_t N = interior_points.size();
      size_t c = 0;
      for (size_t i = 0; i < N; ++i) {
        const auto& p = interior_points[i];
        for (size_t j = i; j < N; ++j) {
          const auto& q = interior_points[j];
          if (!_is_good_edge(p, q)) {
            is_bad_edge_cache.insert({p, q});
            is_bad_edge_cache.insert({q, p});
          }
          ++c;
          if (c % 100000 == 0) LOG(INFO) << fmt::format("{}/{} ({:.2f}%) bad_edges={}", c, N * (N + 1) / 2, 100.0 * c / double(N * (N + 1) / 2), is_bad_edge_cache.size());
        }
      }
      LOG(INFO) << fmt::format("is_bad_edge_cache #{}", is_bad_edge_cache.size());
    } else {
      LOG(INFO) << "bad_edge_cache OFF";
    }
    auto is_good_edge = [&](const Point& p, const Point& q) -> bool {
      if (!use_cache_for_good_edge)
        return _is_good_edge(p, q);

      auto it = is_bad_edge_cache.find({p, q});
      return it == is_bad_edge_cache.end();
    };

    // (starting position, radius**2) -> (available points)
    using SKey = std::pair<Point, int>;
    struct SKey_hash {
      std::size_t operator()(const SKey& key) const {
        return std::hash<int>()(key.first.first)
             ^ std::hash<int>()(key.first.second)
             ^ std::hash<int>()(key.second)
          ;
      }
    };
    int64_t movable_cache_count = 0;
    std::unordered_map<SKey, std::vector<Point>, SKey_hash> movable_cache;
    // exact lookup.
    auto get_movable_points = [&](Point p, int d2) -> std::vector<Point> {
      const SKey query {p, d2};
      auto it = movable_cache.find(query);
      if (it != movable_cache.end()) {
        return it->second;
      }
      std::vector<Point> points;
      for (Point ip : interior_points) {
        if (d2 == distance2(p, ip) && is_good_edge(p, ip)) {
          ++movable_cache_count;
          points.push_back(ip);
        }
      }
      movable_cache.insert(it, {query, points});
      return points;
    };
    // consider tolerance.
    LOG(INFO) << (use_cache_for_movable_edges_with_tolerance ? "movable_points_with_tolerance_cache ON" : "movable_points_with_tolerance_cache OFF");
    std::unordered_map<SKey, std::vector<Point>, SKey_hash> movable_points_with_tolerance_cache;
    auto _get_movable_points_with_tolerance = [&](Point p, int original_distance2) -> std::vector<Point> {
      // |d_new/d_old - 1| <= eps / 1000000
      constexpr int k = 1'000'000;
      const int e = args.problem->epsilon;
      const int distance2_min = std::floor(original_distance2 * double(k - e) / double(k));
      const int distance2_max = std::ceil(original_distance2 * double(k + e) / double(k));
      std::vector<Point> all_points;
      for (int d2 = distance2_min; d2 <= distance2_max; ++d2) {
        if (std::abs(d2 - original_distance2) * k <= original_distance2 * e) { // to make sure.
          auto points = get_movable_points(p, d2);
          std::copy(points.begin(), points.end(), std::back_inserter(all_points));
        }
      }
      return all_points;
    };
    auto get_movable_points_with_tolerance = [&](Point p, int original_distance2) -> std::vector<Point> {
      if (!use_cache_for_movable_edges_with_tolerance)
        return _get_movable_points_with_tolerance(p, original_distance2);

      const SKey query {p, original_distance2};
      auto it = movable_points_with_tolerance_cache.find(query);
      if (it != movable_points_with_tolerance_cache.end()) {
        return it->second;
      }
      auto all_points = _get_movable_points_with_tolerance(p, original_distance2);
      movable_points_with_tolerance_cache.insert(it, {query, all_points});
      return all_points;
    };

    constexpr int USED = 1;
    constexpr int REMAINING = 0;
    struct State;
    using StatePtr = std::shared_ptr<State>;
    struct State {
      // placing vertices[vid] to position p.
      int depth = 0;
      int vid = 0;
      Point p = {0, 0};
      // remaining indices after placing this node.
      std::vector<int> indices_flag;
      // vertices after placing this node.
      std::vector<Point> vertices;

      State(int depth, int vid, Point p, const std::vector<int>& indices_flag, const std::vector<Point>& vertices)
        : depth(depth), vid(vid), p(p), indices_flag(indices_flag), vertices(vertices) {}
    };
    auto create_fixed_indices = [&](StatePtr s) {
      std::vector<int> fixed_indices;
      for (int i = 0; i < V; ++i) {
        if (s->indices_flag[i] == USED) {
          fixed_indices.push_back(i);
        }
      }
      return fixed_indices;
    };

    // infeasible placement cache
    int min_x = std::numeric_limits<int>::max();
    int min_y = std::numeric_limits<int>::max();
    int max_x = std::numeric_limits<int>::min();
    int max_y = std::numeric_limits<int>::min();
    for (auto& p : interior_points) {
      chmin<int>(min_x, p.first);
      chmax<int>(max_x, p.first);
      chmin<int>(min_y, p.second);
      chmax<int>(max_y, p.second);
    }
    const int size_x = max_x - min_x + 1;
    const int size_y = max_y - min_y + 1;
    const int key_size = V * size_x * size_y;
    SZobristHash zobrist(key_size);
    const size_t infeasible_placement_cache_size = infeasible_placement_cache_size_B * 8;
    std::vector<bool> infeasible_placement_cache;
    if (use_cache_for_infeasible_placement_set) {
      LOG(INFO) << fmt::format("use_cache_for_infeasible_placement_set ON {:.2f} MB (smaller cache leads to increasing FP) {}x{}x{}", infeasible_placement_cache_size_B / 1024.0 / 1024.0, size_x, size_y, V);
      infeasible_placement_cache.assign(infeasible_placement_cache_size, false);
    } else {
      LOG(INFO) << "use_cache_for_infeasible_placement_set OFF";
    }
    auto State_to_zobrist = [&](StatePtr s) {
      SZobristHash::key_t hash = 0;
      for (int i = 0; i < V; ++i) {
        if (s->indices_flag[i] == USED) {
          const int dx = s->vertices[i].first - min_x;
          const int dy = s->vertices[i].second - min_y;
          const int idx = (i * size_y + dy) * size_x + dx;
          //CHECK(idx < key_size);
          hash ^= zobrist[idx];
        }
      }
      return hash;
    };
    size_t infeasible_cache_hit_count = 0;
    size_t infeasible_cache_query_count = 0;
    auto is_infeasible_placement = [&](StatePtr s) {
      const bool infeasible = infeasible_placement_cache[State_to_zobrist(s) % infeasible_placement_cache_size];
      ++infeasible_cache_query_count;
      if (infeasible) ++infeasible_cache_hit_count;
      return infeasible;
    };
    auto set_infeasible_placement = [&](StatePtr s) {
      infeasible_placement_cache[State_to_zobrist(s) % infeasible_placement_cache_size] = true;
    };

    std::stack<StatePtr> stack;
    // start from any valid points.
    {
      const int start_index = 0;
      std::vector<int> indices_flag(V, REMAINING);
      indices_flag[start_index] = USED;
      for (auto ip : interior_points) {
        auto vertices = ret.solution->vertices;
        vertices[start_index] = ip;
        stack.push(std::make_shared<State>(1 /* depth */, start_index, ip, indices_flag, vertices));
      }
    }

    const int n_roots = stack.size();
    integer best_dislikes = std::numeric_limits<integer>::max();
    bool found = false;
    int64_t root_counter = 0;
    int64_t counter = 0;
    int max_depth = 1;
    Timer timer;
    double lazy_elapsed_ms = 0.0; // not always updated.
    if (args.timeout_s) {
      LOG(WARNING) << fmt::format("Timeout : {} s", *args.timeout_s);
    }
    if (subsample_roots) {
      LOG(WARNING) << fmt::format("SUBSAMPLING : {}", *subsample_roots);
    }

    while (!stack.empty()) {
      if (!exhaustive_search && found) {
        break;
      }
      StatePtr s = stack.top(); stack.pop();

      if (s->depth == 1) {
        ++root_counter;
        if (subsample_roots && root_counter > *subsample_roots) {
          LOG(ERROR) << fmt::format("SUBSAMPLING DONE! {}", root_counter);
          break;
        }
      }
      ++counter;
      max_depth = std::max(max_depth, s->depth);
      bool report = (counter % report_every_iter == 0);

      if (counter % check_timeout_every_iter == 0 && args.timeout_s) {
        lazy_elapsed_ms = timer.elapsed_ms();
        if (lazy_elapsed_ms * 1e-3 > *args.timeout_s) {
          LOG(ERROR) << fmt::format("TIMEOUT! {} / {} s", lazy_elapsed_ms * 1e-3, *args.timeout_s);
          break;
        }
      }

      if (max_depth == V) {
        auto temp_solution = args.problem->create_solution(s->vertices);
        auto judge_res = judge(*args.problem, *temp_solution);
        if (judge_res.is_valid()) {
          if (judge_res.dislikes < best_dislikes) {
            LOG(INFO) << fmt::format("#{} foud better solution {} -> {}", root_counter, best_dislikes, judge_res.dislikes);
            ret.solution = temp_solution;
            best_dislikes = judge_res.dislikes;
            const std::string filename = args.problem->problem_id ? fmt::format("{}.bestsofar.pose.json", *args.problem->problem_id) : "bestsofar.pose.json";
            save_solution(args.problem, ret.solution, "NaiveSearchSolver", filename);
          }
          found = true;
          report = true;
        }
      }

      if (report) {
        lazy_elapsed_ms = timer.elapsed_ms();
        const double root_per_s = root_counter / lazy_elapsed_ms * 1e3;
        const auto stat = fmt::format("[{}][{}][best DL={}] root {}/{}({:.2f}%) {:.2f} root/s, ETA {:.2f} s, visited {}, max depth {}, {:.2f} ms, {:.2f} node/s, cache {:.2f} MB, infi {}/{}({:.2f}%)",
          args.problem->problem_id ? *args.problem->problem_id : -1, 
          found ? "O" : "X", found ? best_dislikes : -1,
          root_counter, interior_points.size(), 100.0 * root_counter / interior_points.size(), root_per_s,
          double(n_roots) / root_per_s,
          counter, max_depth, lazy_elapsed_ms, counter / lazy_elapsed_ms * 1e3,
          movable_cache_count * sizeof(Point) / 1024.0 / 1024.0,
          infeasible_cache_hit_count, infeasible_cache_query_count, 100.0 * infeasible_cache_hit_count / infeasible_cache_query_count
          );
        LOG(INFO) << stat;
        if (editor) {
          editor->set_oneshot_custom_stat(stat);
          editor->set_marked_indices(create_fixed_indices(s));
          editor->set_pose(args.problem->create_solution(s->vertices));
          editor->show(editor_sleep);
        }
      }

      // first enumerate remaining edge ..
      struct SEdgeItem {
        int num_undetermined_edges = 0;
        int eid = -1;
        // smaller is better.
        bool operator<(const SEdgeItem& rhs) const {
          return num_undetermined_edges != rhs.num_undetermined_edges
            ? num_undetermined_edges > rhs.num_undetermined_edges
            : eid < rhs.eid;
        }
      };
      std::vector<SEdgeItem> remaining_edges;
      for (int eid : edges_from_vertex_cache[s->vid]) {
        auto [u, v] = args.problem->edges[eid];
        auto counter_vid = u == s->vid ? v : u;

        if (s->indices_flag[counter_vid] == REMAINING) {
          // count number of determined and undetermined edges.
          int num_determined_edges = 0;
          int num_undetermined_edges = 0;
          for (auto counter_eid : edges_from_vertex_cache[counter_vid]) {
            auto [j, k] = args.problem->edges[counter_eid];
            auto neighbor_vid = j == counter_vid ? k : j;
            if (s->indices_flag[neighbor_vid] == USED) {
              ++num_determined_edges;
            } else {
              ++num_undetermined_edges;
            }
          }
          remaining_edges.push_back({num_undetermined_edges, eid});
        }
      }
      std::sort(remaining_edges.begin(), remaining_edges.end());

      bool pushed = false;
      for (const SEdgeItem& edge : remaining_edges) {
        const int eid = edge.eid;
        auto [u, v] = args.problem->edges[eid];
        auto counter_vid = u == s->vid ? v : u;

        // push nodes for this unused edge (s->vid, counter_vid)
        auto original_distance2 = distance2(args.problem->vertices[s->vid], args.problem->vertices[counter_vid]);
        auto movable_points = get_movable_points_with_tolerance(s->p, original_distance2);


        for (auto p : movable_points) {
          // all fixed neighbors of counter_vid should agree with this move.
          bool agree = true;
          for (auto counter_eid : edges_from_vertex_cache[counter_vid]) {
            auto [j, k] = args.problem->edges[counter_eid];
            auto neighbor_vid = j == counter_vid ? k : j;
            if (s->indices_flag[neighbor_vid] == USED) {
              const double neighbor_original_distance2 = distance2(args.problem->vertices[neighbor_vid], args.problem->vertices[counter_vid]);
              const double neighbor_distance2 = distance2(s->vertices[neighbor_vid], p);
              if (!tolerate(neighbor_original_distance2, neighbor_distance2, args.problem->epsilon)) {
                agree = false;
                break;
              }
            }
          }
          if (agree) {
            auto new_remaining_index_flag = s->indices_flag;
            new_remaining_index_flag[counter_vid] = USED;
            auto new_vertices = s->vertices;
            new_vertices[counter_vid] = p;
            auto new_s = std::make_shared<State>(s->depth + 1, counter_vid, p, new_remaining_index_flag, new_vertices);
            if (!(use_cache_for_infeasible_placement_set && is_infeasible_placement(new_s))) {
              pushed = true;
              stack.push(new_s);
            }
          }
        }
      }

      if (!pushed) { // failed to push .. infeable placement found.
        if (use_cache_for_infeasible_placement_set) {
          set_infeasible_placement(s);
        }
      }
    }
    LOG(INFO) << fmt::format("total nodes = {}, max depth = {}", counter, max_depth);

    return ret;
  }

};
}

REGISTER_SOLVER("NaiveSearchSolver", NNaiveSearchSolver::NaiveSearchSolver);
// vim:ts=2 sw=2 sts=2 et ci

