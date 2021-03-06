#include "stdafx.h"
#include <boost/geometry.hpp>
#include <boost/geometry/geometries/point_xy.hpp>
#include <boost/geometry/geometries/polygon.hpp>
#include <fmt/format.h>
#include "contest_types.h"
#include "solver_registry.h"
#include "visual_editor.h"

namespace SimpleMatchingSolver {

constexpr int kNumTrialsPerComponent = 1000000;
constexpr int kMinComponentSize = 10;
constexpr int kMinDegree = 2;
constexpr bool kPruneLeavesOnly = true;

namespace bg = boost::geometry;
using BoostPoint = bg::model::d2::point_xy<double>;
using BoostPolygon = bg::model::polygon<BoostPoint>;
using BoostLinestring = bg::model::linestring<BoostPoint>;

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
auto SquaredDistance(const T& vertex0, const U& vertex1) {
  const auto [x0, y0] = vertex0;
  const auto [x1, y1] = vertex1;
  return (x0 - x1) * (x0 - x1) + (y0 - y1) * (y0 - y1);
}

class Solver : public SolverBase {
 public:
  SolverOutputs solve(const SolverArguments& args) override {
    counter_ = 0;
    if (args.visualize) {
      editor_ = std::make_shared<SVisualEditor>(args.problem, "SimpleMatchingSolver", "visualize");
    }

    hole_ = args.problem->hole_polygon;
    vertices_ = args.problem->vertices;
    edges_ = args.problem->edges;
    epsilon_ = args.problem->epsilon;
    const auto hole_polygon = ToBoostPolygon(hole_);

    N_ = vertices_.size();
    queued_.assign(N_, -1);
    assigned_.assign(N_, -1);
    adjacent_.assign(N_, {});
    std::vector<std::vector<double>> distances(
        N_, std::vector<double>(N_, std::numeric_limits<double>::infinity()));
    for (int i = 0; i < N_; ++i) {
      distances[i][i] = 0.0;
    }
    for (const auto& [a, b] : edges_) {
      const auto squared_distance = SquaredDistance(vertices_[a], vertices_[b]);
      const auto margin = epsilon_ * squared_distance / 1'000'000;
      const auto min = squared_distance - margin;
      const auto max = squared_distance + margin;
      adjacent_[a].emplace_back(b, min, max);
      adjacent_[b].emplace_back(a, min, max);
      distances[a][b] = distances[b][a] = std::sqrt(squared_distance) * (1.0 + 1.0e-6 * epsilon_);
    }
    for (int k = 0; k < N_; ++k) {
      for (int i = 0; i < N_; ++i) {
        for (int j = 0; j < N_; ++j) {
          distances[i][j] = std::min(distances[i][j], distances[i][k] + distances[k][j]);
        }
      }
    }
    vertex_max_distances_.resize(N_);
    for (int i = 0; i < N_; ++i) {
      vertex_max_distances_[i].resize(N_);
      for (int j = 0; j < N_; ++j) {
        vertex_max_distances_[i][j] = std::ceil(std::pow(distances[i][j], 2.0));
      }
    }

    M_ = hole_.size();
    hole_candidates_.clear();
    hole_distances_.resize(M_);
    hole_visibilities_.resize(M_);
    for (int i = 0; i < M_; ++i) {
      hole_candidates_.push_back(i);
      hole_distances_[i].resize(M_);
      hole_visibilities_[i].resize(M_);
      for (int j = 0; j < M_; ++j) {
        const BoostLinestring linestring{ToBoostPoint(hole_[i]), ToBoostPoint(hole_[j])};
        hole_visibilities_[i][j] = bg::covered_by(linestring, hole_polygon);
        hole_distances_[i][j] = SquaredDistance(hole_[i], hole_[j]);
      }
    }

    last_updated_ = 0;
    component_sizes_.assign(M_, M_);
    best_sizes_.assign(M_, 0);
    bool found = false;
    for (int i = 0; i < N_; ++i) {
      queued_[i] = N_;
      vertex_candidates_.push_back(i);
      assigned_counts_.assign(1, 0);
      if (Search()) {
        found = true;
        break;
      }
      vertex_candidates_.pop_back();
      queued_[i] = -1;
    }

    SolverOutputs outputs;
    if (found) {
      auto pose = vertices_;
      const auto assigned_vertices = kPruneLeavesOnly ? CleanupLeaves() : Cleanup();
      for (const int vertex : assigned_vertices) {
        pose[vertex] = hole_[assigned_[vertex]];
      }
      outputs.solution = args.problem->create_solution(pose);
    } else {
      outputs.solution = args.problem->create_solution();
    }
    return outputs;
  }

  std::set<int> Cleanup() const {
    std::set<int> assigned_vertices;
    for (int i = 0; i < N_; ++i) {
      if (assigned_[i] < 0) continue;
      assigned_vertices.insert(i);
    }
    while (true) {
      bool converged = true;
      for (int i = 0; i < N_; ++i) {
        if (!assigned_vertices.count(i)) continue;
        int degree = 0;
        for (const auto& [next, min, max] : adjacent_[i]) {
          degree += assigned_vertices.count(next);
        }
        if (degree < kMinDegree) {
          converged = false;
          assigned_vertices.erase(i);
        }
      }
      if (converged) break;
    }
    return assigned_vertices;
  }

  std::set<int> CleanupLeaves() const {
    std::set<int> assigned_vertices;
    for (int i = 0; i < N_; ++i) {
      if (assigned_[i] < 0) continue;
      assigned_vertices.insert(i);
    }
    for (int i = 0; i < N_; ++i) {
      if (assigned_[i] < 0) continue;
      int degree = 0;
      for (const auto& [next, min, max] : adjacent_[i]) {
        degree += assigned_[next] >= 0;
      }
      if (degree < kMinDegree) {
        assigned_vertices.erase(i);
      }
    }
    return assigned_vertices;
  }

  bool Search() {
    if (hole_candidates_.empty()) return true;
    if (++counter_ % 10000 == 0 && editor_) {
      auto pose = vertices_;
      std::vector<int> marked;
      for (int j = 0; j < N_; ++j) {
        if (assigned_[j] < 0) continue;
        pose[j] = hole_[assigned_[j]];
        marked.push_back(j);
      }
      editor_->set_pose(std::make_shared<SSolution>(pose));
      editor_->set_marked_indices(marked);
      editor_->set_persistent_custom_stat(fmt::format("assigned counts = {}", fmt::join(assigned_counts_, ", ")));
      editor_->show(1);
    }
    const int component_index = assigned_counts_.size() - 1;
    best_sizes_[component_index] = std::max(best_sizes_[component_index], assigned_counts_[component_index]);
    if (counter_ > last_updated_ + kNumTrialsPerComponent) {
      last_updated_ = counter_;
      component_sizes_[component_index] = best_sizes_[component_index];
      if (component_sizes_[component_index] < kMinComponentSize) return true;
    }
    for (int i = vertex_candidates_.size() - 1; i >= 0; --i) {
      const int vertex = vertex_candidates_[i];
      vertex_candidates_.erase(vertex_candidates_.begin() + i);
      for (int j = hole_candidates_.size() - 1; j >= 0; --j) {
        const int hole_vertex = hole_candidates_[j];
        bool feasible = true;
        for (const auto& [next, min, max] : adjacent_[vertex]) {
          if (assigned_[next] < 0) continue;
          const auto squared_distance = hole_distances_[assigned_[next]][hole_vertex];
          const auto visible = hole_visibilities_[assigned_[next]][hole_vertex];
          if (!visible || squared_distance < min || squared_distance > max) {
            feasible = false;
            break;
          }
        }
        for (int other = 0; feasible && other < N_; ++other) {
          if (assigned_[other] < 0) continue;
          if (hole_distances_[assigned_[other]][hole_vertex] > vertex_max_distances_[other][vertex]) {
            feasible = false;
          }
        }
        if (feasible) {
          hole_candidates_.erase(hole_candidates_.begin() + j);
          assigned_[vertex] = hole_vertex;
          ++assigned_counts_.back();
          for (const auto& [next, min, max] : adjacent_[vertex]) {
            if (queued_[next] >= 0) continue;
            queued_[next] = vertex;
            vertex_candidates_.push_back(next);
          }
          if (Search()) return true;
          for (const auto& [next, min, max] : adjacent_[vertex]) {
            if (queued_[next] != vertex) continue;
            queued_[next] = -1;
            vertex_candidates_.pop_back();
          }
          --assigned_counts_.back();
          assigned_[vertex] = -1;
          hole_candidates_.insert(hole_candidates_.begin() + j, hole_vertex);
        }
      }
      vertex_candidates_.insert(vertex_candidates_.begin() + i, vertex);
    }
    if (assigned_counts_.back() >= component_sizes_[component_index]) {
      if (assigned_counts_.size() == component_sizes_.size()) return true;
      for (int i = 0; i < N_; ++i) {
        if (queued_[i] >= 0) continue;
        queued_[i] = N_;
        vertex_candidates_.push_back(i);
        assigned_counts_.push_back(0);
        if (Search()) return true;
        assigned_counts_.pop_back();
        vertex_candidates_.pop_back();
        queued_[i] = -1;
      }
    }
    return false;
  }

 private:
  std::vector<Point> hole_;
  std::vector<Point> vertices_;
  std::vector<Edge> edges_;
  integer epsilon_;

  int N_;
  int M_;
  std::vector<int> queued_;
  std::vector<int> assigned_;
  std::vector<int> vertex_candidates_;
  std::vector<int> hole_candidates_;
  std::vector<std::vector<std::tuple<int, integer, integer>>> adjacent_;
  std::vector<std::vector<integer>> hole_distances_;
  std::vector<std::vector<bool>> hole_visibilities_;
  std::vector<std::vector<integer>> vertex_max_distances_;
  std::vector<int> assigned_counts_;
  std::vector<int> component_sizes_;
  std::vector<int> best_sizes_;
  std::size_t last_updated_;
  std::size_t counter_;
  SVisualEditorPtr editor_;
};

}

REGISTER_SOLVER("SimpleMatchingSolver", SimpleMatchingSolver::Solver);
// vim:ts=2 sw=2 sts=2 et ci
