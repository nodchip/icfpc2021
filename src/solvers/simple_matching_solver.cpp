#include "stdafx.h"
#include "contest_types.h"
#include "solver_registry.h"

namespace SimpleMatchingSolver {

template <typename T, typename U>
double SquaredDistance(const T& vertex0, const U& vertex1) {
  const auto [x0, y0] = vertex0;
  const auto [x1, y1] = vertex1;
  return (x0 - x1) * (x0 - x1) + (y0 - y1) * (y0 - y1);
}

class Solver : public SolverBase {
 public:
  SolverOutputs solve(const SolverArguments& args) override {
    hole_ = args.problem->hole_polygon;
    vertices_ = args.problem->vertices;
    edges_ = args.problem->edges;
    epsilon_ = args.problem->epsilon;

    N_ = vertices_.size();
    queued_.assign(N_, -1);
    assigned_.assign(N_, -1);
    adjacent_.assign(N_, {});
    for (const auto& [a, b] : edges_) {
      const auto squared_distance = SquaredDistance(vertices_[a], vertices_[b]);
      const auto margin = epsilon_ * squared_distance / 1'000'000;
      const auto min = squared_distance - margin;
      const auto max = squared_distance + margin;
      adjacent_[a].emplace_back(b, min, max);
      adjacent_[b].emplace_back(a, min, max);
    }

    M_ = hole_.size();
    hole_candidates_.clear();
    hole_distances_.resize(M_);
    for (int i = 0; i < M_; ++i) {
      hole_candidates_.push_back(i);
      hole_distances_[i].resize(M_);
      for (int j = 0; j < M_; ++j) {
        hole_distances_[i][j] = SquaredDistance(hole_[i], hole_[j]);
      }
    }

    auto pose = vertices_;
    SolverOutputs outputs;
    outputs.solution = std::make_shared<SSolution>(pose);
    for (int i = 0; i < N_; ++i) {
      queued_[i] = N_;
      vertex_candidates_.push_back(i);
      if (Search()) {
        for (int j = 0; j < N_; ++j) {
          if (assigned_[j] < 0) continue;
          pose[j] = hole_[assigned_[j]];
        }
        outputs.solution = std::make_shared<SSolution>(pose);
        break;
      }
      vertex_candidates_.pop_back();
      queued_[i] = -1;
    }
    return outputs;
  }

  bool Search() {
    if (hole_candidates_.empty()) return true;
    if (vertex_candidates_.empty()) return false;
    for (int i = vertex_candidates_.size() - 1; i >= 0; --i) {
      const int vertex = vertex_candidates_[i];
      vertex_candidates_.erase(vertex_candidates_.begin() + i);
      for (int j = hole_candidates_.size() - 1; j >= 0; --j) {
        const int hole_vertex = hole_candidates_[j];
        bool feasible = true;
        for (const auto& [next, min, max] : adjacent_[vertex]) {
          if (assigned_[next] < 0) continue;
          const auto squared_distance = hole_distances_[assigned_[next]][hole_vertex];
          if (squared_distance < min || squared_distance > max) {
            feasible = false;
            break;
          }
        }
        if (feasible) {
          hole_candidates_.erase(hole_candidates_.begin() + j);
          assigned_[vertex] = hole_vertex;
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
          assigned_[vertex] = -1;
          hole_candidates_.insert(hole_candidates_.begin() + j, hole_vertex);
        }
      }
      vertex_candidates_.insert(vertex_candidates_.begin() + i, vertex);
    }
    return false;
  }

 private:
  std::mt19937 rng_;
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
};

}

REGISTER_SOLVER("SimpleMatchingSolver", SimpleMatchingSolver::Solver);
// vim:ts=2 sw=2 sts=2 et ci
