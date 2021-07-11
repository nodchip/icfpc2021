#include "stdafx.h"
#include <random>
#include <boost/geometry.hpp>
#include <boost/geometry/geometries/point_xy.hpp>
#include <boost/geometry/geometries/polygon.hpp>
#include "contest_types.h"
#include "solver_registry.h"

namespace NaiveAnnealingSolver {

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

    const int N = vertices_.size();
    auto pose = vertices_;
    double cost = std::numeric_limits<double>::infinity();
    double best_feasible_cost = std::numeric_limits<double>::infinity();
    std::vector<Point> best_feasible_pose;

    const int num_iters = 100000;
    const double T0 = 1.0e1;
    const double T1 = 1.0e-2;
    for (int iter = 0; iter < num_iters; ++iter) {
      const double progress = 1.0 * iter / num_iters;
      const int v = std::uniform_int_distribution(0, N - 1)(rng_);
      const int dx = std::uniform_int_distribution(-3, 3)(rng_);
      const int dy = std::uniform_int_distribution(-3, 3)(rng_);
      auto& [x, y] = pose[v];
      x += dx;
      y += dy;
      const auto [feasible, updated_cost] = Evaluate(pose);
      if (feasible && updated_cost < best_feasible_cost) {
        best_feasible_cost = updated_cost;
        best_feasible_pose = pose;
      }
      const double T = std::pow(T0, 1.0 - progress) * std::pow(T1, progress);
      if (std::uniform_real_distribution(0.0, 1.0)(rng_) < std::exp(-(updated_cost - cost) / T)) {
        cost = updated_cost;
      } else {
        x -= dx;
        y -= dy;
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
      BoostLinestring linestring{ToBoostPoint(pose[a]), ToBoostPoint(pose[b])};
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
      dislikes_cost += best * 1.0e-5;
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

REGISTER_SOLVER("NaiveAnnealingSolver", NaiveAnnealingSolver::Solver);
// vim:ts=2 sw=2 sts=2 et ci
