#include "stdafx.h"
#include <chrono>
#include <iostream>

#include <fmt/format.h>

#include "timer.h"
#include "judge.h"

#define USE_BOOST_GEOMETRY
#if defined(USE_BOOST_GEOMETRY)
#include <boost/geometry.hpp>
#include <boost/geometry/geometries/point_xy.hpp>
#include <boost/geometry/geometries/polygon.hpp>

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

void test_bg() {
    typedef boost::geometry::model::polygon<boost::geometry::model::d2::point_xy<double> > polygon;

    polygon blue;
    boost::geometry::read_wkt(
        "POLYGON((0 0, 10 0, 10 5, 5 5, 5 10, 0 10))", blue);

    if (bg::area(blue) < 0.0) {
      bg::reverse(blue);
    }

    BoostLinestring ls({ToBoostPoint(Point(5.0, 8.0)), ToBoostPoint(Point(8.0, 5.0))});

    LOG(INFO) << "linestring coverd by blue?:" << boost::geometry::covered_by(ls, blue);

}
#endif

SJudgeResult judge(const SProblem& problem, const SSolution& solution) {
  SJudgeResult res;
  for (auto& b : solution.bonuses) {
    if (b.type == SBonus::Type::GLOBALIST) {
      res.is_globalist_mode = true;
    }
    if (b.type == SBonus::Type::WALLHACK) {
      res.is_wallhack_mode = true;
    }
    if (b.type == SBonus::Type::SUPERFLEX) {
      res.is_superflex_mode = true;
    }
  }

  // all figure points are inside the hole.
  for (size_t ivert = 0; ivert < solution.vertices.size(); ++ivert) {
    // (c) Every point located on any line segment of the figure in the assumed pose must either lay inside the hole, or on its boundary 
    if (contains(problem.hole_polygon, solution.vertices[ivert]) == EContains::EOUT) {
      res.out_of_hole_vertices.push_back(ivert);
    }
  }

  if (res.is_wallhack_mode) {
    if (res.out_of_hole_vertices.size() == 1) {
      res.wallhacking_index = res.out_of_hole_vertices[0];
    }
  }

  // figure does not intersect with edges of the hole.
#if defined(USE_BOOST_GEOMETRY)
  //test_bg();

  auto hole_polygon_ = ToBoostPolygon(problem.hole_polygon);
  for (size_t iedge = 0; iedge < problem.edges.size(); ++iedge) {
    const auto [a, b] = problem.edges[iedge];
    BoostLinestring linestring{ToBoostPoint(solution.vertices[a]), ToBoostPoint(solution.vertices[b])};
    if (!bg::covered_by(linestring, hole_polygon_)) {
      const bool wallhacking_edge = res.wallhacking_index.value_or(-1) == a || res.wallhacking_index.value_or(-1) == b;
      if (!wallhacking_edge) {
        res.out_of_hole_edges_except_wallhack.push_back(iedge);
      }
      res.out_of_hole_edges.push_back(iedge);
    }
  }
#else

  std::vector<Point2d> hole_polygon_d;
  for (auto& p : problem.hole_polygon) {
    hole_polygon_d.emplace_back(p.first, p.second);
  }

  for (size_t iedge = 0; iedge < problem.edges.size(); ++iedge) {
    const auto& edge = problem.edges[iedge];
    auto moved_i = solution.vertices[edge.first];
    auto moved_j = solution.vertices[edge.second];
    const Line figure_segment = {moved_i, moved_j};

    bool intersects = false;
    for (size_t ihole = 0; ihole < problem.hole_polygon.size(); ++ihole) {
      auto h0 = problem.hole_polygon[ihole];
      auto h1 = problem.hole_polygon[(ihole + 1) % problem.hole_polygon.size()];
      const Line hole_segment = {h0, h1};
      if (intersectSS_strict(figure_segment, hole_segment)) {
        intersects = true;
        break;
      }
    }
    if (!intersects
      && contains(problem.hole_polygon, moved_i) == EContains::EON
      && contains(problem.hole_polygon, moved_j) == EContains::EON
      ) {
      bool on_edge = false;
      for (size_t ihole = 0; ihole < problem.hole_polygon.size(); ++ihole) {
        auto h0 = problem.hole_polygon[ihole];
        auto h1 = problem.hole_polygon[(ihole + 1) % problem.hole_polygon.size()];
        if ((moved_i == h0 && moved_j == h1) ||
            (moved_i == h1 && moved_j == h0)) {
          on_edge = true;
          break;
        }
      }
      if (!on_edge) {
        // if moved_i, moved_j is exactly on the hole polygon, intersectSS / bg::difference always say that
        // the segment is INSIDE the polygon which is wrong when it is concave.
        // dirty hack:
        const double p = 1e-6;
        const Point2d interp {
          get_x(moved_i) * p + get_x(moved_j) * (1.0 - p) ,
          get_y(moved_i) * p + get_y(moved_j) * (1.0 - p) ,
        };
        if (contains(hole_polygon_d, interp) == EContains::EOUT) {
          intersects = true;
        }
      }
    }
    if (intersects) {
      res.out_of_hole_edges.push_back(iedge);
    }
  }
#endif
  
  // stretch
  if (res.is_globalist_mode) {
    double globalist = 0.0;
    for (size_t iedge = 0; iedge < problem.edges.size(); ++iedge) {
      const auto& edge = problem.edges[iedge];
      auto org_i = problem.vertices[edge.first];
      auto org_j = problem.vertices[edge.second];
      auto moved_i = solution.vertices[edge.first];
      auto moved_j = solution.vertices[edge.second];
      globalist += std::abs(double(distance2(moved_i, moved_j)) / double(distance2(org_i, org_j)) - 1);
    }
    if (globalist * 1000000.0 > problem.edges.size() * problem.epsilon) {
      res.violates_globalist = true;
    }
  } else {
    for (size_t iedge = 0; iedge < problem.edges.size(); ++iedge) {
      const auto& edge = problem.edges[iedge];
      auto org_i = problem.vertices[edge.first];
      auto org_j = problem.vertices[edge.second];
      auto moved_i = solution.vertices[edge.first];
      auto moved_j = solution.vertices[edge.second];
      if (!tolerate(distance2(org_i, org_j), distance2(moved_i, moved_j), problem.epsilon)) {
        res.stretch_violating_edges.push_back(iedge);
      }
    }
    if (res.is_superflex_mode && res.stretch_violating_edges.size() == 1) {
      res.superflex_index = res.stretch_violating_edges[0];
    }
  }

  // dislikes
  res.individual_dislikes.assign(problem.hole_polygon.size(), 0);
  for (size_t ihole = 0; ihole < problem.hole_polygon.size(); ++ihole) {
    const auto& h = problem.hole_polygon[ihole];
    integer minval = std::numeric_limits<integer>::max();
    for (size_t ivert = 0; ivert < solution.vertices.size(); ++ivert) {
      const auto& v = solution.vertices[ivert];
      minval = std::min(minval, distance2(h, v));
    }
    res.individual_dislikes[ihole] = minval;
    res.dislikes += minval;
  }

  // gain bonus
  res.gained_bonus_indices.clear();
  for (size_t ibonus = 0; ibonus < problem.bonuses.size(); ++ibonus) {
    for (size_t ivert = 0; ivert < solution.vertices.size(); ++ivert) {
      if (problem.bonuses[ibonus].position == solution.vertices[ivert]) {
        res.gained_bonus_indices.push_back(ibonus);
        break;
      }
    }
  }

  return res;
}

bool update_judge(const SProblem& problem, const SJudgeResult& res, nlohmann::json& solution_json) {
  if (solution_json.find("meta") == solution_json.end()) solution_json["meta"] = {};
  auto& meta_json = solution_json["meta"];
  if (meta_json.find("judge") == meta_json.end()) meta_json["judge"] = {};
  
  meta_json["judge"]["dislikes"] = res.dislikes;
  meta_json["judge"]["fit_in_hole"] = res.fit_in_hole();
  meta_json["judge"]["satisfy_stretch"] = res.satisfy_stretch();
  meta_json["judge"]["is_valid"] = res.is_valid();

  meta_json["judge"]["gained_bonuses"] = nlohmann::json::array();
  for (auto bid : res.gained_bonus_indices) {
    meta_json["judge"]["gained_bonuses"].push_back({
      {"position", problem.bonuses[bid].position},
      {"bonus", SBonus::bonus_name(problem.bonuses[bid].type)},
      {"problem", problem.bonuses[bid].problem_id},
      });
  }

  return true;
}
