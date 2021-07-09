#include "stdafx.h"
#include "timer.h"
#include <chrono>
#include <iostream>

#include <fmt/format.h>

#include "judge.h"

SJudgeResult judge(const SProblem& problem, const SSolution& solution) {
  SJudgeResult res;

  // figure does not intersect with edges of the hole.
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
    if (intersects) {
      res.out_of_hole_edges.push_back(iedge);
    }
  }

  // all figure points are inside the hole.
  for (size_t ivert = 0; ivert < solution.vertices.size(); ++ivert) {
    // (c) Every point located on any line segment of the figure in the assumed pose must either lay inside the hole, or on its boundary 
    if (contains(problem.hole_polygon, solution.vertices[ivert]) == EContains::EOUT) {
      res.out_of_hole_vertices.push_back(ivert);
    }
  }
  
  // stretch
  constexpr integer denominator = 1000000;
  for (size_t iedge = 0; iedge < problem.edges.size(); ++iedge) {
    const auto& edge = problem.edges[iedge];
    auto org_i = problem.vertices[edge.first];
    auto org_j = problem.vertices[edge.second];
    auto moved_i = solution.vertices[edge.first];
    auto moved_j = solution.vertices[edge.second];
    auto a = distance2(moved_i, moved_j);
    auto b = distance2(org_i, org_j);
    // |d(moved) / d(original) - 1| <= eps / 1000000
    if (std::abs(denominator * a - denominator * b) > problem.epsilon * b) {
      res.stretch_violating_edges.push_back(iedge);
    }
  }

  // dislikes
  for (size_t ihole = 0; ihole < problem.hole_polygon.size(); ++ihole) {
    const auto& h = problem.hole_polygon[ihole];
    integer minval = std::numeric_limits<integer>::max();
    for (size_t ivert = 0; ivert < solution.vertices.size(); ++ivert) {
      const auto& v = solution.vertices[ivert];
      minval = std::min(minval, distance2(h, v));
    }
    res.dislikes += minval;
  }

  return res;
}

bool update_judge(const SJudgeResult& res, nlohmann::json& solution_json) {
  if (solution_json.find("meta") == solution_json.end()) solution_json["meta"] = {};
  auto& meta_json = solution_json["meta"];
  if (meta_json.find("juge") == meta_json.end()) meta_json["judge"] = {};
  
  meta_json["judge"]["dislikes"] = res.dislikes;
  meta_json["judge"]["fit_in_hole"] = res.fit_in_hole();
  meta_json["judge"]["satisfy_stretch"] = res.satisfy_stretch();
  meta_json["judge"]["is_valid"] = res.is_valid();

  return true;
}
