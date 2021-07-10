#pragma once
#include <vector>
#include <array>
#include "contest_types.h"

inline integer get_x(const Point& p) { return p.first; }
inline integer get_y(const Point& p) { return p.second; }
inline Point& operator-=(Point& lhs, const Point& rhs) {
  lhs.first -= rhs.first;
  lhs.second -= rhs.second;
  return lhs;
}
inline Point& operator+=(Point& lhs, const Point& rhs) {
  lhs.first += rhs.first;
  lhs.second += rhs.second;
  return lhs;
}
inline Point operator-(const Point& lhs, const Point& rhs) {
  return { get_x(lhs) - get_x(rhs), get_y(lhs) - get_y(rhs) };
}
inline Point operator+(const Point& lhs, const Point& rhs) {
  return { get_x(lhs) - get_x(rhs), get_y(lhs) - get_y(rhs) };
}
inline Point operator*(const Point& lhs, double rhs) {
  return { get_x(lhs) * rhs, get_y(lhs) * rhs };
}
inline Point operator*(double lhs, const Point& rhs) {
  return { lhs * get_x(rhs), lhs * get_y(rhs) };
}

template <typename T>
T SQ(T x) { return x * x; }

inline integer distance2(const Point& p, const Point& q) {
  return SQ(get_x(p) - get_x(q)) + SQ(get_y(p) - get_y(q));
}

struct SJudgeResult {
  integer dislikes = 0;
  std::vector<integer> out_of_hole_edges;
  std::vector<integer> out_of_hole_vertices;
  std::vector<integer> stretch_violating_edges;
  std::vector<integer> individual_dislikes;
  bool fit_in_hole() const { return out_of_hole_edges.empty() && out_of_hole_vertices.empty(); }
  bool satisfy_stretch() const { return stretch_violating_edges.empty(); }
  bool is_valid() const { return fit_in_hole() && satisfy_stretch(); }
};

SJudgeResult judge(const SProblem& problem, const SSolution& solution);

bool update_judge(const SJudgeResult& res, nlohmann::json& solution_json);

// geometry codes from..
// http://www.prefield.com

inline integer dot(const Point& a, const Point& b) {
  return get_x(a) * get_x(b) + get_y(a) * get_y(b);
}
inline integer norm(const Point& a) {
  return dot(a, a);
}
//inline integer abs(const Point& a) {
//  return std::sqrt(norm(a)); // not integer..
//}
inline integer cross(const Point& a, const Point& b) {
  return get_y(a) * get_x(b) - get_x(a) * get_y(b);
}

// http://www.prefield.com/algorithm/geometry/ccw.html
inline integer ccw(Point a, Point b, Point c) {
  b -= a; c -= a;
  if (cross(b, c) > 0)   return +1;       // counter clockwise
  if (cross(b, c) < 0)   return -1;       // clockwise
  if (dot(b, c) < 0)     return +2;       // c--a--b on line
  if (norm(b) < norm(c)) return -2;       // a--b--c on line
  return 0;
}

// http://www.prefield.com/algorithm/geometry/intersection.html 
constexpr integer EPS = 0;
inline bool intersectLL(const Line &l, const Line &m) {
  return abs(cross(l[1]-l[0], m[1]-m[0])) > EPS || // non-parallel
         abs(cross(l[1]-l[0], m[0]-l[0])) < EPS;   // same line
}
inline bool intersectLS(const Line &l, const Line &s) {
  return cross(l[1]-l[0], s[0]-l[0])*       // s[0] is left of l
         cross(l[1]-l[0], s[1]-l[0]) < EPS; // s[1] is right of l
}
inline bool intersectLP(const Line &l, const Point &p) {
  return abs(cross(l[1]-p, l[0]-p)) < EPS;
}
inline bool intersectSS(const Line &s, const Line &t) {
  return ccw(s[0],s[1],t[0])*ccw(s[0],s[1],t[1]) <= 0 &&
         ccw(t[0],t[1],s[0])*ccw(t[0],t[1],s[1]) <= 0;
}
inline bool intersectSS_strict(const Line &s, const Line &t) { // false if two segments ar ON.
  return ccw(s[0],s[1],t[0])*ccw(s[0],s[1],t[1]) < 0 &&
         ccw(t[0],t[1],s[0])*ccw(t[0],t[1],s[1]) < 0;
}
//inline bool intersectSP(const Line &s, const Point &p) {
//  return abs(s[0]-p)+abs(s[1]-p)-abs(s[1]-s[0]) < EPS; // triangle inequality
//}

// http://www.prefield.com/algorithm/geometry/contains.html
#define curr(P, i) P[i]
#define next(P, i) P[(i+1)%P.size()]
enum EContains {
  EOUT, EON, EIN
};
inline EContains contains(const std::vector<Point>& polygon, const Point& p) {
  bool in = false;
  for (int i = 0; i < polygon.size(); ++i) {
    Point a = curr(polygon,i) - p, b = next(polygon,i) - p;
    if (get_y(a) > get_y(b)) swap(a, b);
    if (get_y(a) <= 0 && 0 < get_y(b))
      if (cross(a, b) < 0) in = !in;
    if (cross(a, b) == 0 && dot(a, b) <= 0) return EContains::EON;
  }
  return in ? EContains::EIN : EContains::EOUT;
}
#undef curr
#undef next
