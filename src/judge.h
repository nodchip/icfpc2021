#pragma once
#include <vector>
#include <optional>
#include <array>
#include "contest_types.h"

template <typename T>
inline T get_x(const std::pair<T, T>& p) { return p.first; }
template <typename T>
inline T get_y(const std::pair<T, T>& p) { return p.second; }

template <typename T>
inline std::pair<T, T>& operator-=(std::pair<T, T>& lhs, const std::pair<T, T>& rhs) {
  lhs.first -= rhs.first;
  lhs.second -= rhs.second;
  return lhs;
}
template <typename T>
inline std::pair<T, T>& operator+=(std::pair<T, T>& lhs, const std::pair<T, T>& rhs) {
  lhs.first += rhs.first;
  lhs.second += rhs.second;
  return lhs;
}
template <typename T>
inline std::pair<T, T> operator-(const std::pair<T, T>& lhs, const std::pair<T, T>& rhs) {
  return { get_x(lhs) - get_x(rhs), get_y(lhs) - get_y(rhs) };
}
template <typename T>
inline std::pair<T, T> operator+(const std::pair<T, T>& lhs, const std::pair<T, T>& rhs) {
  return { get_x(lhs) - get_x(rhs), get_y(lhs) - get_y(rhs) };
}
template <typename T>
inline std::pair<T, T> operator*(const std::pair<T, T>& lhs, double rhs) {
  return { get_x(lhs) * rhs, get_y(lhs) * rhs };
}
template <typename T>
inline std::pair<T, T> operator*(double lhs, const std::pair<T, T>& rhs) {
  return { lhs * get_x(rhs), lhs * get_y(rhs) };
}

template <typename T>
T SQ(T x) { return x * x; }

template <typename TPoint>
inline auto distance2(const TPoint& p, const TPoint& q) {
  return SQ(get_x(p) - get_x(q)) + SQ(get_y(p) - get_y(q));
}

struct SJudgeResult {
  bool is_globalist_mode = false;
  bool is_wallhack_mode = false;
  bool violates_globalist = false;
  integer dislikes = 0;
  std::optional<int> wallhacking_index; // nullopt if not using wallhack (always nullopt when !is_wallhack_mode)
  std::vector<integer> out_of_hole_edges; // wall hacking edges are included
  std::vector<integer> out_of_hole_vertices; // even when wallhacking is used, wall hacking vertex is included
  std::vector<integer> out_of_hole_edges_except_wallhack; // wall hacking edges are NOT included. if !is_wallhack_mode, identical to out_of_hole_edges.
  std::vector<integer> stretch_violating_edges;
  std::vector<integer> individual_dislikes;
  std::vector<integer> gained_bonus_indices; // SProblem::bonus[gained_bonus_indices[i]]
  bool fit_in_hole() const { return out_of_hole_edges.empty() && out_of_hole_vertices.empty(); }
  bool fit_in_hole_except_wallhack() const {
    return out_of_hole_edges_except_wallhack.empty() && (
      out_of_hole_vertices.empty() || out_of_hole_vertices.size() == 1 && wallhacking_index && *wallhacking_index == out_of_hole_vertices[0]
      );
  }
  bool satisfy_stretch() const { 
    if (is_globalist_mode) {
      return !violates_globalist;
    } else {
      return stretch_violating_edges.empty();
    }
  }
  bool is_valid() const {
    if (is_wallhack_mode) {
      return fit_in_hole_except_wallhack() && satisfy_stretch();
    } else {
      return fit_in_hole() && satisfy_stretch();
    }
  }
  // better is smaller.
  friend bool operator<(const SJudgeResult& lhs, const SJudgeResult& rhs) {
    if (lhs.is_valid() != rhs.is_valid()) {
      return int(lhs.is_valid()) > int(rhs.is_valid()); 
    }
    return lhs.dislikes < rhs.dislikes;
  }
};

SJudgeResult judge(const SProblem& problem, const SSolution& solution);

// WARNING! arguments are not interchangeable!
inline bool tolerate(integer d2_original, integer d2_moved, integer epsilon) {
    // |d(moved) / d(original) - 1| <= eps / 1000000
    constexpr integer denominator = 1000000;
    return std::abs(denominator * d2_moved - denominator * d2_original) <= epsilon * d2_original;
}

bool update_judge(const SProblem& problem, const SJudgeResult& res, nlohmann::json& solution_json);

// geometry codes from..
// http://www.prefield.com

template <typename TPoint>
inline auto dot(const TPoint& a, const TPoint& b) {
  return get_x(a) * get_x(b) + get_y(a) * get_y(b);
}
template <typename TPoint>
inline auto norm(const TPoint& a) {
  return dot(a, a);
}
template <typename TPoint>
inline auto cross(const TPoint& a, const TPoint& b) {
  return get_y(a) * get_x(b) - get_x(a) * get_y(b);
}

// http://www.prefield.com/algorithm/geometry/ccw.html
template <typename TPoint>
inline integer ccw(TPoint a, TPoint b, TPoint c) {
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
template <typename TPoint>
inline EContains contains(const std::vector<TPoint>& polygon, const TPoint& p) {
  bool in = false;
  for (int i = 0; i < polygon.size(); ++i) {
    TPoint a = curr(polygon,i) - p, b = next(polygon,i) - p;
    if (get_y(a) > get_y(b)) swap(a, b);
    if (get_y(a) <= 0 && 0 < get_y(b))
      if (cross(a, b) < 0) in = !in;
    if (cross(a, b) == 0 && dot(a, b) <= 0) return EContains::EON;
  }
  return in ? EContains::EIN : EContains::EOUT;
}
#undef curr
#undef next
