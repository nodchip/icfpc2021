#include "stdafx.h"
#include "contest_types.h"
#include "judge.h"
#include "timer.h"
#include "solver_registry.h"

#include <boost/geometry.hpp>
#include <queue>
#include <set>
#include <fmt/core.h>
#include "visual_editor.h"
using vec = std::vector<integer>;
using mat = std::vector<vec>;

integer lower_limit(integer len_given, integer epsilon) {
  auto tmp = len_given * epsilon;
  tmp /= 1000000;
  return len_given - tmp;
}

integer upper_limit(integer len_given, integer epsilon) {
  auto tmp = len_given * epsilon;
  tmp /= 1000000;
  return len_given + tmp;
}



mat create_vertices_distances(SProblemPtr problem) {
  int V = problem->vertices.size();
  mat vertices_distances(V, vec(V, 0));

  for (const auto& e : problem->edges) {
    vertices_distances[e.first][e.second] = distance2(problem->vertices[e.first], problem->vertices[e.second]);
    vertices_distances[e.second][e.first] = vertices_distances[e.first][e.second];
  }
  return vertices_distances;

}
struct vec_holenum {
  std::vector<int> v2h;
  std::vector<int> h2v;
  int H;
  int get_holenum(int x) const { return v2h[x]; }
  int get_vecnum(int x) const { return h2v[x]; }

  vec_holenum(int H_) : H(H_) {
    for (int i = 0; i < H; i++) v2h.push_back(i);
    for (int i = 0; i < H; i++) h2v.push_back(i);
  }

  void Swap(int a, int b) {
    if (a == b) return;
    int A = v2h[a], B = v2h[b];
    v2h[a] = B;
    v2h[b] = A;
    h2v[A] = b;
    h2v[B] = a;
  }

  void Change_V2H(int v, int h) {
    if (v2h[v] == h) return;
    int prev_v = h2v[h];
    Swap(v, prev_v);
  }

  std::vector<int> decided_points(const std::vector<int>& v) const {
    std::vector<int> ret(H);
    for (int i = 0; i < H; i++) ret[v2h[i]] = v[i];
    return ret;
  }
  void shuffle(std::mt19937_64& rng) {
    std::uniform_int_distribution rand(0, H - 1);
    Swap(0, rand(rng));
    Swap(1, rand(rng));
    for (int i = 0; i < 5; i++) Swap(rand(rng), rand(rng));
    //for (int i = 0; i < H; i++) LOG(INFO) << v2h[i] << ' ' << h2v[i];
  }
  void check_valid() const {
    for (int i = 0; i < H; i++) LOG(INFO) << i << ' ' << get_vecnum(i) << ' ' << get_holenum(get_vecnum(i));
  }
};


struct SVOI {
public:
  integer x_min;
  integer x_max;
  integer y_min;
  integer y_max;

  SVOI() {
    x_min = INT64_MAX, x_max = INT64_MIN;
    y_min = INT64_MAX, y_max = INT64_MIN;
  }
  SVOI(const SVOI& rhs) : x_min(rhs.x_min), x_max(rhs.x_max), y_min(rhs.y_min), y_max(rhs.y_max) {}
  SVOI(const std::vector<Point>& points) {
    x_min = INT64_MAX, x_max = INT64_MIN;
    y_min = INT64_MAX, y_max = INT64_MIN;
    for (const auto [x, y] : points) {
      x_min = std::min(x, x_min);
      x_max = std::max(x, x_max);
      y_min = std::min(y, y_min);
      y_max = std::max(y, y_max);
      x_max++;
      y_max++;
    }
  }
  SVOI(const Point& point, integer length) {
    integer len = std::floor(std::sqrt(length)) + 1;
    x_min = get_x(point) - len;
    x_max = get_x(point) + len + 1;
    y_min = get_y(point) - len;
    y_max = get_y(point) + len + 1;

  }
  SVOI operator&(SVOI rhs) const {
    SVOI ret;
    ret.x_min = std::max(x_min, rhs.x_min);
    ret.x_max = std::min(x_max, rhs.x_max);
    ret.y_min = std::max(y_min, rhs.y_min);
    ret.y_max = std::min(y_max, rhs.y_max);
    return ret;
  }
  void operator&=(SVOI rhs) {
    x_min = std::max(x_min, rhs.x_min);
    x_max = std::min(x_max, rhs.x_max);
    y_min = std::max(y_min, rhs.y_min);
    y_max = std::min(y_max, rhs.y_max);
  }
  SVOI operator^(SVOI rhs) const {
    SVOI ret;
    ret.x_min = std::min(x_min, rhs.x_min);
    ret.x_max = std::max(x_max, rhs.x_max);
    ret.y_min = std::min(y_min, rhs.y_min);
    ret.y_max = std::max(y_max, rhs.y_max);
    return ret;
  }
  void operator^=(SVOI rhs) {
    x_min = std::min(x_min, rhs.x_min);
    x_max = std::max(x_max, rhs.x_max);
    y_min = std::min(y_min, rhs.y_min);
    y_max = std::max(y_max, rhs.y_max);
  }

  bool is_valid() {
    if (x_min >= x_max) return false;
    if (y_min >= y_max) return false;
    return true;
  }

  integer size() {
    return this->is_valid() ? (x_max - x_min) * (y_max - y_min) : -1;
  }

  std::vector<Point> const all_points() {
    std::vector<Point> ret;
    for (auto x = x_min; x < x_max; x++) for (auto y = y_min; y < y_max; y++) ret.push_back(std::make_pair(x, y));
    return ret;
  }

};

constexpr integer LARGE = 1e15;
constexpr integer VERYLARGE = 1e17;

struct Striction {
  int vertex;
  std::vector<Point> consider_points;
  std::vector<int> use;
  Striction(int vertex_, int Size) : vertex(vertex_) {
    consider_points.resize(Size, std::make_pair(0, 0));
    use.resize(Size, false);
  }
  Striction(const Striction& rhs) : vertex(rhs.vertex), consider_points(rhs.consider_points), use(rhs.use) {}
  void Add(int x, Point p, const mat& vertices_distances) { if (vertices_distances[x][vertex])consider_points[x] = p; use[x] = 1; }
  void Delete(int x) { consider_points[x] = std::make_pair(0, 0); use[x] = 0; }
  void Set(const std::vector<int> cand_points, SSolutionPtr solution, const mat& vertices_distances) {
    for (auto e : cand_points) if (vertices_distances[e][vertex]) {
      use[e] = true;
      consider_points[e] = solution->vertices[e];
    }
  }
  bool operator<(const Striction& rhs) const {
    if (vertex != rhs.vertex) return vertex < rhs.vertex;
    return consider_points != rhs.consider_points ? consider_points < rhs.consider_points : use < rhs.use;
  }
  bool operator>(const Striction& rhs) const {
    if (vertex != rhs.vertex) return vertex > rhs.vertex;
    return consider_points != rhs.consider_points ? consider_points > rhs.consider_points : use > rhs.use;
  }
  bool operator==(const Striction& rhs) const {
    if (vertex != rhs.vertex) return false;
    return consider_points != rhs.consider_points ? false : use == rhs.use;
  }
};
std::ostream& operator<<(std::ostream& os, const Striction& s) {
  os << std::endl;
  os << "vertex:::" << s.vertex << std::endl;
  for (int i = 0; i < s.use.size(); i++) if (s.use[i]) os << "i is " << i;
  return os;
}



struct MapValue {
  bool available;
  SVOI voi;
  std::vector<Point> cands;
  MapValue() : available(false) {
    cands = {};
  }
  MapValue(const SVOI& voi_, const std::vector<Point> cands_) : available(true), voi(voi_), cands(cands_) {}
  //MapValue(const MapValue& rhs) : available(rhs.available), voi(rhs.voi), cands(rhs.cands) {}

};

using HashMap = std::map<Striction, MapValue>;
bool is_point_valid(SProblemPtr problem, SSolutionPtr solution, const Striction& striction, const mat& vertices_distances, Point& point) {
  int vertex = striction.vertex;
  for (int e = 0; e < striction.consider_points.size(); e++) if (striction.use[e]) {
    auto length = distance2(solution->vertices[e], point);
    if (!tolerate(vertices_distances[e][vertex], length, problem->epsilon)) return false;
  }
  return true;
}



MapValue able_points(SProblemPtr problem, SSolutionPtr solution, const Striction& striction, SVOI voi, const mat& vertices_distances, long long& counter) {
  counter++;
  //LOG(INFO) << "arrived able";
  std::vector<Point> ret = {};
  if (counter > LARGE) return MapValue(voi, ret);
  int vertex = striction.vertex;
  int min_e = -1;
  integer min_len = 1e18;
  for (int e = 0; e < striction.consider_points.size(); e++) if (striction.use[e]) {
    SVOI tmp(solution->vertices[e], upper_limit(vertices_distances[e][vertex], problem->epsilon));
    voi &= tmp;
    if (min_len > vertices_distances[e][vertex]) {
      min_e = e;
      min_len = vertices_distances[e][vertex];
    }
  }
  //LOG(INFO) << "voi valid is " << voi.is_valid();
  //LOG(INFO) << "min_e is " << min_e;
  if (!voi.is_valid()) return MapValue(voi, ret);
  //LOG(INFO) << "voi size is " << voi.size();
  if (min_e < 0) return MapValue();
  integer e_max2 = upper_limit(vertices_distances[min_e][vertex], problem->epsilon);
  integer e_min2 = lower_limit(vertices_distances[min_e][vertex], problem->epsilon);
  for (integer y = voi.y_min; y < voi.y_max; y++) {
    integer X = solution->vertices[min_e].first;
    integer len = abs(solution->vertices[min_e].second - y);
    integer len2 = (len * len);
    if (len2 <= e_max2) {
      integer max_len2 = e_max2 - len2;
      integer min_len2 = e_min2 - len2;
      integer max_len = std::ceil(std::sqrt(max_len2));

      if (min_len2 <= 0) {
        integer x_min = std::max(X - max_len, voi.x_min);
        integer x_max = std::min(X + max_len + 1, voi.x_max);
        for (integer x = x_min; x < x_max; x++) {
          Point point = std::make_pair(x, y);
          if (is_point_valid(problem, solution, striction, vertices_distances, point)) ret.push_back(point);
        }
      }

      else {
        integer min_len = std::floor(std::sqrt(min_len2));
        integer x_min = std::max(X - max_len, voi.x_min);
        integer x_max = std::min(X - min_len + 1, voi.x_max);
        for (integer x = x_min; x < x_max; x++) {
          Point point = std::make_pair(x, y);
          if (is_point_valid(problem, solution, striction, vertices_distances, point)) ret.push_back(point);
        }
        x_min = std::max(X + min_len, voi.x_min);
        x_max = std::min(X + max_len + 1, voi.x_max);
        for (integer x = x_min; x < x_max; x++) {
          Point point = std::make_pair(x, y);
          if (is_point_valid(problem, solution, striction, vertices_distances, point)) ret.push_back(point);
        }
      }


    }
  }
  return MapValue(voi, ret);
}


std::vector<Point> able_points_hash(SProblemPtr problem, SSolutionPtr solution, const Striction& str_holes, const Striction& str_inside, HashMap& hash_map_decided, SVOI voi, const mat& vertices_distances, HashMap& hash_map_unrestricted, long long& counter) {
  counter++;
  //LOG(INFO) << "arrived hash able";
  std::vector<Point> ret = {};
  if (counter > LARGE) return ret;
  int vertex = str_holes.vertex;
  if (hash_map_unrestricted.count(str_inside)) return hash_map_unrestricted[str_inside].cands;
  if (!hash_map_decided.count(str_holes)) {
    for (int e = 0; e < str_holes.consider_points.size(); e++) {
      if (str_holes.use[e]) {
        SVOI tmp(solution->vertices[e], upper_limit(vertices_distances[e][vertex], problem->epsilon));
        voi &= tmp;
      }
    }
    if (!voi.is_valid()) {
      hash_map_decided[str_holes] = MapValue(voi, ret);
      return ret;
    }
    hash_map_decided[str_holes] = able_points(problem, solution, str_holes, voi, vertices_distances, counter);
  }

  auto hash_value = hash_map_decided[str_holes];
  if (hash_value.available) {
    auto cand_points = hash_value.cands;
    //LOG(INFO) << cand_points.size();
    voi = hash_value.voi;
    if (cand_points.size() == 0) {
      hash_map_unrestricted[str_inside] = MapValue(voi, ret);
      return ret;
    }

    for (auto e : cand_points) {
      if (is_point_valid(problem, solution, str_inside, vertices_distances, e)) ret.push_back(e);
    }
    hash_map_unrestricted[str_inside] = MapValue(voi, ret);
    return ret;
  }

  else {
    MapValue mpv(able_points(problem, solution, str_inside, voi, vertices_distances, counter));
    hash_map_unrestricted[str_inside] = mpv;
    return mpv.cands;
  }
}
SSolutionPtr dfs_able_points(SProblemPtr problem, SSolutionPtr solution, const std::vector<Striction>& str_hole_memo, std::vector<int>& decided_points_inside, HashMap& hash_map_decided, std::vector<int> restriction_edges, const mat& vertices_distances, int V, HashMap& hash_map_unrestricted, long long& counter) {
  counter++;
  int H = problem->hole_polygon.size();
  //LOG(INFO) << "dfs_able_points "<<decided_points_inside.size();
#if 0
  SVisualEditor debug(problem);
  debug.set_pose(solution);
  auto key = debug.show(0);
  if (key == 'x') counter += LARGE;
  if (key == 'q') counter += VERYLARGE;
#endif
  if (counter > LARGE) return nullptr;
  //if (counter % integer(1e4) == 0) LOG(INFO) << "counter is (able_points) " << counter;
  //if (counter > 1e9) return nullptr;
  if (decided_points_inside.size() == V - H) {
    auto judgement = judge(*problem, *solution);
    if (judgement.is_valid()) return solution;
    return nullptr;
  }
  int most_restricted_point = -1;
  int memo = -1;
  for (int i = 0; i < V; i++) if (memo < restriction_edges[i]) {
    memo = restriction_edges[i];
    most_restricted_point = i;
  }
  SVOI voi(problem->hole_polygon);
  Striction str_inside(most_restricted_point, V);
  auto cands = able_points_hash(problem, solution, str_hole_memo[most_restricted_point], str_inside, hash_map_decided, voi, vertices_distances, hash_map_unrestricted, counter);
  if (cands.size() == 0) return nullptr;
  decided_points_inside.push_back(most_restricted_point);
  for (int i = 0; i < V; i++) if (vertices_distances[most_restricted_point][i] && restriction_edges[i] >= 0) restriction_edges[i]++;
  restriction_edges[most_restricted_point] = -10000000;
  for (auto point : cands) {
    solution->vertices[most_restricted_point] = point;
    auto sol = dfs_able_points(problem, solution, str_hole_memo, decided_points_inside, hash_map_decided, restriction_edges, vertices_distances, V, hash_map_unrestricted, counter);
    if (sol) return sol;
    solution->vertices[most_restricted_point] = problem->vertices[most_restricted_point];
  }
  decided_points_inside.pop_back();
  return nullptr;


}


SSolutionPtr Fit_Unstricted_Points(SProblemPtr problem, SSolutionPtr solution, const std::vector<int>& decided_points, const mat& vertices_distances, HashMap& hash_map_decided, long long& counter) {
  int V = problem->vertices.size();
  std::vector<bool> used_vertices(V, false);
  for (auto e : decided_points) used_vertices[e] = true;
  std::vector<int> restriction_edges(V, 0);
  for (auto e : decided_points) for (int i = 0; i < V; i++) if (vertices_distances[i][e]) restriction_edges[i]++;
  for (auto e : decided_points) restriction_edges[e] = -100000;
  std::vector<int> decided_points_inside = {};
  std::vector<Striction> str_hole_memo;
  for (int i = 0; i < V; i++) str_hole_memo.push_back(Striction(i, V));
  //LOG(INFO) << str_hole_memo.size() << "SIZE==============";
  for (int i = 0; i < V; i++) if (!used_vertices[i]) {
    str_hole_memo[i].Set(decided_points, solution, vertices_distances);
  }
  HashMap hash_map_unstricted = {};
  auto ptr_ans = dfs_able_points(problem, solution, str_hole_memo, decided_points_inside, hash_map_decided, restriction_edges, vertices_distances, V, hash_map_unstricted, counter);
  //LOG(INFO) << "hash_map_unstricted size is " << hash_map_unstricted.size();
  return ptr_ans;

}


SSolutionPtr FitUnstrictedPoints(SProblemPtr problem, SSolutionPtr solution, const std::vector<int> decided_points) {
  long long counter = 0;
  HashMap hash_map_decided = {};
  return Fit_Unstricted_Points(problem, solution, decided_points, create_vertices_distances(problem), hash_map_decided, counter);
}

SSolutionPtr dfs_holes(SProblemPtr problem, SSolutionPtr solution, HashMap& hash_map_decided, const mat& hole_distances, const mat& vertices_distances, const std::vector<std::vector<double>>& upperlimit_distances, std::vector<int>& v, int V, int H, std::vector<int>& used_vertices, const vec_holenum& v_h, long long& counter) {
  constexpr int one_min = 1000 * 60;
  counter++;
  if (/*counter % 50 == 0*/ false) {
    SVisualEditor debug(problem, "full_research", "dfs_holes");
    debug.set_pose(solution);
    auto key = debug.show(3);
    if (key == 'x') counter += LARGE;
    if (key == 'q') counter += VERYLARGE;
  }
  if (counter > LARGE) return nullptr;

  //if(counter % integer(1e4) == 0) LOG(INFO) << "counter is (holes)" << counter;
  //if (counter > 1e9) return nullptr;
  //LOG(INFO) << "v.size is : " << v.size();
  //if (timer.elapsed_ms() > one_min) return nullptr;
  if (v.size() >= H) {
    return Fit_Unstricted_Points(problem, solution, v, vertices_distances, hash_map_decided, counter);
  }
  else {
    for (int x = 0; x < V; x++) if (!used_vertices[x]) {
      if (counter > 1e15) return nullptr;
      v.push_back(x);
      bool okay = true;
      used_vertices[x]++;
      int hole_x = v_h.get_holenum(v.size() - 1);
      for (int i = 0; i < v.size() - 1; i++) {
        int hole_i = v_h.get_holenum(i);
        int y = v[i];
        auto ver_dist = vertices_distances[x][y];
        auto uplimit_dist = upperlimit_distances[x][y];
        auto hole_dist = hole_distances[hole_i][hole_x];
        //LOG(INFO) << fmt::format("Problem  : {}, {}, {}", x,y);
        if (ver_dist && !tolerate(ver_dist, hole_dist, problem->epsilon)) okay = false;
        if (uplimit_dist + 1 < std::sqrt(hole_dist)) okay = false;
        if (!okay) break;
      }
      //counter++;
      if (okay) {
        //LOG(INFO) << "dfs_holes:: v is ";
        //for (auto e : v) LOG(INFO) << e;
        solution->vertices[x] = problem->hole_polygon[hole_x];
        auto sol = dfs_holes(problem, solution, hash_map_decided, hole_distances, vertices_distances, upperlimit_distances, v, V, H, used_vertices, v_h, counter);
        if (sol) return sol;
        solution->vertices[x] = problem->vertices[x];
      }
      v.pop_back();
      used_vertices[x]--;
    }
  }
  return nullptr;
}



#if 1
bool full_research(SProblemPtr problem, SSolutionPtr& solution) {
  LOG(INFO) << "CALLED FULL RESEARCH";
  int V = problem->vertices.size();
  int H = problem->hole_polygon.size();
  LOG(INFO) << "V,H is " << V << ' ' << H;
  unsigned int seed = 20210711;
  std::mt19937_64 rng(seed);

  constexpr double BIGDOUBLE = 1e18;
  vec_holenum v_h(H);

  mat vertices_distances = create_vertices_distances(problem);
  std::vector<std::vector<double>> upperlimit_distances(V, std::vector<double>(V, BIGDOUBLE));

  for (const auto& e : problem->edges) {
    upperlimit_distances[e.first][e.second] = std::sqrt(upper_limit(distance2(problem->vertices[e.first], problem->vertices[e.second]), problem->epsilon));
    upperlimit_distances[e.second][e.first] = upperlimit_distances[e.first][e.second];
  }

  for (int k = 0; k < V; k++) for (int i = 0; i < V; i++) for (int j = 0; j < V; j++) {
    upperlimit_distances[i][j] = std::min(upperlimit_distances[i][j], upperlimit_distances[i][k] + upperlimit_distances[k][j]);
  }

  mat hole_distances(H, vec(H, 0));
  for (int i = 0; i < H; i++) for (int j = 0; j < H; j++) if (i != j) hole_distances[i][j] = distance2(problem->hole_polygon[i], problem->hole_polygon[j]);
  long long counter = 0;
  std::vector<std::vector<int>> ret;
  std::vector<int> used_vertices(V, 0);
  std::vector<int> v = {};
  SSolutionPtr solution_initial(new SSolution(solution->vertices));
  std::vector<int> decided_points_initial = {};
  for (int i = 0; i < V; i++) if (solution->vertices[i] != problem->vertices[i]) {
    decided_points_initial.push_back(i);
    used_vertices[i]++;
  }
  int hole_same_counter = 0;
  for (int i = 0; i < decided_points_initial.size(); i++) {
    auto e = decided_points_initial[i];
    Point point = solution_initial->vertices[e];
    for (int j = 0; j < H; j++) {
      if (point == problem->hole_polygon[j]) {
        v_h.Change_V2H(i, j);
        hole_same_counter++;
        break;
      }
    }
  }
  //  v_h.check_valid();

  std::vector<int> v_init_memo;
  for (auto e : decided_points_initial) v_init_memo.push_back(e);


  LOG(INFO) << "start decided is " << decided_points_initial.size();
  HashMap hash_map_decided = {};
  auto sol = dfs_holes(problem, solution_initial, hash_map_decided, hole_distances, vertices_distances, upperlimit_distances, decided_points_initial, V, H, used_vertices, v_h, counter);
  if (sol) {
    solution = sol;
    LOG(INFO) << "hash_map_size is" << hash_map_decided.size();
    return true;
  }

#if 0

  for (auto e : v_init_memo) {
    LOG(INFO) << "arrived: changing e is " << e;
    SSolutionPtr sol(new SSolution(solution->vertices));
    sol->vertices[e] = problem->vertices[e];
    std::vector<int> dec = {};
    for (auto x : v_init_memo) if (x != e) dec.push_back(x);
    for (int i = 0; i < dec.size(); i++) {
      auto e = dec[i];
      Point point = sol->vertices[e];
      for (int j = 0; j < H; j++) {
        if (point == problem->hole_polygon[j]) {
          v_h.Change_V2H(i, j);
          hole_same_counter++;
          break;
        }
      }
    }
    SVisualEditor debug(problem, "fullresearch", "main");
    debug.set_pose(sol);
    auto key = debug.show(100);
    std::vector<int> used(V, 0);
    for (auto x : dec) if (x != e) used[x]++;
    sol = dfs_holes(problem, sol, hole_distances, vertices_distances, upperlimit_distances, dec, V, H, timer, used, v_h, counter);
    if (sol) {
      solution = sol;
      return true;
    }


  }

#endif





  return false;

}



class FullResearch : public SolverBase {
public:
  FullResearch() { }
  SolverOutputs solve(const SolverArguments& args) override {
    SolverOutputs ret;

    //Timer timer;
    ret.solution = args.problem->create_solution();
    if (args.optional_initial_solution) ret.solution = args.optional_initial_solution;
    full_research(args.problem, ret.solution);
    LOG(INFO) << "finish fullresearch";
    // dummy.
    //std::this_thread::sleep_for(std::chrono::milliseconds(500));

    return ret;
  }
};

REGISTER_SOLVER("FullResearch", FullResearch);
#endif



namespace SimpleMatchFullResearch {

constexpr int kNumTrialsPerComponent = 100000;
constexpr int kMinComponentSize = 10;
constexpr int kMinDegree = 2;

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
    auto pose = vertices_;
    for (int i = 0; i < N_; ++i) {
      queued_[i] = N_;
      vertex_candidates_.push_back(i);
      assigned_counts_.assign(1, 0);
      if (Search()) {
        for (int j = 0; j < N_; ++j) {
          if (assigned_[j] < 0) continue;
          pose[j] = hole_[assigned_[j]];
        }
        break;
      }
      vertex_candidates_.pop_back();
      queued_[i] = -1;
    }

        //Cleanup(0);
    auto clean_candidates = Cleanup();
    RevertClean(clean_candidates);

    for (int i = 0; i < 2; i++) {
      auto tmp_pose = pose;
      if (i) for (auto e : clean_candidates) tmp_pose[e.first] = args.problem->vertices[e.first];
      SSolutionPtr solution_simple = std::make_shared<SSolution>(tmp_pose);
      auto solved = full_research(args.problem, solution_simple);
      if (solved) return  SolverOutputs{ solution_simple };
    }
    return SolverOutputs{ std::make_shared<SSolution>(pose) };
  }


  std::vector<std::pair<int,int>> Cleanup(int max_clean = 1000) {
    std::vector<std::pair<int,int>> ret = {};
    if (!max_clean) return ret;
    int counter = 0;
    while (true) {
      bool converged = true;
      for (int i = 0; i < N_; ++i) {
        if (assigned_[i] < 0) continue;
        int degree = 0;
        for (const auto& [next, min, max] : adjacent_[i]) {
          degree += assigned_[next] >= 0;
        }
        if (degree < kMinDegree) {
          ret.push_back(std::make_pair(i,assigned_[i]));
          converged = false;
          assigned_[i] = -1;
          counter++;
          if (counter >= max_clean) return ret;
        }
      }
      if (converged) return ret;
    }
  }
  void Clean(const std::vector<std::pair<int, int>>& rev) {
    for (auto e : rev) assigned_[e.first] = -1;

  }

  void RevertClean(const std::vector<std::pair<int,int>>& rev) {
    for (auto e : rev) assigned_[e.first] = e.second;
  }

  bool Search() {
    if (hole_candidates_.empty()) return true;
    if (++counter_ % 1000 == 0 && editor_) {
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
REGISTER_SOLVER("SimpleMatchFullResearch", SimpleMatchFullResearch::Solver);

