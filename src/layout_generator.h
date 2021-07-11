#pragma once
#include <sstream>
#include <fmt/format.h>
#include "util.h"
#include "contest_types.h"

#ifdef _MSC_VER
#include <ppl.h>
#endif

namespace NLayoutGenerator {

  constexpr double eps = 1e-8;

  struct Xorshift {
    uint64_t x = 88172645463325252LL;
    inline void set_seed(unsigned seed, int rep = 100) {
      x = (seed + 1) * 10007;
      for (int i = 0; i < rep; i++) next_int();
    }
    inline unsigned next_int() {
      x = x ^ (x << 7);
      return x = x ^ (x >> 9);
    }
    inline unsigned next_int(unsigned mod) {
      x = x ^ (x << 7);
      x = x ^ (x >> 9);
      return x % mod;
    }
    inline unsigned next_int(unsigned l, unsigned r) {
      x = x ^ (x << 7);
      x = x ^ (x >> 9);
      return x % (r - l + 1) + l;
    }
    inline double next_double() {
      return double(next_int()) / std::numeric_limits<unsigned>::max();
    }
  };

  struct P {
    double x, y;
    P() : x(0.0), y(0.0) {}
    P(double x, double y) : x(x), y(y) {}
    inline P operator+() const { return P(x, y); }
    inline P operator-() const { return P(-x, -y); }
    inline P& operator+=(const P& p) {
      x += p.x; y += p.y;
      return *this;
    }
    inline P& operator-=(const P& p) {
      x -= p.x; y -= p.y;
      return *this;
    }
    inline P& operator*=(double k) {
      x *= k; y *= k;
      return *this;
    }
    inline P& operator/=(double k) {
      x /= k; y /= k;
      return *this;
    }
    inline double length2() const { return x * x + y * y; };
    inline double length() const { return sqrt(length2()); }
    inline P unit() const { return P(*this) /= length(); }
    inline std::string str() const {
      return "(" + std::to_string(x) + ", " + std::to_string(y) + ")";
    }
    inline friend std::ostream& operator<<(std::ostream& o, const P& obj) {
      o << obj.str();
      return o;
    }
  };
  inline P operator+(const P& a, const P& b) { return P(a) += b; }
  inline P operator-(const P& a, const P& b) { return P(a) -= b; }
  inline P operator*(const P& p, double k) { return P(p) *= k; }
  inline P operator*(double k, const P& p) { return P(p) *= k; }
  inline P operator/(const P& p, double k) { return P(p) /= k; }
  inline P projection(const P& ps, const P& pt, const P& p) {
    P pst = pt - ps;
    double det = pst.length2();
    double a = pst.y * ps.x - pst.x * ps.y, b = pst.y * p.y + pst.x * p.x;
    double x = pst.y * a + pst.x * b, y = pst.y * b - pst.x * a;
    return P(x / det, y / det);
  }

  struct SEdge {
    int from, to;
    double orig_len;
    SEdge() {}
    SEdge(int u, int v, double d) : from(u), to(v), orig_len(d) {}
    inline std::string str() const { return fmt::format("SEdge [from={}, to={}, orig_len={}]", from, to, orig_len); }
    inline friend std::ostream& operator<<(std::ostream& o, const SEdge& obj) { o << obj.str(); return o; }
    inline uint64_t hash() const {
      static constexpr uint64_t p = 31;
      uint64_t res = 17;
      res = p * res + from;
      res = p * res + to;
      uint64_t* idbl = (uint64_t*)&orig_len;
      res = p * res + *idbl;
      return res;
    }
  };

  struct SNode {
    int id;
    P r, v;
    std::vector<SEdge> es;
    SNode() {}
    inline std::string str() const {
      std::ostringstream ses;
      ses << es;
      return fmt::format("SNode [r={}, v={}, es={}]", r.str().c_str(), v.str().c_str(), ses.str().c_str());
    }
    inline friend std::ostream& operator<<(std::ostream& o, const SNode& obj) {
      o << obj.str();
      return o;
    }
    inline uint64_t hash() const {
      static constexpr uint64_t p = 31;
      uint64_t res = 17;
      for (const auto& e : es) res = p * res + e.hash();
      return res;
    }
  };

  struct SLayout {
    std::vector<SNode> nodes;
  };

  struct SLayoutGenerator {
    static constexpr int V_MAX = 225;
    SProblemPtr prob;
    int V, E;
    Xorshift& rnd;
    std::vector<SNode> nodes;
    std::vector<SEdge> edges;
    SLayoutGenerator(SProblemPtr prob, Xorshift& rnd);
    std::vector<P> calc_forces() const;
    void force_directed_layout(Xorshift& rnd);
    void vis(int delay = 0) const;
    //uint64_t hash() const {
    //  static constexpr uint64_t p = 31;
    //  uint64_t res = 17;
    //  for (const auto& n : nodes) res = p * res + n.hash();
    //  return res;
    //}
    //std::string get_filename() const {
    //  std::ostringstream oss;
    //  oss << std::hex << hash() << ".graph";
    //  std::string filename = "graphvis/" + oss.str();
    //  return filename;
    //}
    //void load_or_create() {
    //  std::string filename = get_filename();
    //  if (std::filesystem::exists(filename)) {
    //    std::ifstream ifs(filename);
    //    int n;
    //    ifs >> n;
    //    for (int u = 0; u < n; u++) {
    //      ifs >> nodes[u].r.x >> nodes[u].r.y;
    //    }
    //    ifs.close();
    //  }
    //  else {
    //    first_annealing(rnd);
    //    force_directed_layout(rnd);
    //    save();
    //  }
    //}
    //void save() const {
    //  std::string filename = get_filename();
    //  std::ofstream ofs(filename);
    //  ofs << nodes.size() << '\n';
    //  for (const auto& n : nodes) {
    //    ofs << n.r.x << ' ' << n.r.y << std::endl;
    //  }
    //  ofs.close();
    //}
    //std::vector<P> get_positions() const {
    //  std::vector<P> res;
    //  for (const Node& n : nodes) res.push_back(n.r);
    //  return res;
    //}
    //Layout get_layout() const {
    //  Layout layout;
    //  layout.nodes = nodes;
    //  return layout;
    //}
  };

}

// vim:ts=2 sw=2 sts=2 et ci

