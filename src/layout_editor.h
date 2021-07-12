#pragma once
#include <sstream>
#include <fmt/format.h>
#include "util.h"
#include "contest_types.h"
#include <opencv2/core.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/highgui.hpp>

#ifdef _MSC_VER
#include <ppl.h>
#endif

namespace NLayoutEditor {

  constexpr double eps = 1e-8;

  inline void draw_circle(cv::Mat& img,
    int x,
    int y,
    int sz,
    cv::Scalar col,
    int thickness) {
    cv::circle(img, cv::Point(x, y), sz, col, thickness);
  }

  inline void draw_line(cv::Mat& img,
    int x1,
    int y1,
    int x2,
    int y2,
    cv::Scalar col,
    int thickness) {
    cv::line(img, cv::Point(x1, y1), cv::Point(x2, y2), col, thickness);
  }

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

  struct SMouseParams {
    int pe, px, py, pf;
    int e, x, y, f;
    SMouseParams() { e = x = y = f = pe = px = py = pf = INT_MAX; };
    inline void load(int e_, int x_, int y_, int f_) {
      pe = e;
      px = x;
      py = y;
      pf = f;
      e = e_;
      x = x_;
      y = y_;
      f = f_;
    }
    inline bool clicked_left() const { return e == 1 && pe == 0; }
    inline bool clicked_right() const { return e == 2 && pe == 0; }
    inline bool released_left() const { return e == 4; }
    inline bool released_right() const { return e == 5; }
    inline bool drugging_left() const { return e == 0 && f == 1; }
    inline bool drugging_right() const { return e == 0 && f == 2; }
    inline std::pair<int, int> coord() const { return { x, y }; }
    inline std::pair<int, int> displacement() const {
      return { abs(x - px) > 10000 ? 0 : (x - px), abs(y - py) ? 0 : (y - py) };
    }
    inline std::string str() const {
      return fmt::format(
        "SMouseParams [(e,x,y,f)=({},{},{},{}), (pe,px,py,pf)=({},{},{},{})", e,
        x, y, f, pe, px, py, pf);
    }
    inline friend std::ostream& operator<<(std::ostream& o, const SMouseParams& obj) {
      o << obj.str();
      return o;
    }
    inline friend std::ostream& operator<<(std::ostream& o, const std::shared_ptr<SMouseParams>& obj) {
      o << obj->str();
      return o;
    }
  };
  using SMouseParamsPtr = std::shared_ptr<SMouseParams>;

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
    int id;
    int from, to;
    double orig_len;
    SEdge() {}
    SEdge(int id, int u, int v, double d) : id(id), from(u), to(v), orig_len(d) {}
    inline std::string str() const { return fmt::format("SEdge [id={}, from={}, to={}, orig_len={}]", id, from, to, orig_len); }
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

  struct SROI {
    double x_min, y_min, x_max, y_max;
    SROI(double x_min = 0.0, double y_min = 0.0, double x_max = 0.0, double y_max = 0.0)
      : x_min(x_min), y_min(y_min), x_max(x_max), y_max(y_max) {}
    SROI(const std::vector<Point>& points) {
      x_min = y_min = std::numeric_limits<double>::max();
      x_max = y_max = std::numeric_limits<double>::lowest();
      for (const auto& [x, y] : points) {
        x_min = std::min(x_min, double(x));
        y_min = std::min(y_min, double(y));
        x_max = std::max(x_max, double(x));
        y_max = std::max(y_max, double(y));
      }
    }
    inline void clip(P& p) const {
      p.x = std::clamp(p.x, x_min, x_max);
      p.y = std::clamp(p.y, y_min, y_max);
    }
    inline std::string str() const {
      return fmt::format("SROI [({}, {}) - ({}, {})]", x_min, y_min, x_max, y_max);
    }
    inline friend std::ostream& operator<<(std::ostream& o, const SROI& obj) {
      o << obj.str();
      return o;
    }
  };

  struct SEditorParams {
    int nearest_node_id = -1;
    int selected_node_id = -1;

    int spring_const = 10;
    int coulomb_const = 100;
    int edge_const = 100;
    int field_const = 10;
    int edge_field_const = 10;
  };
  using SEditorParamsPtr = std::shared_ptr<SEditorParams>;

  struct SLayout {
  private:
    static constexpr int kBaseOffset = 5;
    static constexpr int kImageWidthPx = 1000;
    static constexpr int kImageHeightPx = 1000;
    static constexpr int kInfoBufferHeightPx = 0;
    static constexpr int max_num_vertices = 225;
  public:
    Xorshift rnd;
    SProblemPtr problem;
    SEditorParamsPtr ep;
    int num_nodes;
    std::vector<SNode> nodes;
    int num_edges;
    std::vector<SEdge> edges;
    std::vector<cv::Scalar> edge_colors;

    SROI roi_poly, roi_fig;
    int img_width;
    int img_height;
    int mag;
    int offset_x = kBaseOffset;
    int offset_y = kBaseOffset;
    cv::Mat_<cv::Vec3b> base_img;
    cv::Mat_<cv::Vec3b> img;
    cv::Mat_<float> img_dist;
    std::vector<std::vector<P>> vec_field;
    
    inline P cvt(double x, double y) const {
      return P(
        (x + offset_x) * mag,
        (y + offset_y) * mag + kInfoBufferHeightPx
      );
    };
    inline P cvt(const P& p) { return cvt(p.x, p.y); }
    inline P cvt(const Point& p) { return cvt(p.first, p.second); }
    inline P icvt(int x, int y) { return { double(x) / mag - offset_x, double(y - kInfoBufferHeightPx) / mag - offset_y }; }
    SLayout(SProblemPtr problem, SEditorParamsPtr ep, int seed = 0);
    std::vector<P> calc_forces() const;
    void vis(int delay = 0) const;
    void init();
    void draw_base_image();
    void update();
    cv::Scalar get_edge_color(integer iedge) const;
  };
  using SLayoutPtr = std::shared_ptr<SLayout>;

  struct SLayoutEditor {
    SLayoutPtr layout;

    const std::string window_name;
    const std::string solver_name;
    SMouseParamsPtr mp;
    SEditorParamsPtr ep;

    SLayoutEditor(SProblemPtr problem, const std::string& solver_name, const std::string window_name, int seed = 0);
    int get_nearest_node_id() const;
    void force_directed_layout(bool clipping = true);
    static void mouse_callback(int e, int x, int y, int f, void* param);
    static void spring_callback(int val, void* param);
    static void coulomb_callback(int val, void* param);
    static void edge_callback(int val, void* param);
    static void field_callback(int val, void* param);
    static void edge_field_callback(int val, void* param);
  };

}

// vim:ts=2 sw=2 sts=2 et ci

