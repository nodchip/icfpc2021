#include "stdafx.h"
#include <opencv2/core.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/highgui.hpp>
#include "layout_generator.h"

namespace NLayoutGenerator {

  SLayoutGenerator::SLayoutGenerator(SProblemPtr prob, Xorshift& rnd) : prob(prob), rnd(rnd) {
    V = prob->vertices.size();
    E = prob->edges.size();
    nodes.resize(V);
    for (int vid = 0; vid < V; vid++) {
      nodes[vid].id = vid;
      nodes[vid].r = P(prob->vertices[vid].first, prob->vertices[vid].second);
    }
    auto original_length = [&](int uid, int vid) {
      auto u = prob->vertices[uid], v = prob->vertices[vid];
      return sqrt((u.first - v.first) * (u.first - v.first) + (u.second - v.second) * (u.second - v.second));
    };
    for (const auto& [uid, vid] : prob->edges) {
      double orig_len = original_length(uid, vid);
      SEdge e(uid, vid, original_length(uid, vid));
      nodes[uid].es.emplace_back(uid, vid, orig_len);
      nodes[vid].es.emplace_back(vid, uid, orig_len);
      edges.emplace_back(e);
    }
  }
  std::vector<P> SLayoutGenerator::calc_forces() const {
    static constexpr double spring_const = 1.0;
    static constexpr double gravity_const = 1.0;
    static constexpr double edge_const = 20.0; // 辺の中点から斥力が生じる
    static P fs[V_MAX][V_MAX];
    for (int i = 0; i < V; i++) {
      memset(fs[i], 0, sizeof(P) * V);
    }
#ifdef _MSC_VER
    concurrency::parallel_for_each(nodes.begin(), nodes.end(), [&](const SNode& n1) {
#else
    for (const SNode& n1 : nodes) {
#endif
      int u = n1.id;
      // spring
      for (const SEdge& e : n1.es) {
        const SNode& n2 = nodes[e.to];
        double d = (n1.r - n2.r).length();
        if (d < eps) continue; // TODO: random
        P vec = n2.r - n1.r;
        fs[u][u] += spring_const * (d - e.orig_len) * vec.unit();
      }
      // gravity
      for (const SNode& n2 : nodes) {
        if (u == n2.id) continue;
        double d = (n1.r - n2.r).length();
        if (d < eps) continue;
        double dinv = std::min(100.0, 1.0 / (d * d));
        P vec = n2.r - n1.r;
        fs[u][u] -= gravity_const * dinv * vec.unit();
      }
      // edge
      for (const SEdge& e : edges) {
        if (u == e.from || u == e.to) continue;
        P ps = nodes[e.from].r, pt = nodes[e.to].r, p = n1.r;
        P pm = (ps + pt) / 2.0;
        double d = (p - pm).length();
        double dinv = std::min(100.0, 1.0 / (d * d));
        P proj = projection(ps, pt, p);
        if ((p - proj).length2() < eps) continue;
        P vec = (p - proj).unit();
        P f = edge_const * dinv * vec;
        fs[u][u] += f;
        fs[e.from][u] -= f / 2.0;
        fs[e.to][u] -= f / 2.0;
      }
#ifdef _MSC_VER
    });
#else
    }
#endif
    std::vector<P> ret(V);
    for (int i = 0; i < V; i++) {
      ret[i] = std::accumulate(fs[i], fs[i] + V, P());
    }
    return ret;
  }
  void SLayoutGenerator::force_directed_layout(Xorshift& rnd) {
    static constexpr double decay_const = 0.999;
    static constexpr double dt = 0.01;
    static constexpr double randomness = 1.0;
    // 同一点・同一直線上に来ないようにバラす
    for (int u = 0; u < V; u++) {
      nodes[u].r += P(
        rnd.next_double() * randomness - randomness * 0.5,
        rnd.next_double() * randomness - randomness * 0.5
      );
    }
    int loop = 0;
    while (true) {
      double energy = 0.0;
      std::vector<P> fs = calc_forces();
      loop++;
      // update
      for (SNode& n : nodes) {
        n.v = (n.v + dt * fs[n.id]) * decay_const;
        n.r += n.v * dt;
        energy += n.v.length2();
      }
      if (loop % 100 == 0) {
        LOG(INFO) << "loop = " << loop << ", energy = " << energy;
        vis(1);
      }
      if (energy < 1e-3) break;
    }
  }
  void SLayoutGenerator::vis(int delay) const {
    double x_min = DBL_MAX, x_max = DBL_MIN;
    double y_min = DBL_MAX, y_max = DBL_MIN;
    for (const auto& n : nodes) {
      x_min = std::min(x_min, n.r.x);
      x_max = std::max(x_max, n.r.x);
      y_min = std::min(y_min, n.r.y);
      y_max = std::max(y_max, n.r.y);
    }
    double margin = 10.0;
    double x_offset = -x_min + margin;
    double y_offset = -y_min + margin;
    double width = (x_max - x_min) + 2.0 * margin;
    double height = (y_max - y_min) + 2.0 * margin;
    int pic_size = 1200;
    double scale = pic_size / std::max(width, height);
    cv::Mat_<cv::Vec3b> img(int(scale * height), int(scale * width), cv::Vec3b(255, 255, 255));
    // edges
    for (int u = 0; u < V; u++) {
      const auto& nu = nodes[u];
      double y1 = (nu.r.y + y_offset) * scale;
      double x1 = (nu.r.x + x_offset) * scale;
      for (const auto& e : nu.es) {
        if (u > e.to) continue;
        const auto& nv = nodes[e.to];
        double y2 = (nv.r.y + y_offset) * scale;
        double x2 = (nv.r.x + x_offset) * scale;
        cv::line(img, cv::Point((int)round(x1), (int)round(y1)), cv::Point((int)round(x2), (int)round(y2)), cv::Scalar(0, 0, 0), 1, cv::LINE_AA);
      }
    }
    // nodes
    for (int u = 0; u < V; u++) {
      const auto& nu = nodes[u];
      double y = (nu.r.y + y_offset) * scale;
      double x = (nu.r.x + x_offset) * scale;
      cv::circle(img, cv::Point((int)round(x), (int)round(y)), 10, cv::Scalar(255, 255, 255), cv::FILLED, cv::LINE_AA);
      cv::circle(img, cv::Point((int)round(x), (int)round(y)), 10, cv::Scalar(0, 0, 0), 1, cv::LINE_AA);
      cv::putText(img, std::to_string(u), cv::Point((int)round(x - 7), (int)round(y + 2)), cv::FONT_HERSHEY_SIMPLEX, 0.25, cv::Scalar(0, 0, 0), 1, cv::LINE_AA);
    }
    cv::resize(img, img, cv::Size(pic_size, pic_size), 0.0, 0.0, cv::INTER_CUBIC);
    cv::imshow("img", img);
    cv::waitKey(delay);
  }
}

// vim:ts=2 sw=2 sts=2 et ci

