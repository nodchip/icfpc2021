#include "stdafx.h"
#include "layout_editor.h"
#include "visual_editor.h"

namespace NLayoutEditor {

  SLayout::SLayout(SProblemPtr problem, int seed) : problem(problem) {
    rnd.set_seed(seed + 10007);
    init();
    draw_base_image();
    update(nullptr);
  }
  std::vector<P> SLayout::calc_forces() const {
    static constexpr double spring_const = 1.0;
    static constexpr double coulomb_const = 100.0;
    static constexpr double edge_const = 1.0; // 辺の中点から斥力が生じる
    static P fs[max_num_vertices][max_num_vertices];
    for (int i = 0; i < num_nodes; i++) {
      memset(fs[i], 0, sizeof(P) * num_nodes);
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
      // 
      for (const SNode& n2 : nodes) {
        if (u == n2.id) continue;
        double d = (n1.r - n2.r).length();
        if (d < eps) continue;
        double dinv = std::min(100.0, 1.0 / (d * d));
        P vec = n2.r - n1.r;
        fs[u][u] -= coulomb_const * dinv * vec.unit();
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
    std::vector<P> ret(num_nodes);
    for (int i = 0; i < num_nodes; i++) {
      ret[i] = std::accumulate(fs[i], fs[i] + num_nodes, P());
    }
    return ret;
  }

  void SLayout::init() {
    // node & edge
    num_nodes = problem->vertices.size();
    num_edges = problem->edges.size();
    nodes.resize(num_nodes);
    for (int vid = 0; vid < num_nodes; vid++) {
      nodes[vid].id = vid;
      nodes[vid].r = P(problem->vertices[vid].first, problem->vertices[vid].second);
    }
    auto original_length = [&](int uid, int vid) {
      auto u = problem->vertices[uid], v = problem->vertices[vid];
      return sqrt((u.first - v.first) * (u.first - v.first) + (u.second - v.second) * (u.second - v.second));
    };
    for (const auto& [uid, vid] : problem->edges) {
      double orig_len = original_length(uid, vid);
      SEdge e(uid, vid, original_length(uid, vid));
      nodes[uid].es.emplace_back(uid, vid, orig_len);
      nodes[vid].es.emplace_back(vid, uid, orig_len);
      edges.emplace_back(e);
    }
    // img
    roi_poly = SROI(problem->hole_polygon);
    roi_fig = SROI(problem->vertices);
    double x_min = std::min(roi_poly.x_min, roi_fig.x_min);
    double x_max = std::max(roi_poly.x_max, roi_fig.x_max);
    double y_min = std::min(roi_poly.y_min, roi_fig.y_min);
    double y_max = std::max(roi_poly.y_max, roi_fig.y_max);
    img_width = (int)round(x_max + kBaseOffset * 2);
    img_height = (int)round(y_max + kBaseOffset * 2);
    integer mag_x = kImageWidthPx / img_width;
    integer mag_y = kImageHeightPx / img_height;
    mag = std::min(mag_x, mag_y);
  }

  void SLayout::draw_base_image() {
    int img_width_px = img_width * mag;
    int img_height_px = img_height * mag + kInfoBufferHeightPx;
    base_img = cv::Mat_<cv::Vec3b>(img_height_px, img_width_px, cv::Vec3b(160, 160, 160));
    {
      std::vector<std::vector<cv::Point>> cv_hole_polygon(1);
      auto& polygon = cv_hole_polygon[0];
      for (auto p : problem->hole_polygon) {
        auto [x, y] = cvt(p);
        polygon.emplace_back((int)round(x), (int)round(y));
      }
      cv::fillPoly(base_img, cv_hole_polygon, cv::Scalar(255, 255, 255));
    }
  }

  void SLayout::update(SEditorParamsPtr ep) {
    img = base_img.clone();
    for (const auto& edge : edges) {
      auto [x1, y1] = cvt(nodes[edge.from].r);
      auto [x2, y2] = cvt(nodes[edge.to].r);
      draw_line(img, (int)round(x1), (int)round(y1), (int)round(x2), (int)round(y2), cv::Scalar(0, 0, 0), 2);
    }
    for (const auto& node : nodes) {
      auto [x, y] = cvt(node.r);
      draw_circle(img, (int)round(x), (int)round(y), 3, cv::Scalar(255, 255, 255), cv::FILLED);
      draw_circle(img, (int)round(x), (int)round(y), 3, cv::Scalar(0, 0, 0), 1);
    }
    if (ep && ep->selected_node_id != -1) {
      auto [x, y] = cvt(nodes[ep->selected_node_id].r);
      int ix = (int)round(x), iy = (int)round(y);
      draw_circle(img, x, y, 5, cv::Scalar(0, 0, 255), cv::FILLED);
    }
    else if (ep && ep->nearest_node_id != -1) {
      auto [x, y] = cvt(nodes[ep->nearest_node_id].r);
      int ix = (int)round(x), iy = (int)round(y);
      draw_circle(img, x, y, 5, cv::Scalar(255, 255, 255), cv::FILLED);
      draw_circle(img, x, y, 5, cv::Scalar(0, 0, 255), 2);
    }
  }
}

// vim:ts=2 sw=2 sts=2 et ci

