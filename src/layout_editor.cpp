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
    // TODO: should be variable
    static constexpr double spring_const = 1.0;
    static constexpr double coulomb_const = 10.0;
    static constexpr double edge_const = 10.0; // 辺の中点から斥力が生じる
    static constexpr double field_const = 1.0;
    static constexpr double edge_field_const = 1.0;
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
        if (d < eps) continue;
        P vec = n2.r - n1.r;
        fs[u][u] += spring_const * (d - e.orig_len) * vec.unit();
      }
      // coulomb
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
      // field
      {
        auto p = n1.r;
        auto xy = cvt(p.x, p.y);
        int ix = (int)round(xy.x); ix = std::clamp(ix, 0, img_dist.rows - 1);
        int iy = (int)round(xy.y); iy = std::clamp(iy, 0, img_dist.cols - 1);
        fs[u][u] -= field_const * vec_field[iy][ix];
      }
#ifdef _MSC_VER
    });
#else
    }
#endif
    for (const auto& edge : edges) {
      int u = edge.from, v = edge.to;
      double len = edge.orig_len; // orig ?
      auto pu = nodes[edge.from].r, pv = nodes[edge.to].r, pm = (pu + pv) * 0.5;
      auto xy = cvt(pm.x, pm.y);
      int ix = (int)round(xy.x); ix = std::clamp(ix, 0, img_dist.rows - 1);
      int iy = (int)round(xy.y); iy = std::clamp(iy, 0, img_dist.cols - 1);
      auto vec = vec_field[iy][ix];
      fs[u][u] -= edge_field_const * len * vec;
      fs[v][v] -= edge_field_const * len * vec;
    }
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
    edge_colors.resize(num_edges);
    for (int vid = 0; vid < num_nodes; vid++) {
      nodes[vid].id = vid;
      nodes[vid].r = P(problem->vertices[vid].first, problem->vertices[vid].second);
    }
    auto original_length = [&](int uid, int vid) {
      auto u = problem->vertices[uid], v = problem->vertices[vid];
      return sqrt((u.first - v.first) * (u.first - v.first) + (u.second - v.second) * (u.second - v.second));
    };
    for (int eid = 0; eid < num_edges; eid++) {
      const auto& [uid, vid] = problem->edges[eid];
      double orig_len = original_length(uid, vid);
      SEdge e(eid, uid, vid, orig_len);
      nodes[uid].es.emplace_back(-1, uid, vid, orig_len);
      nodes[vid].es.emplace_back(-1, vid, uid, orig_len);
      edges.push_back(e);
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
    cv::Vec3b outer_color(160, 160, 160), inner_color(255, 255, 255);
    base_img = cv::Mat_<cv::Vec3b>(img_height_px, img_width_px, outer_color);
    {
      std::vector<std::vector<cv::Point>> cv_hole_polygon(1);
      auto& polygon = cv_hole_polygon[0];
      for (auto p : problem->hole_polygon) {
        auto [x, y] = cvt(p);
        polygon.emplace_back((int)round(x), (int)round(y));
      }
      cv::fillPoly(base_img, cv_hole_polygon, inner_color);
    }
    cv::Mat gray;
    cv::cvtColor(base_img, gray, cv::COLOR_BGR2GRAY);
    cv::Mat binary;
    cv::threshold(gray, binary, 0, 255, cv::THRESH_BINARY_INV | cv::THRESH_OTSU);
    cv::distanceTransform(binary, img_dist, cv::DIST_L2, 3);
    cv::Mat img_blur;
    cv::GaussianBlur(img_dist, img_blur, cv::Size(5, 5), 10.0);
    cv::Mat img_dx, img_dy;
    cv::Sobel(img_blur, img_dx, CV_32F, 1, 0);
    cv::Sobel(img_blur, img_dy, CV_32F, 0, 1);
    vec_field.resize(img_dist.rows, std::vector<P>(img_dist.cols));
    for (int i = 0; i < img_dist.rows; i++) {
      for (int j = 0; j < img_dist.cols; j++) {
        float gx = img_dx.at<float>(i, j), gy = img_dy.at<float>(i, j);
        double grad = sqrt(gx * gx + gy * gy);
        vec_field[i][j] = P(gx, gy);
      }
    }
    //cv::imshow("dist", img_dist / 40.0);
    //cv::waitKey(0);
    //cv::Mat_<cv::Vec3b> img_test(img_dist.rows, img_dist.cols, cv::Vec3b(255, 255, 255));
    //for (int i = 10; i < img_dist.rows; i += 10) {
    //  for (int j = 10; j < img_dist.cols; j += 10) {
    //    auto f = vec_field[i][j] * 2.0;
    //    int ni = i + (int)round(f.y), nj = j + (int)round(f.x);
    //    cv::arrowedLine(img_test, cv::Point(j, i), cv::Point(nj, ni), cv::Scalar(0, 0, 0), 1, 8, 0.2);
    //  }
    //}
    //cv::imshow("arrow", img_test);
    //cv::waitKey(0);
  }

  void SLayout::update(SEditorParamsPtr ep) {
    img = base_img.clone();
    for (int eid = 0; eid < num_edges; eid++) {
      edge_colors[eid] = get_edge_color(eid);
    }
    for (const auto& edge : edges) {
      auto [x1, y1] = cvt(nodes[edge.from].r);
      auto [x2, y2] = cvt(nodes[edge.to].r);
      draw_line(img, (int)round(x1), (int)round(y1), (int)round(x2), (int)round(y2), edge_colors[edge.id], 2);
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

  cv::Scalar SLayout::get_edge_color(integer iedge) const {
    constexpr integer denominator = 1000000;
    const auto& edge = edges[iedge];
    auto org_i = problem->vertices[edge.from];
    auto org_j = problem->vertices[edge.to];
    auto moved_i = nodes[edge.from].r;
    auto moved_j = nodes[edge.to].r;
    auto d2_moved = (moved_i - moved_j).length2();
    double d2_ori = 0.0;
    {
      double dx = org_i.first - org_j.first, dy = org_i.second - org_j.second;
      d2_ori = dx * dx + dy * dy;
    }
    // |d(moved) / d(original) - 1| <= eps / 1000000
    integer diff = denominator * d2_moved - denominator * d2_ori;
    integer thresh = problem->epsilon * d2_ori;
    if (diff < -thresh)
      return cv::Scalar(255, 0, 0);
    if (diff > thresh)
      return cv::Scalar(0, 0, 255);
    return cv::Scalar(0, 255, 0);
  }

  SLayoutEditor::SLayoutEditor(SProblemPtr problem, const std::string& solver_name, const std::string window_name, int seed)
    : window_name(window_name), solver_name(solver_name) {
    layout = std::make_shared<SLayout>(problem, seed);
    mp = std::make_shared<SMouseParams>();
    ep = std::make_shared<SEditorParams>();
    cv::namedWindow(window_name, cv::WINDOW_AUTOSIZE);
    cv::setMouseCallback(window_name, callback, this);
  }

  int SLayoutEditor::get_nearest_node_id() const {
    int x = mp->x, y = mp->y;
    int idx = -1;
    double min_d2 = std::numeric_limits<double>::max();
    for (int i = 0; i < layout->nodes.size(); i++) {
      const auto& node = layout->nodes[i];
      double d2 = (layout->cvt(node.r) - P(x, y)).length2();
      if (d2 < min_d2) {
        min_d2 = d2;
        idx = i;
      }
    }
    return idx;
  }
  void SLayoutEditor::force_directed_layout(bool clipping) {
    static constexpr double decay_const = 0.99;
    static constexpr double dt = 0.01;
    static constexpr double randomness = 0.1;
    // 同一点・同一直線上に来ないようにバラす
    for (int u = 0; u < layout->num_nodes; u++) {
      layout->nodes[u].r += P(
        layout->rnd.next_double() * randomness - randomness * 0.5,
        layout->rnd.next_double() * randomness - randomness * 0.5
      );
    }
    int loop = 0;
    while (true) {
      double energy = 0.0;
      std::vector<P> fs = layout->calc_forces();
      loop++;
      // update
      for (SNode& n : layout->nodes) {
        n.v = (n.v + dt * fs[n.id]) * decay_const;
        n.r += n.v * dt;
        if (clipping) layout->roi_poly.clip(n.r);
        energy += n.v.length2();
      }
      if (ep->selected_node_id != -1) {
        auto& p = layout->nodes[ep->selected_node_id].r;
        p = layout->icvt(mp->x, mp->y);
      }
      if (loop % 100 == 0) {
        layout->update(ep);
        cv::imshow(window_name, layout->img);
        int key = cv::waitKey(15);
        if (key == 27) break;
      }
    }
  }

  void SLayoutEditor::callback(int e, int x, int y, int f, void* param) {
    SLayoutEditor* s = static_cast<SLayoutEditor*>(param);
    auto mp = s->mp;
    auto ep = s->ep;
    mp->load(e, x, y, f);
    ep->nearest_node_id = s->get_nearest_node_id();
    if (mp->clicked_left()) {
      if (ep->nearest_node_id != -1) {
        ep->selected_node_id = ep->nearest_node_id;
      }
    }
    if (mp->released_left()) {
      ep->selected_node_id = -1;
    }
    s->layout->update(ep);
  }
}

// vim:ts=2 sw=2 sts=2 et ci

