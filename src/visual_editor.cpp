#include "stdafx.h"

#include "visual_editor.h"

#include <fmt/format.h>
#include <opencv2/core.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>
#include <set>

#include "judge.h"
#include "solver_registry.h"
#include "util.h"

namespace {

cv::Rect calc_bb(const std::vector<Point>& points) {
  integer x_min = INT64_MAX, x_max = INT64_MIN;
  integer y_min = INT64_MAX, y_max = INT64_MIN;
  for (const auto [x, y] : points) {
    x_min = std::min(x, x_min);
    x_max = std::max(x, x_max);
    y_min = std::min(y, y_min);
    y_max = std::max(y, y_max);
  }
  return cv::Rect(x_min, y_min, x_max - x_min, y_max - y_min);
}

}  // namespace

struct SCanvas {
 private:
  static constexpr int kBaseOffset = 5;
  static constexpr int kImageWidthPx = 1000;
  static constexpr int kImageHeightPx = 1000;
  static constexpr int kInfoBufferHeightPx = 100;
  static constexpr integer kMaxMag = 20;
  static constexpr integer kMinMag = 1;

 public:
  SProblemPtr problem;
  SSolutionPtr solution;
  std::set<int> marked_vertex_indices;

  integer img_width;
  integer img_height;
  integer mag;

  integer offset_x{kBaseOffset};
  integer offset_y{kBaseOffset};
  cv::Mat_<cv::Vec3b> base_img;
  cv::Mat_<cv::Vec3b> img;

  integer dislikes;
  bool fit_in_hole;
  bool satisfy_stretch;
  int num_no_fit_in_hole_vert = 0;
  int num_no_fit_in_hole_edge = 0;
  int num_no_satisfy_stretch = 0;
  int num_gained_bonuses = 0;
  bool is_valid = false;
  std::string oneshot_custom_stat;
  std::string persistent_custom_stat;

  bool draw_distant_hole_vertex = true;
  bool draw_tolerated_vertex = true;
  bool draw_edge_lengths_mode = true;
  bool draw_index_mode = true;

  std::vector<cv::Scalar> edge_colors;

  cv::Scalar violating_vertex_color = cv::Scalar(128, 0, 128);
  cv::Scalar out_of_hole_edge_color = cv::Scalar(128, 0, 128);
  cv::Scalar marked_vartex_color = cv::Scalar(128, 128, 0);

  inline cv::Point cvt(int x, int y) {
    return cv::Point((x + offset_x) * mag,
                     (y + offset_y) * mag + kInfoBufferHeightPx);
  };
  inline cv::Point cvt(const Point& p) { return cvt(p.first, p.second); }
  inline Point icvt(int x, int y) {
    return {x / mag - offset_x, (y - kInfoBufferHeightPx) / mag - offset_y};
  }

  void draw_circle(cv::Mat& img,
                   int x,
                   int y,
                   int sz,
                   cv::Scalar col,
                   int thickness) {
    cv::circle(img, cv::Point(x, y), sz, col, thickness);
  }

  void draw_line(cv::Mat& img,
                 int x1,
                 int y1,
                 int x2,
                 int y2,
                 cv::Scalar col,
                 int thickness) {
    cv::line(img, cv::Point(x1, y1), cv::Point(x2, y2), col, thickness);
  }

  cv::Scalar get_edge_color(integer iedge) const {
    constexpr integer denominator = 1000000;
    const auto& edge = problem->edges[iedge];
    auto org_i = problem->vertices[edge.first];
    auto org_j = problem->vertices[edge.second];
    auto moved_i = solution->vertices[edge.first];
    auto moved_j = solution->vertices[edge.second];
    auto d2_moved = distance2(moved_i, moved_j);
    auto d2_ori = distance2(org_i, org_j);
    // |d(moved) / d(original) - 1| <= eps / 1000000
    integer diff = denominator * d2_moved - denominator * d2_ori;
    integer thresh = problem->epsilon * d2_ori;
    if (diff < -thresh)
      return cv::Scalar(255, 0, 0);
    if (diff > thresh)
      return cv::Scalar(0, 0, 255);
    return cv::Scalar(0, 255, 0);
  }

  void set_oneshot_custom_stat(const std::string& stat_str) {
    oneshot_custom_stat = stat_str;
  }
  void set_persistent_custom_stat(const std::string& stat_str) {
    persistent_custom_stat = stat_str;
  }

  void draw_stats(cv::Mat& img) {
    std::ostringstream oss_bonus;
    for (auto& b : problem->available_bonuses) {
      oss_bonus << SBonus::bonus_name(b.type);
      oss_bonus << " ";
    }
    std::ostringstream oss_using_bonus;
    for (auto& b : solution->bonuses) {
      oss_using_bonus << SBonus::bonus_name(b.type);
      oss_using_bonus << " ";
    }
    std::string stat_str = fmt::format(
        "[{}] DL={}, fit={}(NG edge {} vert {}), stretch={}(NG {}), "
        "B[offerred={}, using={}, gained={}], mark {}",
        is_valid ? "O" : "X", dislikes, fit_in_hole, num_no_fit_in_hole_edge,
        num_no_fit_in_hole_vert, satisfy_stretch, num_no_satisfy_stretch,
        oss_bonus.str(), oss_using_bonus.str(), num_gained_bonuses,
        marked_vertex_indices.size());
    int y = 30;
    cv::putText(img, stat_str, cv::Point(20, y), cv::FONT_HERSHEY_SIMPLEX, 0.5,
                is_valid ? cv::Scalar(0, 0, 0) : cv::Scalar(0, 0, 128), 1,
                cv::LINE_AA);
    y += 30;
    if (!persistent_custom_stat.empty()) {
      cv::putText(img, persistent_custom_stat, cv::Point(20, y),
                  cv::FONT_HERSHEY_SIMPLEX, 0.5,
                  is_valid ? cv::Scalar(0, 0, 0) : cv::Scalar(0, 0, 128), 1,
                  cv::LINE_AA);
    }
    y += 30;
    if (!oneshot_custom_stat.empty()) {
      cv::putText(img, oneshot_custom_stat, cv::Point(20, y),
                  cv::FONT_HERSHEY_SIMPLEX, 0.5,
                  is_valid ? cv::Scalar(0, 0, 0) : cv::Scalar(0, 0, 128), 1,
                  cv::LINE_AA);
    }
  }

  void draw_edge_lengths(cv::Mat& img) {
    int nh = problem->hole_polygon.size();
    for (int i = 0; i < nh; i++) {
      Point raw_u = problem->hole_polygon[i];
      Point raw_v = problem->hole_polygon[(i + 1) % nh];
      cv::Point u = cvt(raw_u), v = cvt(raw_v);
      integer d2 = distance2(raw_u, raw_v);
      cv::putText(img, std::to_string(d2), (u + v) / 2,
                  cv::FONT_HERSHEY_SIMPLEX, 0.35, cv::Scalar(0, 0, 0), 1,
                  cv::LINE_AA);
    }
    for (int eid = 0; eid < problem->edges.size(); eid++) {
      auto [iu, iv] = problem->edges[eid];
      Point raw_u = solution->vertices[iu];
      Point raw_v = solution->vertices[iv];
      Point orig_raw_u = problem->vertices[iu];
      Point orig_raw_v = problem->vertices[iv];
      cv::Point u = cvt(raw_u), v = cvt(raw_v);
      integer d2 = distance2(raw_u, raw_v);
      integer orig_d2 = distance2(orig_raw_u, orig_raw_v);
      auto col = get_edge_color(eid);
      cv::putText(img, std::to_string(d2) + "/" + std::to_string(orig_d2),
                  (u + v) / 2, cv::FONT_HERSHEY_SIMPLEX, 0.35, col / 255 * 150,
                  1, cv::LINE_AA);
    }
  }

  void draw_index(cv::Mat& img) {
    int nv = solution->vertices.size();
    for (int i = 0; i < nv; i++) {
      Point raw_u = solution->vertices[i];
      cv::Point u = cvt(raw_u);
      cv::putText(img, std::to_string(i), u, cv::FONT_HERSHEY_SIMPLEX, 0.4,
                  cv::Scalar(255, 0, 0), 1, cv::LINE_AA);
    }
  }

  void shift_pose(int dx, int dy) {
    for (auto& p : solution->vertices) {
      p.first += dx;
      p.second += dy;
    }
  }

  void shift_all(int dx, int dy) {
    offset_x += dx;
    offset_y += dy;
  }

  void rotate_pose() {
    auto rect_poly = calc_bb(problem->hole_polygon);
    const int cx = rect_poly.x + rect_poly.width / 2;
    const int cy = rect_poly.y + rect_poly.height / 2;
    for (auto& p : solution->vertices) {
      int x = p.first - cx;
      int y = p.second - cy;
      p.first = -y + cx;
      p.second = x + cy;
    }
  }

  void zoom(int dmag) {
    mag += dmag;
    mag = std::max(std::min(mag, kMaxMag), kMinMag);

    img_width = kImageWidthPx / mag;
    img_height = kImageHeightPx / mag;
  }

  void update(int selected_id) {
    // judge
    auto res = judge(*problem, *solution);
    dislikes = res.dislikes;
    fit_in_hole = res.fit_in_hole_except_wallhack();
    satisfy_stretch = res.satisfy_stretch();
    num_no_fit_in_hole_vert = res.out_of_hole_vertices.size() -
                              (res.wallhacking_index.has_value() ? 1 : 0);
    num_no_fit_in_hole_edge = res.out_of_hole_edges_except_wallhack.size();
    num_no_satisfy_stretch = res.stretch_violating_edges.size();
    num_gained_bonuses = res.gained_bonus_indices.size();
    for (int eid = 0; eid < problem->edges.size(); eid++) {
      edge_colors[eid] = cv::Scalar(0, 255, 0);
    }
    for (int eid : res.stretch_violating_edges) {
      if (!(res.superflex_index && res.superflex_index == eid)) {
        edge_colors[eid] = get_edge_color(eid);
      }
    }
    is_valid = res.is_valid();
    // draw
    img = base_img.clone();
    if (draw_distant_hole_vertex) {
      for (int i = 0; i < problem->hole_polygon.size(); i++) {
        auto [x, y] = cvt(problem->hole_polygon[i]);
        auto r =
            2.0 * std::log(double(res.individual_dislikes[i]) + 1e-6) + 1.0;
        if (r > 0.0) {
          draw_circle(img, x, y, r, cv::Scalar(0, 0, 128), 1);
        }
      }
    }
    for (int eid : res.out_of_hole_edges_except_wallhack) {
      auto [u, v] = problem->edges[eid];
      auto [x1, y1] = cvt(solution->vertices[u]);
      auto [x2, y2] = cvt(solution->vertices[v]);
      draw_line(img, x1, y1, x2, y2, out_of_hole_edge_color,
                5);  // 3 is insufficient.
    }
    for (int eid = 0; eid < problem->edges.size(); eid++) {
      auto [u, v] = problem->edges[eid];
      auto [x1, y1] = cvt(solution->vertices[u]);
      auto [x2, y2] = cvt(solution->vertices[v]);
      draw_line(img, x1, y1, x2, y2, edge_colors[eid], 2);
    }
    if (res.wallhacking_index) {
      auto [x, y] = cvt(solution->vertices[*res.wallhacking_index]);
      draw_circle(img, x, y, std::max(12, int(mag)), cv::Scalar(32, 64, 128),
                  3);
    }
    if (selected_id != -1) {
      if (draw_tolerated_vertex) {
        auto bb = calc_bb(problem->hole_polygon);
        auto edges = edges_from_vertex(*problem, selected_id);
        std::map<Point, int> exact_counts;
        auto add_point = [&](int x, int y) {
          auto it = exact_counts.find({x, y});
          if (it == exact_counts.end()) {
            exact_counts.insert(it, {{x, y}, 1});
          } else {
            it->second += 1;
          }
        };
        for (auto eid : edges) {
          auto [u, v] = problem->edges[eid];
          const int counter_vid = u == selected_id ? v : u;
          const auto org_d2 = distance2(problem->vertices[selected_id],
                                        problem->vertices[counter_vid]);
          for (int y = bb.tl().y; y <= bb.br().y; ++y) {
            for (int x = bb.tl().x; x <= bb.br().x; ++x) {
              const auto moved_d2 =
                  distance2({x, y}, solution->vertices[counter_vid]);
              if (tolerate(org_d2, moved_d2, problem->epsilon)) {
                add_point(x, y);
              }
            }
          }
        }
        if (!exact_counts.empty()) {  // common feasible position.
          const int edge_count = edges.size();
          // others
          for (auto& [pos, count] : exact_counts) {
            auto [x, y] = cvt(pos);
            draw_circle(img, x, y, std::max(4, int(mag) / 4),
                        cv::Scalar(64, 0, 0), cv::FILLED);
          }
          // top 2
          if (edge_count - 1 > 1)
            for (auto& [pos, count] : exact_counts) {
              auto [x, y] = cvt(pos);
              if (count == edge_count - 1) {
                draw_circle(img, x, y, std::max(8, int(mag) / 2),
                            cv::Scalar(255, 64, 64), cv::FILLED);
              }
            }
          for (auto& [pos, count] : exact_counts) {
            auto [x, y] = cvt(pos);
            if (count == edge_count) {
              draw_circle(img, x, y, std::max(8, int(mag) / 2),
                          cv::Scalar(255, 255, 96), cv::FILLED);
            }
          }
        }
      }
      auto [x, y] = cvt(solution->vertices[selected_id]);
      draw_circle(img, x, y, std::max(3, int(mag) / 2), cv::Scalar(0, 0, 255),
                  cv::FILLED);
    }
    for (int vid : res.out_of_hole_vertices) {
      auto [x, y] = cvt(solution->vertices[vid]);
      draw_circle(img, x, y, std::max(2, int(mag) / 3), violating_vertex_color,
                  cv::FILLED);
    }
    for (int vid : marked_vertex_indices) {
      auto [x, y] = cvt(solution->vertices[vid]);
      draw_circle(img, x, y, std::max(12, int(mag) / 2), marked_vartex_color,
                  3);
    }
    draw_stats(img);
    if (draw_edge_lengths_mode) {
      draw_edge_lengths(img);
    }
    if (draw_index_mode) {
      draw_index(img);
    }
  }

  bool set_pose(SSolutionPtr pose) {
    CHECK(is_compatible(*problem, *pose));
    solution = pose;
    update(-1);
    return true;
  }

  void draw_base_image() {
    integer img_width_px = img_width * mag;
    integer img_height_px = img_height * mag + kInfoBufferHeightPx;
    base_img = cv::Mat_<cv::Vec3b>(img_height_px, img_width_px,
                                   cv::Vec3b(160, 160, 160));

    {
      // cv::fillPoly() throws with std::vector<cv::Point>
      std::vector<std::vector<cv::Point>> cv_hole_polygon(1);
      auto& polygon = cv_hole_polygon[0];
      for (auto p : problem->hole_polygon) {
        auto [x, y] = cvt(p);
        polygon.emplace_back(x, y);
      }
      cv::fillPoly(base_img, cv_hole_polygon, cv::Scalar(255, 255, 255));
    }

    for (auto& bonus : problem->bonuses) {
      auto [x, y] = bonus.position;
      cv::Scalar color;
      switch (bonus.type) {
        case SBonus::Type::GLOBALIST:
          color = cv::Scalar(32, 192, 192);
          break;
        case SBonus::Type::BREAK_A_LEG:
          color = cv::Scalar(192, 32, 32);
          break;
        case SBonus::Type::WALLHACK:
          color = cv::Scalar(64, 128, 255);
          break;
        case SBonus::Type::SUPERFLEX:
          color = cv::Scalar(192, 192, 32);
          break;
        case SBonus::Type::INVALID:
          CHECK(false);
      }
      cv::circle(base_img, cvt(x, y), 20, color, cv::FILLED);
    }

    // Draw grid points.
    for (integer x = -offset_x; x <= -offset_x + img_width; ++x) {
      for (integer y = -offset_y; y <= -offset_y + img_height; ++y) {
        cv::circle(base_img, cvt(x, y), 2, cv::Scalar(200, 200, 200),
                   cv::FILLED);
      }
    }

    const int n = problem->hole_polygon.size();
    for (int i = 0; i < n; ++i) {
      auto [x1, y1] = cvt(problem->hole_polygon[i]);
      auto [x2, y2] = cvt(problem->hole_polygon[(i + 1) % n]);
      draw_line(base_img, x1, y1, x2, y2, cv::Scalar(0, 0, 0), 2);
    }
  }

  void init() {
    auto rect_poly = calc_bb(problem->hole_polygon);
    auto rect_fig = calc_bb(problem->vertices);
    integer x_min = std::min(rect_poly.x, rect_fig.x);
    integer x_max =
        std::max(rect_poly.x + rect_poly.width, rect_fig.x + rect_fig.width);
    integer y_min = std::min(rect_poly.y, rect_fig.y);
    integer y_max =
        std::max(rect_poly.y + rect_poly.height, rect_fig.y + rect_fig.height);

    img_width = x_max + kBaseOffset * 2;
    img_height = y_max + kBaseOffset * 2;
    integer mag_x = kImageWidthPx / img_width;
    integer mag_y = kImageHeightPx / img_height;
    mag = std::min(mag_x, mag_y);
    mag = std::max(std::min(mag, kMaxMag), kMinMag);
    edge_colors.resize(problem->edges.size(), cv::Scalar(0, 255, 0));
  }

  SCanvas(SProblemPtr problem) : problem(problem) {
    init();
    draw_base_image();

    auto pose = problem->create_solution();
    pose->vertices = problem->vertices;
    set_pose(pose);
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
  inline std::pair<int, int> coord() const { return {x, y}; }
  inline std::pair<int, int> displacement() const {
    return {abs(x - px) > 10000 ? 0 : (x - px), abs(y - py) ? 0 : (y - py)};
  }
  std::string str() const {
    return fmt::format(
        "SMouseParams [(e,x,y,f)=({},{},{},{}), (pe,px,py,pf)=({},{},{},{})", e,
        x, y, f, pe, px, py, pf);
  }
  friend std::ostream& operator<<(std::ostream& o, const SMouseParams& obj) {
    o << obj.str();
    return o;
  }
  friend std::ostream& operator<<(std::ostream& o,
                                  const std::shared_ptr<SMouseParams>& obj) {
    o << obj->str();
    return o;
  }
};

SVisualEditor::SVisualEditor(SProblemPtr problem,
                             const std::string& solver_name,
                             const std::string window_name)
    : window_name(window_name), solver_name(solver_name) {
  canvas = std::make_shared<SCanvas>(problem);
  mp = std::make_shared<SMouseParams>();
  cv::namedWindow(window_name, cv::WINDOW_AUTOSIZE);
  cv::setMouseCallback(window_name, callback, this);
  selected_vertex_id = -1;
}

SVisualEditor::~SVisualEditor() {}

void SVisualEditor::set_oneshot_custom_stat(const std::string& stat_str) {
  canvas->set_oneshot_custom_stat(stat_str);
}
void SVisualEditor::set_persistent_custom_stat(const std::string& stat_str) {
  canvas->set_persistent_custom_stat(stat_str);
}

void SVisualEditor::set_marked_indices(const std::vector<int>& marked_indices) {
  canvas->marked_vertex_indices.clear();
  std::copy(marked_indices.begin(), marked_indices.end(),
            std::inserter(canvas->marked_vertex_indices,
                          canvas->marked_vertex_indices.end()));
  canvas->update(-1);
}

std::vector<int> SVisualEditor::get_marked_indices() const {
  std::vector<int> idx;
  std::copy(canvas->marked_vertex_indices.begin(),
            canvas->marked_vertex_indices.end(), std::back_inserter(idx));
  return idx;
}

bool SVisualEditor::set_pose(SSolutionPtr pose) {
  canvas->set_pose(pose);
  return false;
}

SSolutionPtr SVisualEditor::get_pose() const {
  return canvas->solution;
}

void SVisualEditor::save_intermediate() const {
  const std::string file_path = "intermediate.pose.json";
  std::ofstream ofs(file_path);
  auto json = canvas->solution->json();
  update_meta(json, solver_name);
  update_judge(*canvas->problem, judge(*canvas->problem, *canvas->solution),
               json);
  ofs << json;
  LOG(INFO) << "saved: " << file_path;
}

// Handles key inputs
SShowResult SVisualEditor::show(int delay_ms) {
  static constexpr int kEscKey = 27;
  static constexpr int kNoKeyPress = -1;
  static constexpr int kUpArrowKey = 82;
  static constexpr int kLeftArrowKey = 81;
  static constexpr int kDownArrowKey = 84;
  static constexpr int kRightArrowKey = 83;

  SShowResult res(cv::waitKey(delay_ms));
  if (res.key == kEscKey) {
    if (in_internal_edit_loop()) {
      // Leaving internal edit mode.
      res.key = 'm';
    } else {
      return res.key;
    }
  }

  switch (res.key) {
    case 'd':
      canvas->draw_distant_hole_vertex = !canvas->draw_distant_hole_vertex;
      canvas->update(-1);
      break;
    case 'e':
      canvas->draw_edge_lengths_mode = !canvas->draw_edge_lengths_mode;
      canvas->update(-1);
      break;
    case 'i':
      canvas->draw_index_mode = !canvas->draw_index_mode;
      canvas->update(-1);
      break;
    case 't':
      canvas->draw_tolerated_vertex = !canvas->draw_tolerated_vertex;
      canvas->update(get_mouseover_node_id());
      break;
    case 's':
      save_intermediate();
      break;
    case 'h':
      canvas->shift_pose(-1, 0);
      canvas->update(-1);
      break;
    case 'j':
      canvas->shift_pose(0, 1);
      canvas->update(-1);
      break;
    case 'k':
      canvas->shift_pose(0, -1);
      canvas->update(-1);
      break;
    case 'l':
      canvas->shift_pose(1, 0);
      canvas->update(-1);
      break;
    case 'r':
      canvas->rotate_pose();
      canvas->update(-1);
      break;
    case '/':
      canvas->zoom(1);
      canvas->draw_base_image();
      canvas->update(-1);
      break;
    case '\\':
      canvas->zoom(-1);
      canvas->draw_base_image();
      canvas->update(-1);
      break;
    case kUpArrowKey:
      canvas->shift_all(0, -1);
      canvas->draw_base_image();
      canvas->update(-1);
      break;
    case kDownArrowKey:
      canvas->shift_all(0, 1);
      canvas->draw_base_image();
      canvas->update(-1);
      break;
    case kLeftArrowKey:
      canvas->shift_all(-1, 0);
      canvas->draw_base_image();
      canvas->update(-1);
      break;
    case kRightArrowKey:
      canvas->shift_all(1, 0);
      canvas->draw_base_image();
      canvas->update(-1);
      break;
    case 'm': {  // toggle internal edit mode.
      LOG(INFO) << "in internal edit loop? " << in_internal_edit_loop();
      if (in_internal_edit_loop()) {
        LOG(INFO) << "leaving internal edit loop.";
        CHECK(edit_info);
        edit_info->pose_after_edit = get_pose()->clone();
        edit_info->moved_vertex_indices.clear();
        for (int i = 0; i < edit_info->pose_before_edit->vertices.size(); ++i) {
          if (edit_info->pose_before_edit->vertices[i] !=
              edit_info->pose_after_edit->vertices[i]) {
            edit_info->moved_vertex_indices.push_back(i);
          }
        }
        LOG(INFO) << "leaving internal edit loop. edited = "
                  << edit_info->moved_vertex_indices.size();
        edit_info = nullptr;
        res.key = kEscKey;
      } else {
        LOG(INFO) << "entering internal edit loop.";
        CHECK(!edit_info);
        res.edit_result = SShowResult::SEditResult();
        res.edit_result->pose_before_edit = get_pose()->clone();
        edit_info = &res.edit_result.value();
        while (show(delay_ms) != kEscKey) {
        }
        CHECK(!edit_info);
        CHECK(res.edit_result->pose_before_edit);
        CHECK(res.edit_result->pose_after_edit);
        save_intermediate();
      }
    } break;
    case kNoKeyPress:
      break;
    default:
      LOG(WARNING) << "Unknown key code: " << res.key;
  }

  if (!in_internal_edit_loop()) {
    canvas->oneshot_custom_stat.clear();
  }
  cv::imshow(window_name, canvas->img);
  return res;
}

int SVisualEditor::get_mouseover_node_id() const {
  int radius = canvas->mag / 2;
  int x = mp->x, y = mp->y;
  int idx = -1, min_d2 = INT_MAX;
  for (int i = 0; i < canvas->solution->vertices.size(); i++) {
    auto v = canvas->solution->vertices[i];
    auto [vx, vy] = canvas->cvt(v);
    int d2 = (x - vx) * (x - vx) + (y - vy) * (y - vy);
    if (d2 > radius * radius)
      continue;
    if (d2 < min_d2) {
      min_d2 = d2;
      idx = i;
    }
  }
  return idx;
}

void SVisualEditor::callback(int e, int x, int y, int f, void* param) {
  SVisualEditor* s = static_cast<SVisualEditor*>(param);
  auto mp = s->mp;
  mp->load(e, x, y, f);
  int mouseover_id = s->get_mouseover_node_id();
  if (!mp->drugging_left())
    s->canvas->update(mouseover_id);
  if (mp->clicked_left()) {
    if (mouseover_id != -1) {
      s->selected_vertex_id = mouseover_id;
    }
  }
  if (mp->clicked_right()) {
    if (mouseover_id != -1) {
      auto it = s->canvas->marked_vertex_indices.find(mouseover_id);
      if (it == s->canvas->marked_vertex_indices.end()) {
        s->canvas->marked_vertex_indices.insert(mouseover_id);
      } else {
        s->canvas->marked_vertex_indices.erase(it);
      }
    }
  }
  if (mp->drugging_left()) {
    int id = s->selected_vertex_id;
    if (id != -1) {
      auto& p = s->canvas->solution->vertices[id];
      auto np = s->canvas->icvt(x, y);
      if (p != np) {
        p = np;
        s->canvas->update(id);
      }
    }
  }
  if (mp->released_left()) {
    int id = s->selected_vertex_id;
    if (id != -1) {
      s->canvas->solution->vertices[id] = s->canvas->icvt(x, y);
      s->canvas->update(id);
      s->selected_vertex_id = -1;
    }
  }
}

SSolutionPtr visualize_and_edit(SProblemPtr problem,
                                SSolutionPtr solution,
                                const std::string& base_solver_name) {
  SVisualEditorPtr editor = std::make_shared<SVisualEditor>(
      problem, base_solver_name + "Edit", "visualize");
  editor->set_pose(solution);
  SSolutionPtr editor_solution = problem->create_solution();
  *editor_solution = *solution;
  while (true) {
    int c = editor->show(15);
    if (c == 27) {
      editor_solution = editor->get_pose();
      const std::string file_path = "editor.pose.json";
      std::ofstream ofs(file_path);
      auto json = editor_solution->json();
      update_meta(json, base_solver_name + "PostEdit");
      update_judge(*problem, judge(*problem, *editor_solution), json);
      ofs << json;
      LOG(INFO) << "saved editor solution: " << file_path;
      break;
    }
  }
  return editor_solution;
}

// vim:ts=2 sw=2 sts=2 et ci
