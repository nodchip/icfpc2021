#include "stdafx.h"
#include "visual_editor.h"

#include <set>
#include <opencv2/core.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/highgui.hpp>
#include <fmt/format.h>

#include "solver_registry.h"
#include "judge.h"
#include "util.h"

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

cv::Mat_<cv::Vec3b> create_image(SProblemPtr prob) {
    auto rect_poly = calc_bb(prob->hole_polygon);
    auto rect_fig = calc_bb(prob->vertices);
    integer x_min = std::min(rect_poly.x, rect_fig.x);
    integer x_max = std::max(rect_poly.x + rect_poly.width, rect_fig.x + rect_fig.width);
    integer y_min = std::min(rect_poly.y, rect_fig.y);
    integer y_max = std::max(rect_poly.y + rect_poly.height, rect_fig.y + rect_fig.height);
    integer img_offset = 5;
    integer img_base_size = std::max(x_max, y_max) + img_offset * 2;
    integer mag = 1200 / img_base_size;
    cv::Mat_<cv::Vec3b> img(img_base_size * mag, img_base_size * mag, cv::Vec3b(255, 255, 255));
    auto cvt = [&img_offset, &mag](int x, int y) { return cv::Point((x + img_offset) * mag, (y + img_offset) * mag); };
    for (int x = x_min; x <= x_max; x++) {
        for (int y = y_min; y <= y_max; y++) {
            cv::circle(img, cvt(x, y), 2, cv::Scalar(200, 200, 200), cv::FILLED);
        }
    }
    for (auto [u, v] : prob->edges) {
        auto [x1, y1] = prob->vertices[u];
        auto [x2, y2] = prob->vertices[v];
        cv::line(img, cvt(x1, y1), cvt(x2, y2), cv::Scalar(0, 0, 255), 2);
    }
    int nh = prob->hole_polygon.size();
    for (int i = 0; i < nh; i++) {
        auto [x1, y1] = prob->hole_polygon[i];
        auto [x2, y2] = prob->hole_polygon[(i + 1) % nh];
        cv::line(img, cvt(x1, y1), cvt(x2, y2), cv::Scalar(0, 0, 0), 2);
    }
    return img;
}

struct SCanvas {
    SProblemPtr problem;
    SSolutionPtr solution;

    integer img_offset;
    integer img_base_size;
    integer mag;
    integer img_size;
    cv::Mat_<cv::Vec3b> img_base;
    cv::Mat_<cv::Vec3b> img;

    integer dislikes;
    bool fit_in_hole;
    bool satisfy_stretch;
    int num_no_fit_in_hole_vert = 0;
    int num_no_fit_in_hole_edge = 0;
    int num_no_satisfy_stretch = 0;
    int num_gained_bonuses = 0;
    bool is_valid = false;

    bool draw_distant_hole_vertex = true;
    bool draw_tolerated_vertex = true;
    bool draw_edge_lengths_mode = true;

    std::vector<cv::Scalar> edge_colors;

    cv::Scalar violating_vertex_color = cv::Scalar(128, 0, 128);
    cv::Scalar out_of_hole_edge_color = cv::Scalar(128, 0, 128);

    inline cv::Point cvt(int x, int y) { return cv::Point((x + img_offset) * mag, (y + img_offset) * mag); };
    inline cv::Point cvt(const Point& p) { return cvt(p.first, p.second); }
    inline Point icvt(int x, int y) { return { x / mag - img_offset, y / mag - img_offset }; }

    void draw_circle(cv::Mat& img, int x, int y, int sz, cv::Scalar col, int thickness) {
        cv::circle(img, cv::Point(x, y), sz, col, thickness);
    }

    void draw_line(cv::Mat& img, int x1, int y1, int x2, int y2, cv::Scalar col, int thickness) {
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
        if (diff < -thresh) return cv::Scalar(255, 0, 0);
        if (diff > thresh) return cv::Scalar(0, 0, 255);
        return cv::Scalar(0, 255, 0);
    }

    void draw_stats(cv::Mat& img) {
        std::string stat_str = fmt::format("dislikes={}, fit={}(NG edge {} vert {}), stretch={}(NG {}), is_valid={}, bonus={}",
          dislikes, fit_in_hole, num_no_fit_in_hole_edge, num_no_fit_in_hole_vert, satisfy_stretch, num_no_satisfy_stretch, is_valid, num_gained_bonuses);
        cv::putText(img, stat_str, cv::Point(20, 30), cv::FONT_HERSHEY_SIMPLEX, 0.7, is_valid ? cv::Scalar(0, 0, 0) : cv::Scalar(0, 0, 128), 1, cv::LINE_AA);
    }

    void draw_edge_lengths(cv::Mat& img) {
        int nh = problem->hole_polygon.size();
        for (int i = 0; i < nh; i++) {
            Point raw_u = problem->hole_polygon[i];
            Point raw_v = problem->hole_polygon[(i + 1) % nh];
            cv::Point u = cvt(raw_u), v = cvt(raw_v);
            integer d2 = distance2(raw_u, raw_v);
            cv::putText(img, std::to_string(d2), (u + v) / 2, cv::FONT_HERSHEY_SIMPLEX, 0.35, cv::Scalar(0, 0, 0), 1, cv::LINE_AA);
        }
        for(int eid = 0; eid < problem->edges.size(); eid++) {
            auto [iu, iv] = problem->edges[eid];
            Point raw_u = solution->vertices[iu];
            Point raw_v = solution->vertices[iv];
            Point orig_raw_u = problem->vertices[iu];
            Point orig_raw_v = problem->vertices[iv];
            cv::Point u = cvt(raw_u), v = cvt(raw_v);
            integer d2 = distance2(raw_u, raw_v);
            integer orig_d2 = distance2(orig_raw_u, orig_raw_v);
            auto col = get_edge_color(eid);
            cv::putText(img, std::to_string(d2) + "/" + std::to_string(orig_d2), (u + v) / 2, cv::FONT_HERSHEY_SIMPLEX, 0.35, col / 255 * 150, 1, cv::LINE_AA);
        }
    }

    void shift(int dx, int dy) {
        for (auto& p : solution->vertices) {
          p.first += dx;
          p.second += dy;
        }
    }

    void rotate_clockwise() {
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

    void update(int selected_id) {
        // judge
        auto res = judge(*problem, *solution);
        dislikes = res.dislikes;
        fit_in_hole = res.fit_in_hole();
        satisfy_stretch = res.satisfy_stretch();
        num_no_fit_in_hole_vert = res.out_of_hole_vertices.size();
        num_no_fit_in_hole_edge = res.out_of_hole_edges.size();
        num_no_satisfy_stretch = res.stretch_violating_edges.size();
        num_gained_bonuses = res.gained_bonus_indices.size();
        for (int eid = 0; eid < problem->edges.size(); eid++) {
            edge_colors[eid] = get_edge_color(eid);
        }
        is_valid = res.is_valid();
        // draw
        img = img_base.clone();
        if (draw_distant_hole_vertex) {
          for (int i = 0; i < problem->hole_polygon.size(); i++) {
              auto [x, y] = cvt(problem->hole_polygon[i]);
              auto r = 2.0 * std::log(double(res.individual_dislikes[i]) + 1e-6) + 1.0;
              if (r > 0.0) {
                draw_circle(img, x, y, r, cv::Scalar(0, 0, 128), 1);
              }
          }
        }
        for (int eid : res.out_of_hole_edges) {
            auto [u, v] = problem->edges[eid];
            auto [x1, y1] = cvt(solution->vertices[u]);
            auto [x2, y2] = cvt(solution->vertices[v]);
            draw_line(img, x1, y1, x2, y2, out_of_hole_edge_color, 5); // 3 is insufficient.
        }
        for (int eid = 0; eid < problem->edges.size(); eid++) {
            auto [u, v] = problem->edges[eid];
            auto [x1, y1] = cvt(solution->vertices[u]);
            auto [x2, y2] = cvt(solution->vertices[v]);
            draw_line(img, x1, y1, x2, y2, edge_colors[eid], 2);
        }
        if (selected_id != -1) {
            if (draw_tolerated_vertex) {
                auto bb = calc_bb(problem->hole_polygon);
                auto edges = edges_from_vertex(*problem, selected_id);
                std::vector<std::set<Point>> exact_grids;
                for (auto eid : edges) {
                    std::set<Point> exact_grid;
                    auto [u, v] = problem->edges[eid];
                    const int counter_vid = u == selected_id ? v : u;
                    const auto org_d2 = distance2(problem->vertices[selected_id], problem->vertices[counter_vid]); 
                    for (int y = bb.tl().y; y <= bb.br().y; ++y) {
                        for (int x = bb.tl().x; x <= bb.br().x; ++x) {
                            const auto moved_d2 = distance2({x, y}, solution->vertices[counter_vid]); 
                            if (tolerate(org_d2, moved_d2, problem->epsilon)) {
                                exact_grid.insert({x, y});
                            }
                        }
                    }
                    for (auto v : exact_grid) {
                        auto [x, y] = cvt(v);
                        draw_circle(img, x, y, std::max(4, int(mag) / 4), cv::Scalar(64, 0, 0), cv::FILLED);
                    }
                    exact_grids.emplace_back(std::move(exact_grid));
                }
                if (!exact_grids.empty()) { // common feasible position.
                    std::set<Point> intersection = exact_grids[0];
                    for (int i = 1; i < exact_grids.size(); ++i) {
                        std::set<Point> tmp;
                        std::set_intersection(intersection.begin(), intersection.end(), exact_grids[i].begin(), exact_grids[i].end(), std::inserter(tmp, tmp.end()));
                        std::swap(intersection, tmp);
                    }
                    for (auto v : intersection) {
                        auto [x, y] = cvt(v);
                        draw_circle(img, x, y, std::max(8, int(mag) / 2), cv::Scalar(255, 64, 64), cv::FILLED);
                    }
                }
            }
            auto [x, y] = cvt(solution->vertices[selected_id]);
            draw_circle(img, x, y, std::max(3, int(mag) / 2), cv::Scalar(0, 0, 255), cv::FILLED);
        }
        for (int vid : res.out_of_hole_vertices) {
            auto [x, y] = cvt(solution->vertices[vid]);
            draw_circle(img, x, y, std::max(2, int(mag) / 3), violating_vertex_color, cv::FILLED);
        }
        draw_stats(img);
        if (draw_edge_lengths_mode) {
            draw_edge_lengths(img);
        }
    }

    bool set_pose(SSolutionPtr pose) {
        CHECK(is_compatible(*problem, *pose));
        solution = pose;
        // draw base image
        auto rect_poly = calc_bb(problem->hole_polygon);
        auto rect_fig = calc_bb(problem->vertices);
        integer x_min = std::min(rect_poly.x, rect_fig.x);
        integer x_max = std::max(rect_poly.x + rect_poly.width, rect_fig.x + rect_fig.width);
        integer y_min = std::min(rect_poly.y, rect_fig.y);
        integer y_max = std::max(rect_poly.y + rect_poly.height, rect_fig.y + rect_fig.height);
        img_offset = 5;
        img_base_size = std::max(x_max, y_max) + img_offset * 2;
        mag = 1200 / img_base_size;
        img_size = img_base_size * mag;
        img_base = cv::Mat_<cv::Vec3b>(img_size, img_size, cv::Vec3b(160, 160, 160));
        std::vector<cv::Point> cv_hole_polygon;
        for (auto p : problem->hole_polygon) {
          auto [x, y] = cvt(p);
          cv_hole_polygon.emplace_back(x, y);
        }
        cv::fillPoly(img_base, cv_hole_polygon, cv::Scalar(255, 255, 255));
        for (auto& bonus : problem->bonuses) {
            auto [x, y] = bonus.position;
            cv::Scalar color;
            if (bonus.type == SBonus::Type::GLOBALIST) {
              color = cv::Scalar(32, 192, 192); 
            } else if (bonus.type == SBonus::Type::BREAK_A_LEG) {
              color = cv::Scalar(192, 192, 32); 
            }
            cv::circle(img_base, cvt(x, y), 20, color, cv::FILLED);
        }
        for (int x = x_min; x <= x_max; x++) {
            for (int y = y_min; y <= y_max; y++) {
                cv::circle(img_base, cvt(x, y), 2, cv::Scalar(200, 200, 200), cv::FILLED);
            }
        }
        int nh = problem->hole_polygon.size();
        for (int i = 0; i < nh; i++) {
            auto [x1, y1] = cvt(problem->hole_polygon[i]);
            auto [x2, y2] = cvt(problem->hole_polygon[(i + 1) % nh]);
            draw_line(img_base, x1, y1, x2, y2, cv::Scalar(0, 0, 0), 2);
        }
        edge_colors.resize(problem->edges.size(), cv::Scalar(0, 255, 0));
        update(-1);
        return true;
    }

    SCanvas(SProblemPtr problem) : problem(problem) {
        auto pose = std::make_shared<SSolution>();
        pose->vertices = problem->vertices;
        set_pose(pose);
    }
};

struct SMouseParams {
    int pe, px, py, pf;
    int e, x, y, f;
    SMouseParams() { e = x = y = f = pe = px = py = pf = INT_MAX; };
    inline void load(int e_, int x_, int y_, int f_) {
        pe = e; px = x; py = y; pf = f;
        e = e_; x = x_; y = y_; f = f_;
    }
    inline bool clicked_left() const { return e == 1 && pe == 0; }
    inline bool clicked_right() const { return e == 2 && pe == 0; }
    inline bool released_left() const { return e == 4; }
    inline bool released_right() const { return e == 5; }
    inline bool drugging_left() const { return e == 0 && f == 1; }
    inline bool drugging_right() const { return e == 0 && f == 2; }
    inline std::pair<int, int> coord() const { return { x, y }; }
    inline std::pair<int, int> displacement() const { return { abs(x - px) > 10000 ? 0 : (x - px), abs(y - py) ? 0 : (y - py) }; }
    std::string str() const {
        return fmt::format("SMouseParams [(e,x,y,f)=({},{},{},{}), (pe,px,py,pf)=({},{},{},{})", e, x, y, f, pe, px, py, pf);
    }
    friend std::ostream& operator<<(std::ostream& o, const SMouseParams& obj) {
        o << obj.str();
        return o;
    }
    friend std::ostream& operator<<(std::ostream& o, const std::shared_ptr<SMouseParams>& obj) {
        o << obj->str();
        return o;
    }
};

SVisualEditor::SVisualEditor(SProblemPtr problem, const std::string& solver_name, const std::string window_name)
  : window_name(window_name), solver_name(solver_name) {
    canvas = std::make_shared<SCanvas>(problem);
    mp = std::make_shared<SMouseParams>();
    cv::namedWindow(window_name, cv::WINDOW_AUTOSIZE);
    cv::setMouseCallback(window_name, callback, this);
    selected_vertex_id = -1;
}

SVisualEditor::~SVisualEditor() {
}

bool SVisualEditor::set_pose(SSolutionPtr pose) {
    canvas->set_pose(pose);
    return false;
}

SSolutionPtr SVisualEditor::get_pose() const {
    return canvas->solution;
}

SShowResult SVisualEditor::show(int wait) {
    SShowResult res(cv::waitKey(wait));
    if (res.key == 27) {
      if (in_internal_edit_loop()) {
        res.key = 'm';
      } else {
        return res.key;
      }
    }
    if (res.key == 'd') {
        canvas->draw_distant_hole_vertex = !canvas->draw_distant_hole_vertex;
        canvas->update(-1);
    }
    if (res.key == 'e') {
        canvas->draw_edge_lengths_mode = !canvas->draw_edge_lengths_mode;
        canvas->update(-1);
    }
    if (res.key == 't') {
        canvas->draw_tolerated_vertex = !canvas->draw_tolerated_vertex;
        canvas->update(get_mouseover_node_id());
    }
    if (res.key == 's') {
        const std::string file_path = "intermediate.pose.json";
        std::ofstream ofs(file_path);
        auto json = canvas->solution->json();
        update_meta(json, solver_name);
        update_judge(*canvas->problem, judge(*canvas->problem, *canvas->solution), json);
        ofs << json;
        LOG(INFO) << "saved: " << file_path;
    }
    if (res.key == 'h') {
        canvas->shift(-1, 0);
        canvas->update(-1);
    }
    if (res.key == 'j') {
        canvas->shift(0, 1);
        canvas->update(-1);
    }
    if (res.key == 'k') {
        canvas->shift(0, -1);
        canvas->update(-1);
    }
    if (res.key == 'l') {
        canvas->shift(1, 0);
        canvas->update(-1);
    }
    if (res.key == 'r') {
        canvas->rotate_clockwise();
        canvas->update(-1);
    }
    if (res.key == 'm') { // toggle internal edit mode.
      LOG(INFO) << "in internal edit loop? " << in_internal_edit_loop();
      if (in_internal_edit_loop()) {
        LOG(INFO) << "leaving internal edit loop.";
        CHECK(edit_info);
        edit_info->pose_after_edit = get_pose()->clone();
        edit_info->moved_vertex_indices.clear();
        for (int i = 0; i < edit_info->pose_before_edit->vertices.size(); ++i) {
          if (edit_info->pose_before_edit->vertices[i] != edit_info->pose_after_edit->vertices[i]) {
            edit_info->moved_vertex_indices.push_back(i);
          }
        }
        LOG(INFO) << "leaving internal edit loop. edited = " << edit_info->moved_vertex_indices.size();
        edit_info = nullptr;
        res.key = 27; // ESC
      } else {
        LOG(INFO) << "entering internal edit loop.";
        CHECK(!edit_info);
        res.edit_result = SShowResult::SEditResult();
        res.edit_result->pose_before_edit = get_pose()->clone();
        edit_info = &res.edit_result.value();
        while (true) {
          SShowResult res_internal = show(wait);
          if (res_internal.key == 27) {
            break;
          }
        }
        CHECK(!edit_info);
        CHECK(res.edit_result->pose_before_edit);
        CHECK(res.edit_result->pose_after_edit);
      }
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
        if (d2 > radius * radius) continue;
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
    if (!mp->drugging_left()) s->canvas->update(mouseover_id);
    if (mp->clicked_left()) {
        if (mouseover_id != -1) {
            s->selected_vertex_id = mouseover_id;
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

SSolutionPtr visualize_and_edit(SProblemPtr problem, SSolutionPtr solution, const std::string& base_solver_name) {
    SVisualEditorPtr editor = std::make_shared<SVisualEditor>(problem, base_solver_name + "Edit", "visualize");
    editor->set_pose(solution);
    SSolutionPtr editor_solution = std::make_shared<SSolution>();
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

