#include "stdafx.h"

#include "util.h"
#include "contest_types.h"
#include "solver_registry.h"
#include "judge.h"
#include "fmt/format.h"

#if 1
#include <opencv2/core.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/highgui.hpp>

namespace NManualSolver {

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
    bool is_valid;

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
        std::string stat_str = fmt::format("dislikes={}, fit={}, stretch={}, is_valid={}", dislikes, fit_in_hole, satisfy_stretch, is_valid);
        cv::putText(img, stat_str, cv::Point(20, 30), cv::FONT_HERSHEY_SIMPLEX, 0.7, cv::Scalar(0, 0, 0), 1, cv::LINE_AA);
    }

    void update(int selected_id) {
        // judge
        auto res = judge(*problem, *solution);
        dislikes = res.dislikes;
        fit_in_hole = res.fit_in_hole();
        satisfy_stretch = res.satisfy_stretch();
        for (int eid = 0; eid < problem->edges.size(); eid++) {
            edge_colors[eid] = get_edge_color(eid);
        }
        is_valid = res.is_valid();
        // draw
        img = img_base.clone();
        for (int eid : res.out_of_hole_edges) {
            auto [u, v] = problem->edges[eid];
            auto [x1, y1] = cvt(solution->vertices[u]);
            auto [x2, y2] = cvt(solution->vertices[v]);
            draw_line(img, x1, y1, x2, y2, out_of_hole_edge_color, 3);
        }
        for (int eid = 0; eid < problem->edges.size(); eid++) {
            auto [u, v] = problem->edges[eid];
            auto [x1, y1] = cvt(solution->vertices[u]);
            auto [x2, y2] = cvt(solution->vertices[v]);
            draw_line(img, x1, y1, x2, y2, edge_colors[eid], 2);
        }
        if (selected_id != -1) {
            auto [x, y] = cvt(solution->vertices[selected_id]);
            draw_circle(img, x, y, std::max(3, int(mag) / 2), cv::Scalar(0, 0, 255), cv::FILLED);
        }
        for (int vid : res.out_of_hole_vertices) {
            auto [x, y] = cvt(solution->vertices[vid]);
            draw_circle(img, x, y, std::max(2, int(mag) / 3), violating_vertex_color, cv::FILLED);
        }
        draw_stats(img);
    }

    SCanvas(SProblemPtr problem) : problem(problem) {
        solution = std::make_shared<SSolution>();
        solution->vertices = problem->vertices;
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
    }
};
using SCanvasPtr = std::shared_ptr<SCanvas>;

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
using SMouseParamsPtr = std::shared_ptr<SMouseParams>;

struct SManualSolver {

    SCanvasPtr canvas;

    const std::string window_name;
    SMouseParamsPtr mp;

    int selected_vertex_id;

    SManualSolver(SProblemPtr problem, const std::string window_name = "manual") : window_name(window_name) {
        canvas = std::make_shared<SCanvas>(problem);
        mp = std::make_shared<SMouseParams>();
        cv::namedWindow(window_name, cv::WINDOW_AUTOSIZE);
        cv::setMouseCallback(window_name, callback, this);
        selected_vertex_id = -1;
    }

    int get_mouseover_node_id() const {
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

    static void callback(int e, int x, int y, int f, void* param) {
        SManualSolver* s = static_cast<SManualSolver*>(param);
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

    SSolutionPtr solve() {
        while (true) {
            int c = cv::waitKey(15);
            if (c == 27) {
                return canvas->solution;
            }
            cv::imshow(window_name, canvas->img);
        }
        return nullptr;
    }
};
using SManualSolverPtr = std::shared_ptr<SManualSolver>;

}
#endif

class ManualSolver : public SolverBase {
public:
    ManualSolver() { }
    SolverOutputs solve(const SolverArguments &args) override {
#if 1
        using namespace NManualSolver;
        SolverOutputs ret;

        SManualSolverPtr manual_solver = std::make_shared<SManualSolver>(args.problem);

        ret.solution = manual_solver->solve();
        return ret;
#else
        SolverOutputs ret;

        // do nothing.
        ret.solution = std::make_shared<SSolution>();
        ret.solution->vertices = args.problem->vertices;

        // dummy.
        std::this_thread::sleep_for(std::chrono::milliseconds(500));

        return ret;
#endif
    }
};

REGISTER_SOLVER("ManualSolver", ManualSolver);
// vim:ts=2 sw=2 sts=2 et ci

