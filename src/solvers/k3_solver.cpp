#include "stdafx.h"

#include <thread>
#include "fmt/format.h"
#include "util.h"

#include "contest_types.h"
#include "solver_registry.h"
#include "visual_editor.h"
#include "judge.h"


struct Xorshift {
    uint64_t x = 88172645463325252LL;
    void set_seed(unsigned seed, int rep = 100) {
        x = (seed + 1) * 10007;
        for (int i = 0; i < rep; i++) next_int();
    }
    unsigned next_int() {
        x = x ^ (x << 7);
        return x = x ^ (x >> 9);
    }
    unsigned next_int(unsigned mod) {
        x = x ^ (x << 7);
        x = x ^ (x >> 9);
        return x % mod;
    }
    unsigned next_int(unsigned l, unsigned r) {
        x = x ^ (x << 7);
        x = x ^ (x >> 9);
        return x % (r - l + 1) + l;
    }
    double next_double() {
        return double(next_int()) / UINT_MAX;
    }
};

class K3Solver : public SolverBase {
public:
    K3Solver() {}

    struct SROI {
        integer x_min, y_min, x_max, y_max;
        SROI(integer x_min = 0, integer y_min = 0, integer x_max = 0, integer y_max = 0)
            : x_min(x_min), y_min(y_min), x_max(x_max), y_max(y_max) {}
        std::string str() const {
            return fmt::format("SROI [({}, {}) - ({}, {})]", x_min, y_min, x_max, y_max);
        }
        friend std::ostream& operator<<(std::ostream& o, const SROI& obj) {
            o << obj.str();
            return o;
        }
    };

    struct STrans;
    using STransPtr = std::shared_ptr<STrans>;
    struct STrans {
        enum class Type { MOVE, SLIDE };
        Type type;
        double diff;
        bool is_valid;
        STrans(Type type, double diff, bool valid) : type(type), diff(diff), is_valid(valid) {}
    };

    struct SMove;
    using SMovePtr = std::shared_ptr<SMove>;
    struct SMove : STrans {
        integer vid;
        Point from, to;
        SMove(integer vid, const Point& from, const Point& to, double diff, bool is_valid) 
            : STrans(Type::MOVE, diff, is_valid), vid(vid), from(from), to(to) {}
        std::string str() const {
            return fmt::format("SMove [vid={}, from=({}, {}), to=({}, {}), diff={}, is_valid={}]", vid, from.first, from.second, to.first, to.second, diff, is_valid);
        }
        friend std::ostream& operator<<(std::ostream& o, const SMove& obj) {
            o << obj.str();
            return o;
        }
        friend std::ostream& operator<<(std::ostream& o, const SMovePtr& obj) {
            o << obj->str();
            return o;
        }
    };
    
    struct SSlide;
    using SSlidePtr = std::shared_ptr<SSlide>;
    struct SSlide : STrans {
        int dir;
        SSlide(int dir, double diff, bool is_valid) : STrans(Type::SLIDE, diff, is_valid), dir(dir) {}
        std::string str() const {
            return fmt::format("SSlide [dir={}, diff={}, is_valid={}]", dir, diff, is_valid);
        }
        friend std::ostream& operator<<(std::ostream& o, const SSlide& obj) {
            o << obj.str();
            return o;
        }
        friend std::ostream& operator<<(std::ostream& o, const SSlidePtr& obj) {
            o << obj->str();
            return o;
        }
    };


    void initialize(const SolverArguments& args) {
        // master data
        prob = args.problem;
        bonuses = prob->bonuses;
        epsilon = prob->epsilon;
        edges = prob->edges;
        vertices_orig = prob->vertices;
        for (auto [uid, vid] : edges) {
            auto u = vertices_orig[uid], v = vertices_orig[vid];
            d2_orig.push_back(distance2(u, v));
        }
        hole_polygon = prob->hole_polygon;
        inner_polygon_points = enum_inner_polygon_points(hole_polygon);
        // solution
        vertices = prob->vertices;
    }

    SROI calc_roi(const std::vector<Point>& points) const {
        integer x_min = std::numeric_limits<integer>::max();
        integer x_max = std::numeric_limits<integer>::min();
        integer y_min = std::numeric_limits<integer>::max();
        integer y_max = std::numeric_limits<integer>::min();
        for (auto [x, y] : points) {
            x_min = std::min(x_min, x);
            x_max = std::max(x_max, x);
            y_min = std::min(y_min, y);
            y_max = std::max(y_max, y);
        }
        return SROI(x_min, y_min, x_max, y_max);
    }

    void build_polygon_map(const std::vector<Point>& polygon) const {
        SROI roi = calc_roi(polygon);
        
    }

    std::vector<Point> enum_inner_polygon_points(const std::vector<Point>& polygon) const {
        std::vector<Point> res;
        SROI roi = calc_roi(polygon);
        for (integer x = roi.x_min; x <= roi.x_max; x++) {
            for (integer y = roi.y_min; y <= roi.y_max; y++) {
                Point p(x, y);
                if (contains(polygon, p)) {
                    res.push_back(p);
                }
            }
        }
        return res;
    }

    Point calc_representative_polygon_point(const std::vector<Point>& inner_polygon_points) const {
        int npoints = inner_polygon_points.size();
        double x_sum = 0.0, y_sum = 0.0;
        for (auto [x, y] : inner_polygon_points) {
            x_sum += x; y_sum += y;
        }
        x_sum /= npoints; y_sum /= npoints;
        double nearest_d2 = std::numeric_limits<double>::max();
        Point nearest_point;
        for (auto [x, y] : inner_polygon_points) {
            double d2 = (x_sum - x) * (x_sum - x) + (y_sum - y) * (y_sum - y);
            if (d2 < nearest_d2) {
                nearest_d2 = d2;
                nearest_point = Point(x, y);
            }
        }
        return nearest_point;
    }

    double evaluate(const std::vector<Point>& vertices, bool verbose = false) const {
        SSolutionPtr sol = std::make_shared<SSolution>();
        sol->vertices = vertices;
        auto res = judge(*prob, *sol);
        if (verbose) {
            LOG(INFO) << "dislikes = " << res.dislikes;
            LOG(INFO) << "fit_in_hole = " << res.fit_in_hole();
            LOG(INFO) << "satisfy_stretch = " << res.satisfy_stretch();
            LOG(INFO) << "is_valid = " << res.is_valid();
            LOG(INFO) << "out_of_hole_vertices = " << res.out_of_hole_vertices;
            LOG(INFO) << "out_of_hole_edges = " << res.out_of_hole_edges;
            LOG(INFO) << "stretch_violating_edges = " << res.stretch_violating_edges;
        }
        if (!res.fit_in_hole()) return std::numeric_limits<integer>::max();
        double stretch_cost = 0.0;
        auto calc_violation = [this](integer orig_g2, integer mod_g2) {
            double window = epsilon * orig_g2 / 1000000.0;
            double penalty = 0.0;
            if (mod_g2 < orig_g2 - window) {
                penalty = orig_g2 - window - mod_g2;
            }
            if (orig_g2 + window < mod_g2) {
                penalty = mod_g2 - orig_g2 - window;
            }
            return penalty;
            //return abs(orig_g2 - mod_g2);
        };
        for (int eid : res.stretch_violating_edges) {
            auto [uid, vid] = edges[eid];
            auto u = vertices[uid], v = vertices[vid];
            stretch_cost += calc_violation(d2_orig[eid], distance2(u, v));
        }
        if (verbose) {
            LOG(INFO) << "stretch_cost = " << stretch_cost;
        }
        return stretch_cost + res.dislikes * 3;
    }

    SMovePtr calc_move_stat(integer vid, const Point& to, double now_score) const {
        auto pose = vertices;
        auto from = pose[vid];
        pose[vid] = to;
        double new_score = evaluate(pose);
        if (new_score == std::numeric_limits<double>::max()) {
            return std::make_shared<SMove>(vid, from, to, new_score, false);
        }
        return std::make_shared<SMove>(vid, from, to, new_score - now_score, true);
    }

    SSlidePtr calc_slide_stat(int dir, double now_score) const {
        auto pose = vertices;
        
    }

    void move_vertex(std::vector<Point>& pose, integer vid, const Point& to) {
        pose[vid] = to;
    }

    std::vector<Point> scatter(std::vector<Point> pose, integer num_loop) {
        double now_score = evaluate(vertices);
        for (integer loop = 0; loop < num_loop; loop++) {
            int vid = rnd.next_int(pose.size());
            int to_id = rnd.next_int(inner_polygon_points.size());
            SMovePtr mv = calc_move_stat(vid, inner_polygon_points[to_id], now_score);
            if (mv->is_valid) {
                move_vertex(pose, mv->vid, mv->to);
                now_score += mv->diff;
            }
            if (editor && loop % 1000 == 0) {
                LOG(INFO) << "loop = " << loop << ", score = " << now_score;
                editor->set_pose(std::make_shared<SSolution>(vertices));
                int c = editor->show(1);
            }
            loop++;
        }
        return pose;
    }

    SolverOutputs solve(const SolverArguments& args) override {
        initialize(args);
        
        // initial state
        auto representative_point = calc_representative_polygon_point(inner_polygon_points);
        for (int i = 0; i < vertices.size(); i++) move_vertex(vertices, i, representative_point);

        constexpr bool visualize = true;
        if (visualize) {
            editor = std::make_shared<SVisualEditor>(args.problem, "visualize");
        }

        auto get_temp = [](double stemp, double etemp, double loop, double num_loop) {
            return etemp + (stemp - etemp) * (num_loop - loop) / num_loop;
        };

        vertices = scatter(vertices, 10000);

        double now_score = evaluate(vertices);
        integer loop = 0, num_loop = 3000000;
        for(integer loop = 0; loop < num_loop; loop++) {
            int vid = rnd.next_int(vertices.size());
            int to_id = rnd.next_int(inner_polygon_points.size());
            //LOG(INFO) << "vid = " << vid << ", to_id = " << to_id << ", to = " << inner_polygon_points[to_id];
            SMovePtr mv = calc_move_stat(vid, inner_polygon_points[to_id], now_score);

            if (!mv->is_valid) continue;

            double temp = get_temp(10.0, 0.0, loop, num_loop);
            double prob = exp(-mv->diff / temp);
            if (rnd.next_double() < prob) {
                move_vertex(vertices, mv->vid, mv->to);
                now_score += mv->diff;
            }

            if (editor && loop % 1000 == 0) {
                LOG(INFO) << "loop = " << loop << ", score = " << now_score;
                editor->set_pose(std::make_shared<SSolution>(vertices));
                int c = editor->show(1);
            }
            loop++;
        }

        // do nothing.
        SolverOutputs ret;
        ret.solution = std::make_shared<SSolution>();
        ret.solution->vertices = vertices;
        return ret;
    }

private:
    Xorshift rnd;
    // master data
    SProblemPtr prob;
    std::vector<SBonus> bonuses;
    integer epsilon;
    std::vector<Edge> edges;
    std::vector<integer> d2_orig;
    std::vector<Point> vertices_orig;
    std::vector<Point> hole_polygon;
    std::vector<Point> inner_polygon_points;
    // solution
    std::vector<Point> vertices;
    // for visualize
    SVisualEditorPtr editor;
};

REGISTER_SOLVER("K3Solver", K3Solver);
// vim:ts=2 sw=2 sts=2 et ci

