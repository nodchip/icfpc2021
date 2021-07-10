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
        return double(next_int()) / std::numeric_limits<unsigned>::max();
    }
};

class K3Solver : public SolverBase {
public:
    K3Solver() {}

    using Pose = std::vector<Point>;

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
        build_polygon_map();
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

    void build_polygon_map() {
        polygon_roi = calc_roi(hole_polygon);
        auto& roi = polygon_roi;
        inner_polygon_map.resize(roi.y_max - roi.y_min + 1, std::vector<bool>(roi.x_max - roi.x_min + 1, false));
        for (integer y = roi.y_min; y <= roi.y_max; y++) {
            for (integer x = roi.x_min; x <= roi.x_max; x++) {
                Point p(x, y);
                if (contains(hole_polygon, p)) {
                    inner_polygon_points.push_back(p);
                    inner_polygon_map[y - roi.y_min][x - roi.x_min] = true;
                }
            }
        }
    }

    bool is_inside_polygon(integer x, integer y) const {
        auto& roi = polygon_roi;
        if (x < roi.x_min || x > roi.x_max || y < roi.y_min || y > roi.y_max) return false;
        return inner_polygon_map[y - roi.y_min][x - roi.x_min];
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

    double evaluate(const Pose& pose) const {
        SSolutionPtr sol = std::make_shared<SSolution>();
        sol->vertices = pose;
        auto res = judge(*prob, *sol);
        if (!res.fit_in_hole()) return std::numeric_limits<double>::max();
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
            return penalty * penalty;
            //return abs(orig_g2 - mod_g2);
        };
        for (int eid : res.stretch_violating_edges) {
            auto [uid, vid] = edges[eid];
            auto u = pose[uid], v = pose[vid];
            stretch_cost += calc_violation(d2_orig[eid], distance2(u, v));
        }
        //return stretch_cost + res.dislikes * 3;
        double weight = 0.5 * progress_rate + 0.5;
        return weight * stretch_cost + (1.0 - weight) * res.dislikes;
    }

    SMovePtr calc_move_stat(Pose& pose, integer vid, const Point& to, double now_score) {
        auto from = pose[vid];
        pose[vid] = to;
        double new_score = evaluate(pose);
        pose[vid] = from;
        if (new_score == std::numeric_limits<double>::max()) return nullptr;
        return std::make_shared<SMove>(vid, from, to, new_score - now_score, true);
    }

    SSlidePtr calc_slide_stat(const Pose& pose, int dir, double now_score) const {
        static constexpr int di[] = { 0, -1, 0, 1 };
        static constexpr int dj[] = { 1, 0, -1, 0 };
        auto pose_cpy = pose;
        for (auto& [x, y] : pose_cpy) {
            if (!is_inside_polygon(x + dj[dir], y + di[dir])) return nullptr;
            x += dj[dir]; y += di[dir];
        }
        double new_score = evaluate(pose_cpy);
        if (new_score == std::numeric_limits<double>::max()) return nullptr;
        return std::make_shared<SSlide>(dir, new_score - now_score, true);
    }

    STransPtr create_random_trans(Pose& pose, double now_score) {
        // TODO: vertices ‚Å‚Í‚È‚­ pose ‚Å
        static constexpr int di[] = { 0, -1, 0, 1 };
        static constexpr int dj[] = { 1, 0, -1, 0 };
        int r = rnd.next_int(10);
        if (r < 8) {
            // adjacent move
            int vid = rnd.next_int(pose.size());
            auto [x, y] = pose[vid];
            int dir = rnd.next_int(4);
            if (!is_inside_polygon(x + dj[dir], y + di[dir])) return nullptr;
            return calc_move_stat(pose, vid, Point(x + dj[dir], y + di[dir]), now_score);
        }
        if (r < 9) {
            // completely random warp
            int vid = rnd.next_int(pose.size());
            int to_id = rnd.next_int(inner_polygon_points.size());
            return calc_move_stat(pose, vid, inner_polygon_points[to_id], now_score);
        }
        else {
            return calc_slide_stat(pose, rnd.next_int(4), now_score);
        }
    }

    void move_vertex(std::vector<Point>& pose, integer vid, const Point& to) {
        pose[vid] = to;
    }

    void slide_all_vertices(std::vector<Point>& pose, int dir) {
        static constexpr int di[] = { 0, -1, 0, 1 };
        static constexpr int dj[] = { 1, 0, -1, 0 };
        for (auto& [x, y] : pose) {
            x += dj[dir]; y += di[dir];
        }
    }

    void transition(std::vector<Point>& pose, STransPtr trans) {
        STrans::Type type = trans->type;
        switch (type) {
        case K3Solver::STrans::Type::MOVE:
        {
            SMovePtr mv = std::static_pointer_cast<SMove>(trans);
            move_vertex(pose, mv->vid, mv->to);
        }
            break;
        case K3Solver::STrans::Type::SLIDE:
        {
            SSlidePtr sl = std::static_pointer_cast<SSlide>(trans);
            slide_all_vertices(pose, sl->dir);
        }
            break;
        default:
            break;
        }
    }

    std::vector<Point> scatter(std::vector<Point> pose, integer num_loop) {
        double now_score = evaluate(pose);
        for (integer loop = 0; loop < num_loop; loop++) {
            int vid = rnd.next_int(pose.size());
            int to_id = rnd.next_int(inner_polygon_points.size());
            SMovePtr mv = calc_move_stat(pose, vid, inner_polygon_points[to_id], now_score);
            if (mv->is_valid) {
                move_vertex(pose, mv->vid, mv->to);
                now_score += mv->diff;
            }
            if (editor && loop % 1000 == 0) {
                LOG(INFO) << "loop = " << loop << ", score = " << now_score;
                editor->set_pose(std::make_shared<SSolution>(pose));
                int c = editor->show(1);
            }
            loop++;
        }
        return pose;
    }

    SolverOutputs solve(const SolverArguments& args) override {
        initialize(args);
        
        auto pose = vertices_orig;

        // initial state
        auto representative_point = calc_representative_polygon_point(inner_polygon_points);
        for (int i = 0; i < pose.size(); i++) move_vertex(pose, i, representative_point);

        constexpr bool visualize = true;
        if (visualize) {
            editor = std::make_shared<SVisualEditor>(args.problem, "visualize");
        }

        auto get_temp = [](double stemp, double etemp, double loop, double num_loop) {
            return etemp + (stemp - etemp) * (num_loop - loop) / num_loop;
        };

        //vertices = scatter(vertices, 10000);

        progress_rate = 0.0;
        double now_score = evaluate(pose);
        integer loop = 0, num_loop = 3000000, accepted = 0;
        for(integer loop = 0; loop < num_loop; loop++) {
            if (editor && loop % 1000 == 0) {
                progress_rate = double(loop) / num_loop;
                LOG(INFO) 
                    << "loop = " << loop 
                    << ", progress_rate = " << progress_rate
                    << ", accept_rate = " << double(accepted) / loop
                    << ", score = " << now_score;
                editor->set_pose(std::make_shared<SSolution>(pose));
                int c = editor->show(1);
            }
            STransPtr trans = create_random_trans(pose, now_score);
            if (!trans) continue;
            double temp = get_temp(30.0, 0.0, loop, num_loop);
            double prob = exp(-trans->diff / temp);
            if (rnd.next_double() < prob) {
                accepted++;
                transition(pose, trans);
                now_score += trans->diff;
            }
            loop++;
        }

        // do nothing.
        SolverOutputs ret;
        ret.solution = std::make_shared<SSolution>();
        ret.solution->vertices = pose;
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
    SROI polygon_roi;
    std::vector<std::vector<bool>> inner_polygon_map;
    std::vector<Point> inner_polygon_points;
    // for evaluation
    double progress_rate;
    // for visualize
    SVisualEditorPtr editor;
};

REGISTER_SOLVER("K3Solver", K3Solver);
// vim:ts=2 sw=2 sts=2 et ci

