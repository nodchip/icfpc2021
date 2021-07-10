#pragma once
#include <nlohmann/json.hpp>

using integer = int64_t;

using Edge = std::pair<integer, integer>;

using Point = std::pair<integer, integer>;
using Line = std::array<Point, 2>;

struct SBonus {
    enum class Type { GLOBALIST, BREAK_A_LEG };
    Type type;
    Point position;
    integer problem_id;
    SBonus(Type type, const Point& position, integer problem_id) : type(type), position(position), problem_id(problem_id) {}
};

struct SProblem;
using SProblemPtr = std::shared_ptr<SProblem>;
struct SProblem {
    nlohmann::json json;
    std::vector<SBonus> bonuses;
    integer epsilon = 0;
    std::vector<Point> hole_polygon;
    std::vector<Point> vertices;
    std::vector<Edge> edges;
    SProblem() {};
    SProblem(const nlohmann::json& json);
    static SProblemPtr load_file(const std::string& path);
    static SProblemPtr load_file_ext(const std::string& path); // expand 1 -> ../data/problems/1.problem.json
    std::string str() const;
    friend std::ostream& operator<<(std::ostream& o, const SProblem& obj);
    friend std::ostream& operator<<(std::ostream& o, const SProblemPtr& obj);
};

std::vector<std::vector<int>> edges_from_vertex(const SProblem& problem);
std::vector<int> edges_from_vertex(const SProblem& problem, int vid);


struct SSolution;
using SSolutionPtr = std::shared_ptr<SSolution>;
struct SSolution {
    std::vector<Point> vertices;
    SSolution() {};
    SSolution(const std::vector<Point>& vertices);
    static SSolutionPtr load_file(const std::string& path);
    std::string str() const;
    nlohmann::json json() const;
    friend std::ostream& operator<<(std::ostream& o, const SSolution& obj);
    friend std::ostream& operator<<(std::ostream& o, const SSolutionPtr& obj);
};

bool is_compatible(const SProblem& problem, const SSolution& solution);
