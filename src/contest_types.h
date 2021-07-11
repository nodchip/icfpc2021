#pragma once
#include <optional>
#include <nlohmann/json.hpp>

using integer = int64_t;

using Edge = std::pair<integer, integer>;

using Point = std::pair<integer, integer>;
using Point2d = std::pair<double, double>;

using Line = std::array<Point, 2>;

struct SBonus {
    enum class Type { GLOBALIST, BREAK_A_LEG, WALLHACK, SUPERFLEX, INVALID };
    static constexpr integer DUMMY_PROBLEM_ID = -1;

    Type type;
    Point position; // not meaningful in Pose file.
    integer problem_id;
    SBonus(Type type, const Point& position, integer problem_id) : type(type), position(position), problem_id(problem_id) {}
    SBonus(Type type, integer problem_id) : type(type), problem_id(problem_id) {}
    SBonus(Type type) : type(type), problem_id(DUMMY_PROBLEM_ID) {}
    static Type parse_bonus_name(const std::string& name) {
      if (name == "GLOBALIST") return Type::GLOBALIST;
      if (name == "BREAK_A_LEG") return Type::BREAK_A_LEG;
      if (name == "WALLHACK") return Type::WALLHACK;
      if (name == "SUPERFLEX") return Type::SUPERFLEX;
      return Type::INVALID;
    }
    static const char* bonus_name(Type type) {
      if (type == Type::GLOBALIST) return "GLOBALIST";
      if (type == Type::BREAK_A_LEG) return "BREAK_A_LEG";
      if (type == Type::WALLHACK) return "WALLHACK";
      if (type == Type::SUPERFLEX) return "SUPERFLEX";
      return "INVALID";
    }
};

struct SSolution;
using SSolutionPtr = std::shared_ptr<SSolution>;
struct SProblem;
using SProblemPtr = std::shared_ptr<SProblem>;
struct SProblem {
    nlohmann::json json;
    std::optional<int> problem_id;
    std::vector<SBonus> bonuses; // bonuses in the JSON. these bonuses are gaind by solving this problem.
    integer epsilon = 0;
    std::vector<Point> hole_polygon;
    std::vector<Point> vertices;
    std::vector<Edge> edges;
    std::vector<SBonus> available_bonuses; // bonuses that can be used to solve this problem.
    std::optional<int> force_use_bonus_index; // available_bonuses[*force_use_bonus_index]
    SProblem() {};
    SProblem(const nlohmann::json& json);
    static SProblemPtr load_file(const std::string& path);
    static SProblemPtr load_file_ext(const std::string& path); // expand 1 -> ../data/problems/1.problem.json
    std::string str() const;
    SSolutionPtr create_solution() const;
    SSolutionPtr create_solution(const std::vector<Point>& vertices) const;
    friend std::ostream& operator<<(std::ostream& o, const SProblem& obj);
    friend std::ostream& operator<<(std::ostream& o, const SProblemPtr& obj);
};

std::vector<std::vector<int>> edges_from_vertex(const SProblem& problem);
std::vector<int> edges_from_vertex(const SProblem& problem, int vid);


struct SSolution {
    std::vector<Point> vertices;
    std::vector<SBonus> bonuses;
    SSolution();
    SSolution(const std::vector<Point>& vertices);
    SSolution(const std::vector<Point>& vertices, const std::vector<SBonus>& bonuses);
    static SSolutionPtr load_file(const std::string& path);
    SSolutionPtr clone() const {
      auto res = std::make_shared<SSolution>();
      *res = *this;
      return res;
    }
    std::string str() const;
    nlohmann::json json() const;
    friend std::ostream& operator<<(std::ostream& o, const SSolution& obj);
    friend std::ostream& operator<<(std::ostream& o, const SSolutionPtr& obj);
};


bool is_compatible(const SProblem& problem, const SSolution& solution);
