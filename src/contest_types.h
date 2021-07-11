#pragma once
#include <nlohmann/json.hpp>

using integer = int64_t;

using Edge = std::pair<integer, integer>;

using Point = std::pair<integer, integer>;
using Point2d = std::pair<double, double>;

using Line = std::array<Point, 2>;

struct SBonus {
    enum class Type { GLOBALIST, BREAK_A_LEG, WALLHACK, INVALID };
    static constexpr integer DUMMY_PROBLEM_ID = -1;

    Type type;
    Point position; // not meaningful in Pose file.
    integer problem_id;
    SBonus(Type type, const Point& position, integer problem_id) : type(type), position(position), problem_id(problem_id) {}
    SBonus(Type type) : type(type), problem_id(DUMMY_PROBLEM_ID) {}
    static Type parse_bonus_name(const std::string& name) {
      if (name == "GLOBALIST") return Type::GLOBALIST;
      if (name == "BREAK_A_LEG") return Type::BREAK_A_LEG;
      if (name == "WALLHACK") return Type::WALLHACK;
      return Type::INVALID;
    }
    static const char* bonus_name(Type type) {
      if (type == Type::GLOBALIST) return "GLOBALIST";
      if (type == Type::BREAK_A_LEG) return "BREAK_A_LEG";
      if (type == Type::WALLHACK) return "WALLHACK";
      return "INVALID";
    }
};

struct SProblem;
using SProblemPtr = std::shared_ptr<SProblem>;
struct SProblem {
    nlohmann::json json;
    std::vector<SBonus> bonuses; // bonuses in the JSON. these bonuses are gaind by solving this problem.
    integer epsilon = 0;
    std::vector<Point> hole_polygon;
    std::vector<Point> vertices;
    std::vector<Edge> edges;
    bool is_globalist_mode = false; // solve this problem in GLOBALIST mode.
    bool is_wallhack_mode = false; // solve this problem in WALLHACK mode.
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
    std::vector<SBonus> bonuses;
    SSolution() {};
    SSolution(const std::vector<Point>& vertices);
    SSolution(const std::vector<Point>& vertices, const std::vector<SBonus>& bonuses);
    static SSolutionPtr load_file(const std::string& path);
    SSolutionPtr clone() const {
      auto res = std::make_shared<SSolution>();
      *res = *this;
      return res;
    }
    void set_used_bonuses(const SProblem& problem) {
      if (problem.is_globalist_mode) {
        bonuses.push_back(SBonus(SBonus::Type::GLOBALIST)); // TODO: include problem id (refactoring required)
      }
      if (problem.is_wallhack_mode) {
        bonuses.push_back(SBonus(SBonus::Type::WALLHACK)); // TODO: include problem id (refactoring required)
      }
    }
    std::string str() const;
    nlohmann::json json() const;
    friend std::ostream& operator<<(std::ostream& o, const SSolution& obj);
    friend std::ostream& operator<<(std::ostream& o, const SSolutionPtr& obj);
};


bool is_compatible(const SProblem& problem, const SSolution& solution);
