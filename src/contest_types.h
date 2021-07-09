#pragma once
#include "nlohmann/json.hpp"

using integer = int64_t;

using Edge = std::pair<integer, integer>;

using Point = std::pair<integer, integer>;

struct SProblem {
    nlohmann::json json;
    integer epsilon = 0;
    std::vector<Point> hole_polygon;
    std::vector<Point> vertices;
    std::vector<Edge> edges;
    SProblem() {};
    SProblem(const nlohmann::json& json);
    std::string str() const;
    friend std::ostream& operator<<(std::ostream& o, const SProblem& obj);
};
using SProblemPtr = std::shared_ptr<SProblem>;

struct SSolution {
    std::vector<Point> vertices;
    SSolution() {};
    SSolution(const std::vector<Point>& vertices);
    std::string str() const;
    friend std::ostream& operator<<(std::ostream& o, const SSolution& obj);
};