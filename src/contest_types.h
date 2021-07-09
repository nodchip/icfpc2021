#pragma once
#include "nlohmann/json.hpp"

namespace NTypes {

    using integer = int64_t;

    using Edge = std::pair<integer, integer>;

    using Point = std::pair<integer, integer>;

    struct SProblem {
        nlohmann::json json;
        integer epsilon;
        std::vector<Point> hole_polygon;
        std::vector<Point> vertices;
        std::vector<Edge> edges;
        SProblem(const nlohmann::json& json);
        std::string str() const;
        friend std::ostream& operator<<(std::ostream& o, const SProblem& obj);
    };
    using SProblemPtr = std::shared_ptr<SProblem>;
}