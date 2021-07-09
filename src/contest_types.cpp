#include "stdafx.h"
#include <fstream>
#include <iostream>
#include "contest_types.h"

SProblem::SProblem(const nlohmann::json& json) : json(json) {
    using std::cerr;
    using std::endl;
    epsilon = json["epsilon"];
    auto hs = json["hole"];
    for (auto h : hs) {
        hole_polygon.emplace_back(h[0], h[1]);
    }
    auto figure = json["figure"];
    auto vs = figure["vertices"];
    for (auto v : vs) {
        vertices.emplace_back(v[0], v[1]);
    }
    auto es = figure["edges"];
    for (auto e : es) {
        edges.emplace_back(e[0], e[1]);
    }
}

std::string SProblem::str() const {
    return json.dump();
}

std::ostream& operator<<(std::ostream& o, const SProblem& obj) {
    o << obj.str();
    return o;
}

SProblemPtr SProblem::load_file(const std::string& path) {
    std::ifstream input_data_ifs(path);
    nlohmann::json j;
    input_data_ifs >> j;
    return std::make_shared<SProblem>(j);
}

SSolution::SSolution(const std::vector<Point>& vertices) : vertices(vertices) {}

SSolutionPtr SSolution::load_file(const std::string& path) {
    std::ifstream input_data_ifs(path);
    nlohmann::json j;
    input_data_ifs >> j;
    return std::make_shared<SSolution>(j);
}

std::string SSolution::str() const {
    nlohmann::json json(vertices);
    return json.dump();
}

std::ostream& operator<<(std::ostream& o, const SSolution& obj) {
    o << obj.str();
    return o;
}