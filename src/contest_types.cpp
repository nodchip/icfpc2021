#include "stdafx.h"
#include <fstream>
#include <iostream>
#include <filesystem>
#include "contest_types.h"
#include "util.h"

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
    if (!std::filesystem::exists(path)) {
      return nullptr;
    }
    std::ifstream input_data_ifs(path);
    nlohmann::json j;
    input_data_ifs >> j;
    return std::make_shared<SProblem>(j);
}

SProblemPtr SProblem::load_file_ext(const std::string& path) {
    if (std::filesystem::exists(path)) {
      return load_file(path);
    }
    try {
      const int num = std::stoi(path);
      return load_file(default_problem_path(num).string());
    } catch (const std::invalid_argument& e) {
      LOG(ERROR) << "not an number: " << path;
      throw e;
    } catch (const std::out_of_range& e) {
      LOG(ERROR) << "out of range: " << path;
      throw e;
    }
}

SSolution::SSolution(const std::vector<Point>& vertices) : vertices(vertices) {}

SSolutionPtr SSolution::load_file(const std::string& path) {
    if (!std::filesystem::exists(path)) {
      return nullptr;
    }
    std::ifstream input_data_ifs(path);
    nlohmann::json j;
    input_data_ifs >> j;
    return std::make_shared<SSolution>(j["vertices"]);
}

nlohmann::json SSolution::json() const {
    nlohmann::json json;
    json["vertices"] = vertices;
    return json;
}

std::string SSolution::str() const {
    return json().dump();
}

std::ostream& operator<<(std::ostream& o, const SSolution& obj) {
    o << obj.str();
    return o;
}

bool is_compatible(const SProblem& problem, const SSolution& solution) {
  return problem.vertices.size() == solution.vertices.size();
}
