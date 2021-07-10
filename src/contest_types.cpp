#include "stdafx.h"
#include <fstream>
#include <iostream>
#include <filesystem>
#include "contest_types.h"
#include "util.h"

SProblem::SProblem(const nlohmann::json& json) : json(json) {
    using std::cerr;
    using std::endl;
    auto bs = json["bonuses"];
    for (auto b : bs) {
        SBonus::Type type;
        if (b["bonus"] == "GLOBALIST") type = SBonus::Type::GLOBALIST;
        else if (b["bonus"] == "BREAK_A_LEG") type = SBonus::Type::BREAK_A_LEG;
        Point position = b["position"];
        integer problem_id = b["problem"];
        bonuses.emplace_back(type, position, problem_id);
    }
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

std::ostream& operator<<(std::ostream& o, const SProblemPtr& obj) {
    o << obj->str();
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

std::vector<std::vector<int>> edges_from_vertex(const SProblem& problem) {
    std::vector<std::vector<int>> edges(problem.vertices.size());
    for (int vid = 0; vid < problem.vertices.size(); ++vid) {
      for (int eid = 0; eid < problem.edges.size(); ++eid) {
          auto [u, v] = problem.edges[eid];
          if (u == vid || v == vid) edges[vid].push_back(eid);
      }
    }
    return edges;
}

std::vector<int> edges_from_vertex(const SProblem& problem, int vid) {
    std::vector<int> edges;
    for (int eid = 0; eid < problem.edges.size(); ++eid) {
        auto [u, v] = problem.edges[eid];
        if (u == vid || v == vid) edges.push_back(eid);
    }
    return edges;
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

std::ostream& operator<<(std::ostream& o, const SSolutionPtr& obj) {
    o << obj->str();
    return o;
}

bool is_compatible(const SProblem& problem, const SSolution& solution) {
  return problem.vertices.size() == solution.vertices.size();
}
