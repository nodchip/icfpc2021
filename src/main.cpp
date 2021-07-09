#include "stdafx.h"

#include "nlohmann/json.hpp"
#include "util.h"

#include "fmt/core.h"

#include <iostream>
#include <filesystem>

using integer = int64_t;

using Edge = std::pair<integer, integer>;
using Point = std::pair<integer, integer>;

struct SProblem {
    nlohmann::json json;
    integer epsilon;
    std::vector<Point> hole_polygon;
    std::vector<Point> vertices;
    std::vector<Edge> edges;
    SProblem(const nlohmann::json& json) : json(json) {
        using std::cerr, std::endl;
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
    std::string str() const {
        return json.dump();
    }
    friend std::ostream& operator<<(std::ostream& o, const SProblem& obj) {
        o << obj.str();
        return o;
    }
};
using SProblemPtr = std::shared_ptr<SProblem>;

int main(int argc, char* argv[]) {
    using std::cin, std::cout, std::cerr, std::endl;
    using json = nlohmann::json;

    google::InitGoogleLogging(argv[0]);
    google::InstallFailureSignalHandler();
    google::SetStderrLogging(google::INFO);
    // TODO: Rename log file name.
    google::SetLogDestination(google::INFO, "main.log.");

    std::ios::sync_with_stdio(false);
    std::cin.tie(NULL);

    auto work_dir = std::filesystem::current_path();
    auto input_dir = work_dir.parent_path().parent_path().append("data").append("problems");
    auto input_data_path = input_dir.append("1.problem.json");

    cout << std::filesystem::exists(input_data_path) << endl;

    std::ifstream input_data_ifs(input_data_path);

    json j;
    input_data_ifs >> j;

    SProblemPtr problem = std::make_shared<SProblem>(j);

    debug(*problem);

    return 0;
}
