#include "stdafx.h"

#include <filesystem>
#include <iostream>

#include "contest_types.h"
#include "fmt/core.h"
#include "nlohmann/json.hpp"
#include "util.h"

int main(int argc, char* argv[]) {
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

    std::cout << std::filesystem::exists(input_data_path) << std::endl;

    std::ifstream input_data_ifs(input_data_path);

    nlohmann::json j;
    input_data_ifs >> j;

    SProblemPtr problem = std::make_shared<SProblem>(j);

    debug(*problem);

    debug(problem->vertices);

    nlohmann::json test_json(problem->vertices);
    debug(test_json.dump());

    return 0;
}
