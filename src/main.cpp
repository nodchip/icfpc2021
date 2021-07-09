#include "stdafx.h"

#include <chrono>
#include <filesystem>
#include <iostream>
#include <CLI/CLI.hpp>

#include "contest_types.h"
#include "fmt/core.h"
#include "nlohmann/json.hpp"
#include "util.h"
#include "judge.h"
#include "solver_registry.h"

int main(int argc, char* argv[]) {
    google::InitGoogleLogging(argv[0]);
    google::InstallFailureSignalHandler();
    google::SetStderrLogging(google::INFO);
    // TODO: Rename log file name.
    google::SetLogDestination(google::INFO, "main.log.");

    std::ios::sync_with_stdio(false);
    std::cin.tie(NULL);

    CLI::App app { "main module" };
    app.require_subcommand(0, 1);

    auto sub_solve = app.add_subcommand("solve");
    std::string solver_name;
    std::string problem_json;
    std::string solution_json;
    std::string initial_solution_json;
    bool output_meta = true;
    bool output_judge = true;
    sub_solve->add_option("solver_name", solver_name, "solver name");
    sub_solve->add_option("problem_json", problem_json, "problem JSON file path");
    sub_solve->add_option("solution_json", solution_json, "output solution JSON file path");
    sub_solve->add_option("initial_solution_json", initial_solution_json, "input solution JSON file path (optional)");
    sub_solve->add_flag("-m,--output-meta,!--no-output-meta", output_meta, "output meta info to solution JSON");
    sub_solve->add_flag("-j,--output-judge,!--no-output-judge", output_judge, "output judge info to solution JSON");

    auto sub_list_solvers = app.add_subcommand("list_solvers", "list up registered solvers");

    CLI11_PARSE(app, argc, argv);

    if (sub_solve->parsed()) {
      LOG(ERROR) << fmt::format("Solver   : {}", solver_name);

      auto solver = SolverRegistry::getSolver(solver_name);
      if (!solver) {
        LOG(ERROR) << fmt::format("solver [{0}] not found!", solver_name);
        return 0;
      }
      SProblemPtr problem = SProblem::load_file_ext(problem_json);
      LOG(INFO) << fmt::format("Problem  : {}", problem_json);

      SSolutionPtr initial_solution;
      if (std::filesystem::exists(initial_solution_json)) {
        SSolutionPtr initial_solution = SSolution::load_file(initial_solution_json);
        CHECK(initial_solution);
        CHECK(is_compatible(*problem, *initial_solution));
        LOG(INFO) << fmt::format("Initial Solution  : {}", initial_solution_json);
      }

      SolverOutputs out;
      const auto t0 = std::chrono::system_clock::now();
      out = solver->solve({ problem, initial_solution });
      const auto t1 = std::chrono::system_clock::now();
      const double solve_s = std::chrono::duration_cast<std::chrono::microseconds>(t1 - t0).count() * 1e-6;
      LOG(INFO) << fmt::format("Elapsed  : {:.2f} s", solve_s);

      {
        std::ofstream ofs(solution_json);
        nlohmann::json json = out.solution->json();
        if (output_meta) {
          if (json.find("meta") == json.end()) json["meta"] = {};
          json["meta"]["elapsed_s"] = solve_s;
        }
        if (output_judge) {
          SJudgeResult res = judge(*problem, *out.solution);
          LOG(INFO) << "judge : dislikes = " << res.dislikes;
          LOG(INFO) << "judge : fit_in_hole = " << res.fit_in_hole();
          LOG(INFO) << "judge : satisfy_stretch = " << res.satisfy_stretch();
          LOG(INFO) << "judge : is_valid = " << res.is_valid();
          update_judge(res, json);
        }
        ofs << json.dump();
        LOG(INFO) << fmt::format("Output   : {}", solution_json);
      }

      return 0;
    }

    if (sub_list_solvers->parsed()) {
      SolverRegistry::displaySolvers();
      return 0;
    }

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
