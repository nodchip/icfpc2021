#include "stdafx.h"

#include <chrono>
#include <filesystem>
#include <iostream>

#include <CLI/CLI.hpp>
#include <fmt/core.h>
#include <nlohmann/json.hpp>

#include "contest_types.h"
#include "util.h"
#include "judge.h"
#include "solver_registry.h"
#include "visual_editor.h"

int main(int argc, char* argv[]) {
    google::InitGoogleLogging(argv[0]);
    google::InstallFailureSignalHandler();
    google::SetStderrLogging(google::INFO);
    google::SetLogDestination(google::INFO, "main.log.");

    std::ios::sync_with_stdio(false);
    std::cin.tie(NULL);

    CLI::App app { "main module" };
    app.require_subcommand(0, 1);

    auto sub_solve = app.add_subcommand("solve");
    std::string solver_name;
    std::string problem_json;
    std::string solution_json = "out.pose.json";
    std::string initial_solution_json;
    bool output_meta = true;
    bool output_judge = true;
    bool visualize = false;
    bool post_edit = false;
    bool offer_globalist_bonus = false;
    bool offer_wallhack_bonus = false;
    sub_solve->add_option("solver_name", solver_name, "solver name");
    sub_solve->add_option("problem_json", problem_json, "problem JSON file path");
    sub_solve->add_option("solution_json", solution_json, "output solution JSON file path (optional)");
    sub_solve->add_option("initial_solution_json", initial_solution_json, "input solution JSON file path (optional)");
    sub_solve->add_flag("-m,--output-meta,!--no-output-meta", output_meta, "output meta info to solution JSON");
    sub_solve->add_flag("-j,--output-judge,!--no-output-judge", output_judge, "output judge info to solution JSON");
    sub_solve->add_flag("--offer-globalist", offer_globalist_bonus, "offer GLOBALIST bonus from nowhere");
    sub_solve->add_flag("--offer-wallhack", offer_wallhack_bonus, "offer WALLHACK bonus from nowhere");
    sub_solve->add_flag("--visualize", visualize, "realtime visualize");
    sub_solve->add_flag("--post-edit", post_edit, "post edit output");

    auto sub_list_solvers = app.add_subcommand("list_solvers", "list up registered solvers");

    std::string solution_json_base = "try_globalist";
    auto sub_try_globalist = app.add_subcommand("try_globalist");
    sub_try_globalist->add_option("solver_name", solver_name, "solver name");
    sub_try_globalist->add_option("problem_json", problem_json, "problem JSON file path");
    sub_try_globalist->add_option("solution_json_base", solution_json_base, "output solution JSON file path (optional)");
    sub_try_globalist->add_option("initial_solution_json", initial_solution_json, "input solution JSON file path (optional)");
    sub_try_globalist->add_flag("-m,--output-meta,!--no-output-meta", output_meta, "output meta info to solution JSON");
    sub_try_globalist->add_flag("-j,--output-judge,!--no-output-judge", output_judge, "output judge info to solution JSON");
    sub_try_globalist->add_flag("--visualize", visualize, "realtime visualize");
    sub_try_globalist->add_flag("--post-edit", post_edit, "post edit output");

    CLI11_PARSE(app, argc, argv);

    if (sub_solve->parsed()) {
      LOG(ERROR) << fmt::format("Solver   : {}", solver_name);

      auto solver = SolverRegistry::getSolver(solver_name);
      if (!solver) {
        LOG(ERROR) << fmt::format("solver [{0}] not found!", solver_name);
        return 0;
      }
      SProblemPtr problem = SProblem::load_file_ext(problem_json);
      problem->is_globalist_mode = offer_globalist_bonus;
      problem->is_wallhack_mode = offer_wallhack_bonus;
      LOG(INFO) << fmt::format("Problem  : {}", problem_json);
      LOG(INFO) << fmt::format("Offerred bonuses: GLOBALIST={}, WALLHACK={}", offer_globalist_bonus, offer_wallhack_bonus);

      SSolutionPtr initial_solution;
      if (std::filesystem::exists(initial_solution_json)) {
        initial_solution = SSolution::load_file(initial_solution_json);
        CHECK(initial_solution);
        CHECK(is_compatible(*problem, *initial_solution));
        LOG(INFO) << fmt::format("Initial Solution  : {}", initial_solution_json);
      }

      SolverOutputs out;
      const auto t0 = std::chrono::system_clock::now();
      out = solver->solve({ problem, initial_solution, visualize });
      const auto t1 = std::chrono::system_clock::now();
      const double solve_s = std::chrono::duration_cast<std::chrono::microseconds>(t1 - t0).count() * 1e-6;
      LOG(INFO) << fmt::format("Elapsed  : {:.2f} s", solve_s);

      {
        nlohmann::json json = out.solution->json();
        if (output_meta) {
          if (json.find("meta") == json.end()) json["meta"] = {};
          json["meta"]["elapsed_s"] = solve_s;
          update_meta(json, solver_name);
        }
        if (output_judge) {
          SJudgeResult res = judge(*problem, *out.solution);
          LOG(INFO) << "judge : dislikes = " << res.dislikes;
          LOG(INFO) << "judge : fit_in_hole = " << res.fit_in_hole();
          LOG(INFO) << "judge : satisfy_stretch = " << res.satisfy_stretch();
          LOG(INFO) << "judge : is_valid = " << res.is_valid();
          update_judge(*problem, res, json);
        }
        if (solution_json.empty()) {
          LOG(INFO) << "No output";
        } else {
          std::ofstream ofs(solution_json);
          ofs << json.dump();
          LOG(INFO) << fmt::format("Output   : {}", solution_json);
        }
      }

      if (post_edit) {
        visualize_and_edit(problem, out.solution, solver_name);
      }

      return 0;
    }

    if (sub_list_solvers->parsed()) {
      SolverRegistry::displaySolvers();
      return 0;
    }

    if (sub_try_globalist->parsed()) {
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
        initial_solution = SSolution::load_file(initial_solution_json);
        CHECK(initial_solution);
        CHECK(is_compatible(*problem, *initial_solution));
        LOG(INFO) << fmt::format("Initial Solution  : {}", initial_solution_json);
      }

      std::vector<SolverOutputs> out(2);
      std::vector<SJudgeResult> res(2);
      std::vector<nlohmann::json> out_json(2);
      for (int trial = 0; trial < 2; ++trial) {
        problem->is_globalist_mode = trial == 0;
        LOG(INFO) << fmt::format("Trying GLOBALIST  : {}", problem->is_globalist_mode);

        const auto t0 = std::chrono::system_clock::now();
        out[trial] = solver->solve({ problem, initial_solution, visualize });
        const auto t1 = std::chrono::system_clock::now();
        const double solve_s = std::chrono::duration_cast<std::chrono::microseconds>(t1 - t0).count() * 1e-6;
        LOG(INFO) << fmt::format("Elapsed  : {:.2f} s", solve_s);

        {
          if (problem->is_globalist_mode) {
            out[trial].solution->bonuses.push_back(SBonus(SBonus::Type::GLOBALIST));
          }

          nlohmann::json json = out[trial].solution->json();
          if (output_meta) {
            if (json.find("meta") == json.end()) json["meta"] = {};
            json["meta"]["elapsed_s"] = solve_s;
            update_meta(json, solver_name);
          }
          if (output_judge) {
            res[trial] = judge(*problem, *out[trial].solution);
            LOG(INFO) << "judge : dislikes = " << res[trial].dislikes;
            LOG(INFO) << "judge : fit_in_hole = " << res[trial].fit_in_hole();
            LOG(INFO) << "judge : satisfy_stretch = " << res[trial].satisfy_stretch();
            LOG(INFO) << "judge : is_valid = " << res[trial].is_valid();
            update_judge(*problem, res[trial], json);
          }
          out_json[trial] = json;

          {
            auto path = trial == 0 ? solution_json_base + ".on.nosubmit.pose.json" : solution_json_base + ".off.nosubmit.pose.json";
            std::ofstream ofs(path);
            ofs << json;
            LOG(INFO) << fmt::format("Output   : {}", path);
          }

          if (post_edit) {
            visualize_and_edit(problem, out[trial].solution, solver_name);
          }
        }
      }
      return 0;
    }

    return 0;
}
