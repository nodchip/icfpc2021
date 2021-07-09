#include "stdafx.h"

#include <iostream>
#include <glog/logging.h>
#include <CLI/CLI.hpp>
#include "judge.h"

int main(int argc, char* argv[]) {
  google::InitGoogleLogging(argv[0]);
  google::InstallFailureSignalHandler();
  google::SetStderrLogging(google::INFO);
  google::SetLogDestination(google::INFO, "judge.log.");

  std::ios::sync_with_stdio(false);
  std::cin.tie(NULL);

  CLI::App app { "judge module" };

  std::string problem_json;
  std::string solution_json;
  bool output_judge = true;
  app.add_option("problem_json", problem_json, "problem JSON file path");
  app.add_option("solution_json", solution_json, "solution JSON file path");
  app.add_flag("-j,--output-judge,!--no-output-judge", output_judge, "output judge info to solution JSON");

  CLI11_PARSE(app, argc, argv);

  SProblemPtr problem = SProblem::load_file_ext(problem_json);
  CHECK(problem);
  SSolutionPtr solution = SSolution::load_file(solution_json);
  CHECK(solution);
  CHECK(is_compatible(*problem, *solution));

  SJudgeResult res = judge(*problem, *solution);
  LOG(INFO) << "judge : dislikes = " << res.dislikes;
  LOG(INFO) << "judge : fit_in_hole = " << res.fit_in_hole();
  LOG(INFO) << "judge : satisfy_stretch = " << res.satisfy_stretch();
  LOG(INFO) << "judge : is_valid = " << res.is_valid();

  if (output_judge) {
    nlohmann::json json;
    {
      std::ifstream ifs(solution_json);
      ifs >> json;
    }
    update_judge(res, json);
    {
      std::ofstream ofs(solution_json);
      ofs << json;
    }
  }

  return 0;
}
