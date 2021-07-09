#include "stdafx.h"

#include <iostream>
#include <glog/logging.h>
#include <CLI/CLI.hpp>
#include "judge.h"

int main(int argc, char* argv[]) {
  google::InitGoogleLogging(argv[0]);
  google::InstallFailureSignalHandler();
  google::SetStderrLogging(google::INFO);
  // TODO: Rename log file name.
  google::SetLogDestination(google::INFO, "main.log.");

  std::ios::sync_with_stdio(false);
  std::cin.tie(NULL);

  int return_code = 0;
  
  CLI::App app { "judge module" };

  std::string problem_json;
  std::string solution_json;
  app.add_option("problem_json", problem_json, "problem JSON file path");
  app.add_option("solution_json", solution_json, "solution JSON file path");

  CLI11_PARSE(app, argc, argv);

  SProblem problem;
  // TODO: load problem

  SSolution solution;
  // TODO: load solution

  SJudgeResult res = judge(problem, solution);
  std::cout << "dislikes = " << res.dislikes;
  std::cout << "fit_in_hole = " << res.fit_in_hole();
  std::cout << "satisfy_stretch = " << res.satisfy_stretch();
  std::cout << "is_valid = " << res.is_valid();

  // TODO: save solution file with these meta info.

  return 0;
}
