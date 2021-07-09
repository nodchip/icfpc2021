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

  SProblemPtr problem;
  {
    std::ifstream input_data_ifs(problem_json);
    nlohmann::json j;
    input_data_ifs >> j;
    problem = std::make_shared<SProblem>(j);
  }

  SSolution solution;
  // TODO: load solution
  if (false) {
    solution.vertices = { // copy of problem 1
      {20,30},{20,40},{30,95},{40,15},{40,35},{40,65},{40,95},{45,5},{45,25},{50,15},{50,70},{55,5},{55,25},{60,15},{60,35},{60,65},{60,95},{70,95},{80,30},{80,40} 
    };
  }
  if (true) {
    solution.vertices = { // copy of problem 11
      {10,10},{0,10},{10,0},
    };
  }

  SJudgeResult res = judge(*problem, solution);
  std::cout << "dislikes = " << res.dislikes << std::endl;
  std::cout << "fit_in_hole = " << res.fit_in_hole() << std::endl;
  std::cout << "satisfy_stretch = " << res.satisfy_stretch() << std::endl;
  std::cout << "is_valid = " << res.is_valid() << std::endl;

  // TODO: save solution file with these meta info.

  return 0;
}
