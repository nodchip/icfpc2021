#include "stdafx.h"

#include <filesystem>
#include <glog/logging.h>
#include <gtest/gtest.h>
#include "judge.h"

namespace fs = std::filesystem;

class JudgeTest : public testing::Test {
 public:
  JudgeTest() {
    root_dir_ = fs::absolute(fs::current_path());
    while (!isProjectRoot(root_dir_)) {
      root_dir_ = root_dir_.parent_path();
    }
    problem_dir_ = problem_dir_;
    problem_dir_ /= "data/problems";
  }

  const fs::path& root_dir() { return root_dir_; }
  const fs::path& problem_dir() { return problem_dir_; }

 private:
  bool isProjectRoot(const fs::path& dir) {
    auto www_path = dir;
    www_path /= "www";
    return fs::exists(www_path);
  }

  fs::path root_dir_;
  fs::path problem_dir_;
};

TEST_F(JudgeTest, TestProblem11_OK) {
  auto filepath = problem_dir();
  filepath /= "11.problem.json";
  SProblemPtr problem = SProblem::load_file(filepath.c_str());
  EXPECT_EQ(problem->vertices.size(), 3);

  SSolution solution;
  solution.vertices = { // copy of problem 11
    {10,0},{10,10},{0,10},
  };

  SJudgeResult res = judge(*problem, solution);

  EXPECT_EQ(res.dislikes, 0);
  EXPECT_TRUE(res.fit_in_hole());
  EXPECT_TRUE(res.satisfy_stretch());
  EXPECT_TRUE(res.is_valid());
}

TEST_F(JudgeTest, TestProblem1_VeryClose) {
  auto filepath = problem_dir();
  filepath /= "1.problem.json";
  SProblemPtr problem = SProblem::load_file(filepath.c_str());
  EXPECT_FALSE(problem->vertices.empty());

  SSolution solution;
  solution.vertices = { // ynasu87 first attempt
    {{21,19},{30,23},{29,72},{34,22},{46,38},{61,62},{31,82},{25,30},{44,26},{36,32},{55,72},{31,23},{46,35},{42,24},{52,45},{44,72},{78,87},{70,93},{41,61},{50,64}}
  };

  SJudgeResult res = judge(*problem, solution);
  //EXPECT_EQ(res.dislikes, 0);
  EXPECT_FALSE(res.fit_in_hole());
  EXPECT_FALSE(res.satisfy_stretch());
  EXPECT_FALSE(res.is_valid());
}

// vim:ts=2 sw=2 sts=2 et ci
