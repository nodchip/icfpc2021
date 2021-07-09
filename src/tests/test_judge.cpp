#include "stdafx.h"

#include <glog/logging.h>
#include <gtest/gtest.h>
#include "judge.h"

#ifdef _MSC_VER
#define DATA_PATH "../../data"
#else
#define DATA_PATH "../data"
#endif

TEST(JudgeTest, TestProblem11_OK) {
  SProblemPtr problem = SProblem::load_file(DATA_PATH "/problems/11.problem.json");
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

TEST(JudgeTest, TestProblem1_VeryClose) {
  SProblemPtr problem = SProblem::load_file(DATA_PATH "/problems/1.problem.json");
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
