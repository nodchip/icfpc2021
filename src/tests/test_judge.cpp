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
  EXPECT_TRUE(1 == 1);
  EXPECT_EQ(3, 3);

  SProblemPtr problem = SProblem::load_file(DATA_PATH "/problems/11.problem.json");

  SSolution solution;
  solution.vertices = { // copy of problem 11
    {10,10},{0,10},{10,0},
  };

  SJudgeResult res = judge(*problem, solution);

  EXPECT_EQ(res.dislikes, 0);
  EXPECT_TRUE(res.fit_in_hole());
  EXPECT_TRUE(res.satisfy_stretch());
  EXPECT_TRUE(res.is_valid());
}

// vim:ts=2 sw=2 sts=2 et ci
