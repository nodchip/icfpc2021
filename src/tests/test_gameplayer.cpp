#include "stdafx.h"
#include "contest_communication.h"

#include <string>
#include <vector>

#include <gtest/gtest.h>

TEST(GamePlayerTest, parse_staticGameInfoTest) {
  Engine e;
  Scope scope(e);

  // staticGameInfo = (x0, role, x2, x3, x4)
  GameInfo static_game_info;
  parseStaticGameInfo(&static_game_info, scope, scope.eval("( t0 , 1 , t2 , t3 , t4 )"));

  EXPECT_EQ(static_game_info.role, 1);
  EXPECT_EQ(scope.locals[":x0"]->eval()->get_name(), "t0");
  EXPECT_EQ(scope.locals[":x2"]->eval()->get_name(), "t2");
  EXPECT_EQ(scope.locals[":x3"]->eval()->get_name(), "t3");
  EXPECT_EQ(scope.locals[":x4"]->eval()->get_name(), "t4");
}

// vim:ts=2 sw=2 sts=2 et ci

