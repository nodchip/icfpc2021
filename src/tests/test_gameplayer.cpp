#include "stdafx.h"
#include "contest_communication.h"

#include <string>
#include <vector>

#include <gtest/gtest.h>

TEST(GamePlayerTest, parse_url_submission) {
  auto conn = parse_url("http://server:12345/aliens/send");
  EXPECT_EQ(conn->serverName, "server");
  EXPECT_EQ(conn->serverPort, 12345);
  EXPECT_EQ(conn->path, "/aliens/send");
}

TEST(GamePlayerTest, parse_url_proxy) {
  auto conn = parse_url("http://ec2-54-150-7-213.ap-northeast-1.compute.amazonaws.com/aliens/send?apiKey=0ca8c871f4774f0fae6f4ff7f8a410c9");
  EXPECT_EQ(conn->serverName, "ec2-54-150-7-213.ap-northeast-1.compute.amazonaws.com");
  EXPECT_EQ(conn->serverPort, 80);
  EXPECT_EQ(conn->path, "/aliens/send?apiKey=0ca8c871f4774f0fae6f4ff7f8a410c9");
}

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

