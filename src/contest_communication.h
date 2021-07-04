#pragma once

namespace httplib {
struct Response;
}

#include <string>
#include <optional>
#include <memory>

#include "nazoengine.h"
#include "gameparams.h"
#include "solver_registry.h"

struct Scope {
  Scope(Engine& engine_) : engine(engine_) {}
  Engine& engine;
  Engine::NamedExpr locals;

  int64_t as_int(Node* expr) {
    number* num = dynamic_cast<number*>(expr);
    CHECK(num);
    return num->value;
  }

  Node* eval(std::string s) {
    auto root = engine.parse_expr_with_locals(s, locals);
    return root->eval();
  }

  int64_t eval_to_int(std::string s) {
    return as_int(eval(s)->eval());
  }
};

// contains all info to play a game (internet / submission)
struct GamePlayer {
 private:
  std::function<std::string(std::string_view)> real_sendrecv_func;
  SolverBase::Ptr active_solver; // will point to either attack_solver or defence_solver.

 public:
  const int64_t player_key;

  // possibly distinct (it is OK if both are the same) solvers for each role.
  SolverBase::Ptr attack_solver;
  SolverBase::Ptr defence_solver;

  // distinct engine for thread safty and environment separation.
  Engine engine;

  // state defined by the protocol.
  // see https://message-from-space.readthedocs.io/en/latest/game.html#
  bool succeeded = true;
  GameInfo staticGameInfo;
  GameState gameState;
  int64_t gameStage = 0;

  GamePlayer(int64_t player_key_, std::function<std::string(std::string_view)> real_sendrecv_func_);

  void set_attack_solver(SolverBase::Ptr attack_solver_) { attack_solver = attack_solver_; }
  void set_defence_solver(SolverBase::Ptr defence_solver_) { defence_solver = defence_solver_; }

  bool game_JOIN();
  bool game_START();
  bool play(std::mutex& lock);

  // internal
  bool check_gameResponse_and_update_internals(Node* gameResponse);
  Node* to_commands(SolverParamOut& solver_result);
};

inline bool parseStaticGameInfo(GameInfo* res, Scope& scope, Node* node) {
  // staticGameInfo = (x0, role, x2, x3, x4)
  scope.locals[":staticGameInfo"] = node;
  scope.locals[":x0"]   = scope.eval("ap car :staticGameInfo");
  scope.locals[":role"] = scope.eval("ap car ap cdr :staticGameInfo")->eval();
  scope.locals[":x2"]   = scope.eval("ap car ap cdr ap cdr :staticGameInfo");
  scope.locals[":x3"]   = scope.eval("ap car ap cdr ap cdr ap cdr :staticGameInfo");
  scope.locals[":x4"]   = scope.eval("ap car ap cdr ap cdr ap cdr ap cdr :staticGameInfo");
  if (res) {
    res->role = scope.eval_to_int(":role");
    return true;
  }
  return false;
}

struct ConnectionInfo {
  std::string serverName;
  int serverPort = 1;
  std::string path;
};
std::optional<ConnectionInfo> parse_url(std::string endpoint_url);

std::shared_ptr<httplib::Response> post_contest_server(std::string endpoint_url, std::string_view text);

std::pair<int64_t, int64_t> create_player_key(Engine& engine);

// vim:ts=2 sw=2 sts=2 et ci
