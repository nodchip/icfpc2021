#include "stdafx.h"
#include "contest_communication.h"
#include "solver_registry.h"
#include "repl.h"
#include "visual_repl.h"
#include "nazoengine.h"
#include <sstream>

std::optional<ConnectionInfo> parse_url(std::string endpoint_url) {
  const std::regex urlRegexp("https?://([^/:]+)(:(\\d+))?(/.+)");
  std::smatch urlMatches;
  if (!std::regex_search(endpoint_url, urlMatches, urlRegexp)) {
    LOG(ERROR) << "Unexpected server response:\nBad server URL" << std::endl;
    return std::nullopt;
  }
  ConnectionInfo conn;
  conn.serverName = urlMatches[1];
  if (urlMatches[3].matched) {
    conn.serverPort = std::stol(urlMatches[3]);
  } else {
    LOG(ERROR) << "NO PORT!";
    conn.serverPort = 80; // default
  }
  conn.path = urlMatches[4];

  LOG(INFO) << fmt::format("parsed (serverName = {} port = {} path = {})", conn.serverName, conn.serverPort, conn.path);
  return conn;
}

std::shared_ptr<httplib::Response> post_contest_server(std::string endpoint_url, std::string_view text) {
  auto conn = parse_url(endpoint_url);
  if (!conn) {
    LOG(ERROR) << "failed to parse URL";
    return nullptr;
  }

#ifdef CPPHTTPLIB_OPENSSL_SUPPORT
  httplib::SSLClient client(conn->serverName, conn->serverPort);
  client.enable_server_certificate_verification(false);
#else
  httplib::Client client(conn->serverName, conn->serverPort);
#endif
  // long timeout.
  client.set_read_timeout(120, 0);
  client.set_write_timeout(120, 0);

  const std::shared_ptr<httplib::Response> serverResponse = client.Post(conn->path.c_str(), text.data(), "text/plain");
  if (!serverResponse) {
    LOG(ERROR) << "Unexpected server response:\nNo response from server" << std::endl;
    return nullptr;
  }
  
  if (serverResponse->status == 302) {
    LOG(ERROR) << "HTTP STATUS = 302";
    if (serverResponse->has_header("Location")) {
      std::string redirect_path = serverResponse->get_header_value("Location");
      std::string redirect_url = fmt::format("http://{}:{}{}", conn->serverName, conn->serverPort, redirect_path);
      LOG(ERROR) << "REDIRECT Location = " << redirect_path << " URL = " << redirect_url;
      return post_contest_server(redirect_url, text);
    } else {
      LOG(ERROR) << "no Location found";
    }
  } else if (serverResponse->status != 200) {
    LOG(ERROR) << "Unexpected server response:"
               << "\nHTTP code: " << serverResponse->status
               << "\nResponse body: " << serverResponse->body << std::endl;
    return nullptr;
  }

  return serverResponse;
}

Node* get_list_item(Node* n, int index) {
  std::string s = "ap car";
  for (int i = 0; i < index; ++i) {
    s += " ap cdr";
  }
  s += " TMP";
  return n->engine->parse_expr_with_locals(s, {{"TMP", n}})->eval();
}

static int64_t as_int(Node* expr) {
  number* num = dynamic_cast<number*>(expr);
  CHECK(num);
  return num->value;
}

static int64_t eval_expr_to_int(Engine& e, std::string s) {
  Node* expr = e.parse_expr(s)->eval();
  CHECK(expr);
  return as_int(expr);
}
static int64_t eval_expr_to_int(Engine& e, std::string s, const Engine::NamedExpr& local_nodes) {
  Node* expr = e.parse_expr_with_locals(s, local_nodes)->eval();
  CHECK(expr);
  return as_int(expr);
}

std::pair<int64_t, int64_t> create_player_key(Engine& engine) {
  // CREATE
  engine.parse_statement(":createResponse = ap send ( 1 , 0 )");
  // (1, ((0, attackPlayerKey), (1, defenderPlayerKey)))
  auto ls = engine.parse_expr(":createResponse");
  const int64_t one0              = as_int(get_list_item(ls, 0));
  auto ls_second                  = get_list_item(ls, 1);
  const int64_t zero0             = as_int(get_list_item(get_list_item(ls_second, 0), 0));
  const int64_t attackPlayerKey   = as_int(get_list_item(get_list_item(ls_second, 0), 1));
  const int64_t one1              = as_int(get_list_item(get_list_item(ls_second, 1), 0));
  const int64_t defenderPlayerKey = as_int(get_list_item(get_list_item(ls_second, 1), 1));
  CHECK(one0 == 1);
  CHECK(zero0 == 0);
  CHECK(one1 == 1);
  LOG(INFO) << fmt::format("attackPlayerKey = {}, defenderPlayerKey = {}", attackPlayerKey, defenderPlayerKey);
  return {attackPlayerKey, defenderPlayerKey};
}

#define DEBUGSYMBOL(x) LOG(INFO) << " *********** " << player_key << " : " << __LINE__ << " " << x << " => " << scope.locals[x]->repr(true)

GamePlayer::GamePlayer(int64_t player_key_, std::function<std::string(std::string_view)> real_sendrecv_func_)
  : real_sendrecv_func(real_sendrecv_func_)
  , player_key(player_key_) {
  engine.set_sendrecv_func(real_sendrecv_func);
  if (false) {
    // do not need GALAXY?
    for (auto text_file : enumerate_text_files({"../data", "../../data"})) {
      engine.parse_statement(fmt::format("!load {}", text_file.string()));
    }
  }
  //engine.parse_statement("!verbose");
}

bool GamePlayer::game_JOIN() {
  Scope scope(engine);
  // JOIN
  // (2, playerKey, (...unknown list...))
  scope.locals[":playerKey"] = new number(&engine, player_key);
  scope.locals[":joinRequest"] = scope.eval("( 2 , :playerKey , nil )");
  scope.locals[":JOIN_gameResponse"] = scope.eval("ap send :joinRequest")->eval();
  DEBUGSYMBOL(":JOIN_gameResponse");
  check_gameResponse_and_update_internals(scope.locals[":JOIN_gameResponse"]);
  if (!succeeded) {
    LOG(ERROR) << "JOIN failed";
    return false;
  }
  LOG(ERROR) << "JOIN succeeded";
  return true;
}

bool GamePlayer::game_START() {
  Scope scope(engine);
  // START
  // (3, playerKey, (x0, x1, x2, x3))
  scope.locals[":playerKey"] = new number(&engine, player_key);
  scope.locals[":x0"] = scope.eval("1");
  scope.locals[":x1"] = scope.eval("1");
  scope.locals[":x2"] = scope.eval("1");
  scope.locals[":x3"] = scope.eval("1");
  scope.locals[":startRequest"] = scope.eval("( 3 , :playerKey , ( :x0 , :x1 , :x2 , :x3 ) )");
  scope.locals[":START_gameResponse"] = scope.eval("ap send :startRequest");
  check_gameResponse_and_update_internals(scope.locals[":START_gameResponse"]);
  if (!succeeded) {
    LOG(ERROR) << "START failed";
    return false;
  }
  LOG(ERROR) << "START succeeded";
  return true;
}


bool GamePlayer::check_gameResponse_and_update_internals(Node* gameResponse) {
  LOG(INFO) << "gameResponse : " << gameResponse->repr(true);
  // example
  // :gameResponse = ( success=1 ,
  //                   gameStage=0 ,
  //                   ( x0=384 , role=1 , x2=( 448 , 1 , 64 ) , x3=( 16 , 128 ) , nil ) ,
  //                   gameState=nil )

  // START:
  // ( 1 , 1 ,
  //   ( 384 , 0 , ( 512 , 1 , 64 ) , ( 16 , 128 ) , ( 1 , 1 , 1 , 1 ) ) ,
  //   ( 0 , ( 16 , 128 ) , ( ( ( 1 , 0 , [33,48] , [0,0] , ( 1 , 1 , 1 , 1 ) , 0 , 64 , 1 ) , nil ) , ( ( 0 , 1 , [-33,-48] , [0,0] , ( 1 , 1 , 1 , 1 ) , 0 , 64 , 1 ) , nil ) ) ) )

  Scope scope(engine);
  scope.locals[":gameResponse"] = gameResponse;

  // if failed, ( 0 ). check by cdr is nil.
  const int64_t idx = scope.eval_to_int("ap car :gameResponse"); // :gameResponse[0]
  if (idx == 0) {
    LOG(INFO) << "FAILED!";
    succeeded = false;
    return false;
  }
  if (idx == 1) {
    succeeded = true;

    // :gameResponse => (1, gameStage, staticGameInfo, gameState)
    scope.locals[":gameStage"] = scope.eval("ap car ap cdr :gameResponse"); // :gameResponse[1]
    DEBUGSYMBOL(":gameStage");
    gameStage = scope.eval_to_int(":gameStage");

    // staticGameInfo => (x0, role, x2, x3, x4)
    CHECK(parseStaticGameInfo(&staticGameInfo, scope, scope.eval("ap car ap cdr ap cdr :gameResponse"))); // :gameResponse[2]
    LOG(INFO) << player_key << " : " << __LINE__;

    if (gameStage != 1) {
        LOG(INFO) << player_key << " : " << __LINE__ << " skip :gameState check";
    } else {
      // gameState => (gameTick, x1, shipsAndCommands)
      // nilのこともある？
      GameState gameState;
      scope.locals[":gameState"]        = scope.eval("ap car ap cdr ap cdr ap cdr :gameResponse"); // :gameResponse[3]
      if (scope.locals[":gameState"]->eval()->get_name() == "nil") {
        LOG(INFO) << player_key << " : " << __LINE__ << " :gameState is nil";
      } else {
        DEBUGSYMBOL(":gameState");
        scope.locals[":gameTick"]         = scope.eval("ap car :gameState");
        scope.locals[":x1"]               = scope.eval("ap car ap cdr :gameState");
        scope.locals[":shipAndCommands"]  = scope.eval("ap car ap cdr ap cdr :gameState");
        gameState.gameTick = scope.eval_to_int(":gameTick");
        // ignore x1

        // TODO: parse :shipsAndCommands
        // shipsAndCommands is a list of items, each item has a structure of (ship, appliedCommands)
        gameState.shipAndCommands.clear();
        Node* work = scope.locals[":work"] = scope.locals[":shipAndCommands"];
        LOG(INFO) << player_key << " : " << __LINE__;
        LOG(INFO) << player_key << " : work = " << work->repr(true);

        while (scope.eval("ap ap ap isnil :work 0 1")) { // if not empty
          LOG(INFO) << player_key << " : " << __LINE__;
          LOG(INFO) << player_key << " : work = " << work->repr(true);
          // (ship, appliedCommands) <= first item of :work.
          Node* ship = scope.locals[":ship"] = scope.eval("ap car ap car :work");
          Node* appliedCommands = scope.locals[":appliedCommands"] = scope.eval("ap car ap cdr ap car :work");
          LOG(INFO) << player_key << " : " << __LINE__;
          LOG(INFO) << player_key << " : ship = " << ship->repr(true);

          ShipAndCommands shipAndCommands;

          // parse :ship
          shipAndCommands.ship.role = scope.eval_to_int("ap car :ship");
          shipAndCommands.ship.shipId = scope.eval_to_int("ap car ap cdr :ship");
          shipAndCommands.ship.position = {
            scope.eval_to_int("ap car ap car ap cdr ap cdr :ship"),
            scope.eval_to_int("ap cdr ap car ap cdr ap cdr :ship")
          };
          shipAndCommands.ship.velocity = {
            scope.eval_to_int("ap car ap car ap cdr ap cdr ap cdr :ship"),
            scope.eval_to_int("ap cdr ap car ap cdr ap cdr ap cdr :ship"),
          };
          scope.locals[":ship_x4"] = scope.eval("ap car ap cdr ap cdr ap cdr ap cdr :ship");
          scope.locals[":ship_x5"] = scope.eval("ap car ap cdr ap cdr ap cdr ap cdr ap cdr :ship");
          scope.locals[":ship_x6"] = scope.eval("ap car ap cdr ap cdr ap cdr ap cdr ap cdr ap cdr :ship");
          scope.locals[":ship_x7"] = scope.eval("ap car ap cdr ap cdr ap cdr ap cdr ap cdr ap cdr ap cdr :ship");
          LOG(INFO) << player_key << " : " << __LINE__;

          // TODO: parse :appliedCommands

          gameState.shipAndCommands.push_back(shipAndCommands);

          // tail
          scope.locals[":work"] = scope.eval("ap cdr :work");
          LOG(INFO) << player_key << " : " << __LINE__;
        }
      }
    }
  }
  return true;
}

Node* GamePlayer::to_commands(SolverParamOut& solver_result) {
  std::ostringstream ss;
  ss << "( ";
  bool first = true;
  for (auto const &command : solver_result.appliedCommands) {
    if (!first) {
      ss << ", ";
      first = false;
    }
    ss << "( " << command->type << " , " << command->shipId << " ";
    switch (command->type) {
      case 0: {
        std::shared_ptr<AccelerateCommand> ac = std::dynamic_pointer_cast<AccelerateCommand>(command);
        ss << "ap ap vec " << ac->vector.x << " " << ac->vector.y << " ";
        break;
      }
      case 1: {
        break;
      }
      case 2: {
        std::shared_ptr<ShootCommand> sc = std::dynamic_pointer_cast<ShootCommand>(command);
        ss << "ap ap vec " << sc->target.x << " " << sc->target.y << " , " << sc->x3 << " ";
        break;
      }
    }
    ss << ") ";
  }
  ss << ")";
  Scope scope(engine);
  auto result = scope.eval(ss.str());
  return result;
}

bool GamePlayer::play(std::mutex& joined) {
  LOG(INFO) << player_key << " : WAIT JOIN..";
  {
    std::scoped_lock lock(joined);
    LOG(INFO) << player_key << " : STARTING JOIN..";
    if (!game_JOIN()) return false;
    LOG(INFO) << player_key << " : JOIN FINISHED..";
  
    if (staticGameInfo.role == 0) {
      CHECK(attack_solver);
      active_solver = attack_solver;
    } else {
      CHECK(defence_solver);
      active_solver = defence_solver;
    }

    LOG(INFO) << player_key << " : STARTING START..";
    if (!game_START()) return false;
    LOG(INFO) << player_key << " : START FINISHED..";
  }

  Scope scope(engine);

  while (gameStage != 2) {

    SolverParamIn solver_param_in; // TODO.
    solver_param_in.gameinfo.role = staticGameInfo.role;
    solver_param_in.gamestate = gameState;

    SolverParamOut solver_param_out = active_solver->next(solver_param_in);
    Node* commands = to_commands(solver_param_out);
    LOG(INFO) << player_key << " commands -- " << commands->repr(true);
    
    // COMMANDS
    scope.locals[":playerKey"] = new number(&engine, player_key);
    scope.locals[":commands"] = commands;
    scope.locals[":commandsRequest"] = scope.eval("( 4 , :playerKey, :commands )");
    scope.locals[":COMMANDS_gameResponse"] = scope.eval("ap send :commandsRequest");
    check_gameResponse_and_update_internals(scope.locals[":COMMANDS_gameResponse"]);
    if (!succeeded) {
      LOG(ERROR) << "COMMANDS failed";
      return 1;
    }

    break;
  }
  return 0;
}

// vim:ts=2 sw=2 sts=2 et ci
