#include "stdafx.h"

#include <chrono>
#include <CLI/CLI.hpp>

#include "solver_registry.h"
#include "contest_communication.h"
#include "repl.h"
#include "visual_repl.h"
#include "nazoengine.h"

int main(int argc, char* argv[]) {
  google::InitGoogleLogging(argv[0]);
  google::InstallFailureSignalHandler();
  google::SetStderrLogging(google::INFO);
  google::SetLogDestination(google::INFO, "solver.log.");

  std::ios::sync_with_stdio(false);
  std::cin.tie(NULL);

  int return_code = 0;
  
  CLI::App app { "main module" };
  app.require_subcommand(0, 1); // in production mode, there are no subcommands.

  // local: http://ec2-54-150-7-213.ap-northeast-1.compute.amazonaws.com/aliens/send?apiKey=XXXXX
  // submission: http://server:12345 -> internally convert to http://server:12345/aliens/send
  std::string endpoint_url;
  std::string player_key;
  app.add_option("endpoint_url", endpoint_url, "URL provided by the judge system");
  app.add_option("player_key", player_key, "player key provided by the judge system");

  //std::string solver_name = "EmptySolver";
  std::string solver_name = "RotateAttackSolver";

  std::string desc_filename;
  std::string map_filename;
  std::string command_output_filename;
  std::string meta_output_filename;

  auto sub_list_solvers = app.add_subcommand("list_solvers", "list up registered solvers");
  
  auto sub_repl = app.add_subcommand("repl");
  sub_repl->add_option("endpoint_url", endpoint_url, "URL provided by the (internal) judge system");
  sub_repl->add_option("player_key", player_key, "player key provided by the (internal) judge system");

  auto sub_visual_repl = app.add_subcommand("visual_repl");
  sub_visual_repl->add_option("endpoint_url", endpoint_url, "URL provided by the (internal) judge system");
  sub_visual_repl->add_option("player_key", player_key, "player key provided by the (internal) judge system");

  auto sub_run = app.add_subcommand("run");
  SolverParamIn solver_param;
  sub_run->add_option("solver", solver_name, "the solver name");
  sub_run->add_option("endpoint_url", endpoint_url, "URL provided by the (internal) judge system");
  sub_run->add_option("player_key", player_key, "player key provided by the (internal) judge system");

  auto sub_check_command = app.add_subcommand("check_command");
  std::string solution_filename;
  sub_check_command->add_option("solution_file", solution_filename, "input .sol file");

  CLI11_PARSE(app, argc, argv);

  if (!endpoint_url.empty()) {
    if (endpoint_url.find("/aliens/send") == std::string::npos) {
      // submission mode.
      while (endpoint_url.back() == '/')
        endpoint_url.pop_back();
      if (std::count(endpoint_url.begin(), endpoint_url.end(), '/') <= 2)
        endpoint_url += "/aliens/send";
    } else {
      // local mode. apiKey is mandatory.
      if (endpoint_url.find("?apiKey=") == std::string::npos) {
        LOG(ERROR) << "local mode requires apiKey in the endpoint_url.";
        return 1;
      }
    }
  } else {
    LOG(INFO) << "endpoint_url is empty";
  }

// temporalily commented out
#if 0
  // ================== list_solvers
  if (sub_list_solvers->parsed()) {
    std::cout << "== game solvers ==" << std::endl;
    SolverRegistry<SolverFunction>::displaySolvers();
    return 0;
  }

  // ================== run
  if (sub_run->parsed()) {
    // solve the task.
    LOG(INFO) << fmt::format("Solver [{0}] URL:{1} key:{2}", solver_name, endpoint_url, player_key);
    const auto t0 = std::chrono::system_clock::now();
    if (SolverFunction solver = SolverRegistry<SolverFunction>::getSolver(solver_name)) {
      solver(solver_param, []{ /* continue running */ return true; });
    } else {
      LOG(ERROR) << fmt::format("solver [{0}] not found!", solver_name);
    }
    const auto t1 = std::chrono::system_clock::now();
    const double solve_s = std::chrono::duration_cast<std::chrono::microseconds>(t1 - t0).count() * 1e-6;

    LOG(INFO) << fmt::format("Elapsed  : {:.2f} s", solve_s);
    return 0;
  }
#endif

  const bool is_on_submission_server = endpoint_url.find("?apiKey=") == std::string::npos;
  LOG(INFO) << (is_on_submission_server ? "solver in the SUBMISSISON mode!" : "solver in INTERET mode!");

  auto real_sendrecv_func = [endpoint_url, player_key](std::string_view send_text) -> std::string {
    LOG(INFO) << fmt::format("ENDPOINT_URL {}", endpoint_url);
    LOG(INFO) << fmt::format("SEND {}", send_text);
    auto response = post_contest_server(endpoint_url, send_text);
    if (response) {
      LOG(INFO) << fmt::format("RECV {}", response->body);
      return response->body;
    }
    return "00";
  };

  // ================== repl
  if (sub_repl->parsed()) {
    LOG(INFO) << fmt::format("URL=[{0}], player_key=[{1}]", endpoint_url, player_key);

    Engine engine;
    engine.set_sendrecv_func(real_sendrecv_func);
    for (auto text_file : enumerate_text_files({"../data", "../../data"})) {
      engine.parse_statement(fmt::format("!load {}", text_file.string()));
    }

    return repl(engine);
  }

  // ================== visual repl
  if (sub_visual_repl->parsed()) {
    LOG(INFO) << fmt::format("URL=[{0}], player_key=[{1}]", endpoint_url, player_key);

    Engine engine;
    engine.is_gui = true;
    engine.set_sendrecv_func(real_sendrecv_func);
    for (auto text_file : enumerate_text_files({"../data", "../../data"})) {
      engine.parse_statement(fmt::format("!load {}", text_file.string()));
    }

    return visual_repl(engine);
  }

  if (sub_check_command->parsed()) {
    assert (std::filesystem::is_regular_file(solution_filename));
    std::ifstream ifs(solution_filename);
    std::string str((std::istreambuf_iterator<char>(ifs)), std::istreambuf_iterator<char>());
    std::cout << "Command: " << str << std::endl;
    if (true /* TODO */) {
      std::cout << "[O] the command looks OK!" << std::endl;
      return 0;
    } else {
      std::cout << "[X] the command is suspicious!" << std::endl;
      return 1;
    }
  }

  // ======================== otherwise: submit mode

  if (is_on_submission_server) {
    LOG(INFO) << fmt::format("Submit mode.. URL=[{0}], player_key=[{1}]", endpoint_url, player_key);

    if (true) {
      GamePlayer player(std::stoll(player_key), real_sendrecv_func);
      player.set_attack_solver(SolverRegistry::getSolver(solver_name));
      player.set_defence_solver(SolverRegistry::getSolver(solver_name));

      std::mutex joined;
      if (player.play(joined)) {
        LOG(INFO) << "successfully terminating";
      } else {
        LOG(INFO) << "failed";
      }
    } else {
      // manual
      Engine engine;
      engine.set_sendrecv_func(real_sendrecv_func);
      engine.parse_statement("!verbose");
      std::this_thread::sleep_for(std::chrono::seconds(5));
      engine.parse_statement(fmt::format("ap mod ap send ( 2 , {} , nil )", player_key)); // JOIN
      engine.parse_statement(fmt::format("ap mod ap send ( 3 , {} , ( 1 , 2 , 3 , 4 ) )", player_key)); // START
      engine.parse_statement(fmt::format("ap mod ap send ( 4 , {} , ( ( 1 , 0 ) )", player_key)); // COMMANDS: detonate shipId=0
      engine.parse_statement(fmt::format("ap mod ap send ( 4 , {} , ( ( 1 , 1 ) )", player_key)); // COMMANDS: detonate shipId=1
      engine.parse_statement(fmt::format("ap mod ap send ( 4 , {} , ( ( 1 , 2 ) )", player_key)); // COMMANDS: detonate shipId=2
      engine.parse_statement(fmt::format("ap mod ap send ( 4 , {} , ( ( 1 , 3 ) )", player_key)); // COMMANDS: detonate shipId=3
    }

  } else {
    LOG(INFO) << fmt::format("Internet mode.. URL=[{0}], player_key=[{1}]", endpoint_url, player_key);

    Engine engine;
    engine.set_sendrecv_func(real_sendrecv_func);
    auto keys = create_player_key(engine);
    auto attackPlayerKey = keys.first;
    auto defenderPlayerKey = keys.second;

#if 0
    // manual.
    std::thread attacker_thread([&, attackPlayerKey] {
      Engine engine;
      engine.set_sendrecv_func(real_sendrecv_func);
      engine.parse_statement("!verbose");
      std::this_thread::sleep_for(std::chrono::seconds(5));
      engine.parse_statement(fmt::format("ap mod ap send ( 2 , {} , nil )", attackPlayerKey));
      engine.parse_statement(fmt::format("ap mod ap send ( 3 , {} , ( 1 , 1 , 1 , 1 ) )", attackPlayerKey));
        });
    std::thread defender_thread([&, defenderPlayerKey] {
      Engine engine;
      engine.set_sendrecv_func(real_sendrecv_func);
      engine.parse_statement("!verbose");
      engine.parse_statement(fmt::format("ap mod ap send ( 2 , {} , nil )", defenderPlayerKey));
      engine.parse_statement(fmt::format("ap mod ap send ( 3 , {} , ( 1 , 1 , 1 , 1 ) )", defenderPlayerKey));
        });
    std::this_thread::sleep_for(std::chrono::seconds(5));
    defender_thread.join();
    attacker_thread.join();
    //std::this_thread::sleep_for(std::chrono::seconds(5));
    //engine.parse_statement(fmt::format("ap send ( 3 , {} , ( 1 , 2 , 3 , 4 ) )", attackPlayerKey));
    //std::this_thread::sleep_for(std::chrono::seconds(5));
    //engine.parse_statement(fmt::format("ap send ( 3 , {} , ( 1 , 2 , 3 , 4 ) )", defenderPlayerKey));
    //std::this_thread::sleep_for(std::chrono::seconds(5));
#else
    GamePlayer attack_player(attackPlayerKey, real_sendrecv_func);
    attack_player.set_attack_solver(SolverRegistry::getSolver(solver_name));
    attack_player.set_defence_solver(SolverRegistry::getSolver(solver_name));

    GamePlayer defender_player(defenderPlayerKey, real_sendrecv_func);
    defender_player.set_attack_solver(SolverRegistry::getSolver(solver_name));
    defender_player.set_defence_solver(SolverRegistry::getSolver(solver_name));

    std::mutex joined0;
    std::mutex joined1;
    std::thread attacker_thread([&] {
      LOG(INFO) << "[ATTACKER] start";
      //std::this_thread::sleep_for(std::chrono::seconds(3));
      attack_player.play(joined0);
      LOG(INFO) << "[ATTACKER] finish";
    });
    std::thread defender_thread([&] {
      LOG(INFO) << "[DEFENDER] start";
      //std::this_thread::sleep_for(std::chrono::seconds(1));
      defender_player.play(joined1);
      LOG(INFO) << "[DEFENDER] finish";
    });
    defender_thread.join();
    attacker_thread.join();
#endif

    LOG(INFO) << "successfully terminating";
  }

  return return_code;
}

// vim:ts=2 sw=2 sts=2 et ci

