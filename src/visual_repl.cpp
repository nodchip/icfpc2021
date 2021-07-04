#include "stdafx.h"
#include <mutex>
#include <nlohmann/json.hpp>
#include "nazoengine.h"
#include "util.h"
#include "picture.h"


int visual_repl(Engine& engine) {
  // see https://github.com/yhirose/cpp-httplib for HTTP server.
  using namespace httplib;

#ifdef _MSC_VER
  std::filesystem::path static_path = "../../visual_repl";
#else
  std::filesystem::path static_path = "../visual_repl";
#endif

  // keep everything as small as possible for lower memory consumption & faster evaluation.
  engine.parse_statement("!quiet");
  engine.parse_statement("!name_partial_off");
  std::mutex mutex_engine;

  Server svr;

  svr.Get("/", [static_path](const Request& req, Response& res) {
    res.set_content(*read_file(static_path / "index.html"), "text/html");
  });

  svr.Get("/static/(.+)", [static_path](const Request& req, Response& res) {
    const char* mime_type = "text/plain";
    if (req.path.find(".css") != std::string::npos) {
      mime_type = "text/css";
    } else if (req.path.find(".js") != std::string::npos) {
      mime_type = "text/javascript";
    }
    auto file_path = static_path / req.path.substr(std::string("/static/").length());
    LOG(INFO) << "static file: " << file_path.string();
    if (auto content = read_file(file_path)) {
      res.set_content(*content, mime_type);
    } else {
      res.status = 404;
    }
  });

  svr.Get("/node_statistics", [&mutex_engine, &engine](const Request& req, Response& res) {
    std::lock_guard lock(mutex_engine);

    nlohmann::json j;
    for (auto [name, count] : engine.node_statistics.get_statistics()) {
      j[name] = count;
    }
    std::ostringstream oss;
    oss << j;
    res.set_content(oss.str(), "application/json");
  });

  // debug.
  svr.Get("/eval", [&mutex_engine, &engine](const Request& req, Response& res) {
    std::lock_guard lock(mutex_engine);

    if (req.has_param("expr")) {
      auto expr = req.get_param_value("expr");
      LOG(INFO) << "got stmt = " << expr;
      auto root = engine.parse_expr(expr);
      root->eval();
      res.set_content(root->repr(true), "text/plain");
    } else {
      res.status = 500;
    }
  });

  svr.Get("/parse_statement", [&mutex_engine, &engine](const Request& req, Response& res) {
    std::lock_guard lock(mutex_engine);

    if (req.has_param("stmt")) {
      auto stmt = strip(req.get_param_value("stmt"));
      LOG(INFO) << "got stmt = " << stmt;
      std::ostringstream oss;
      engine.set_output_stream(oss);
      engine.parse_statement(stmt);
      engine.reset_output_stream();
      res.set_content(oss.str(), "text/plain");
    } else {
      res.status = 500;
    }
  });

  int i = 0;
  engine.parse_statement(fmt::format(":state0 = nil")); // init
  const std::string protocol_name = "galaxy";
  auto click = [&](int x, int y) {
    LOG(INFO) << fmt::format("click({}, {}) -> i => {}", x, y, i + 1);

    engine.parse_statement(fmt::format(":click{} = ap ap cons {} {}", i, x, y));
    engine.parse_statement(fmt::format(":result{} = ap ap ap interact {} :state{} :click{}", i + 1, protocol_name, i, i));
    engine.parse_statement(fmt::format(":state{} = ap car :result{}", i + 1, i + 1));
    engine.parse_statement(fmt::format(":picture{} = ap car ap cdr :result{}", i + 1, i + 1));
    auto picture_node = engine.parse_expr(fmt::format(":picture{}", i + 1));
    picture_node->eval(); // to actually draw the picture.
    engine.parse_statement(fmt::format("!rm :click{} :state{} :result{} :picture{}", i, i, i, i));
    ++i;
  };
  engine.special_commands["!reset_galaxy"] = {
    " .. erase internal state of the galaxy",
    [&](Engine* e, auto args) {
      i = 0;
      engine.parse_statement("!rm :state0");
      engine.parse_statement(":state0 = nil");
      click(0, 0);
    },
  };
  engine.special_commands["!click"] = {
    " <x> <y> .. send a click",
    [&](Engine* e, auto args) {
      if (args.size() == 2) {
        const int x = std::stol(args[0]);
        const int y = std::stol(args[1]);
        click(x, y);
      }
    },
  };
  engine.special_commands["!startup_galaxy"] = {
    ".. send clicks to initiate the galaxy.",
    [&](Engine* e, auto args) {
      std::vector<std::pair<int, int>> coords = {
        {0, 0}, {0, 0}, {0, 0}, {0, 0}, {0, 0}, {0, 0}, {0, 0}, {0, 0},
        {8, 4}, {2, -8}, {3, 6}, {0, -14}, {-4, 10}, {9, -3}, {-4, 10}, {1, 4},
      };
      for (auto [x, y] : coords) {
        click(x, y);
      }
    }
  };

  svr.Get("/click", [&mutex_engine, &engine, &i, click](const Request& req, Response& res) {
    std::lock_guard lock(mutex_engine);

    if (req.has_param("x") && req.has_param("y")) {
      auto sx = req.get_param_value("x");
      auto sy = req.get_param_value("y");
      const int x = std::stol(sx);
      const int y = std::stol(sy);
      LOG(INFO) << fmt::format("click ({}, {})", x, y);

      click(x, y);

      res.set_content(fmt::format("{{\"picture_history_size\": {}}}", Picture::get_history().size()), "application/json");
    } else {
      res.status = 500;
    }
  });

  svr.Get("/picture_size", [&mutex_engine, &engine, &i, click](const Request& req, Response& res) {
    std::lock_guard lock(mutex_engine);
    res.set_content(fmt::format("{{\"picture_history_size\": {}}}", Picture::get_history().size()), "application/json");
  });

  int num_drawn_pictures = 0;
  svr.Post("/picture", [&mutex_engine, &engine, &i, &num_drawn_pictures](const Request& req, Response& res) {
    std::lock_guard lock(mutex_engine);

    auto& picture_history = Picture::get_history();
    const int num_pictures = picture_history.size();

    nlohmann::json jarr = nlohmann::json::array();
    for (int index = num_drawn_pictures; index < num_pictures; ++index) {
      LOG(INFO) << fmt::format("requested picture {} / {}", index, num_pictures);
      jarr.push_back(picture_history[index]->to_json());
    }
    nlohmann::json j = {
      {"picture", jarr},
    };

    std::ostringstream oss;
    oss << j;
    res.set_content(oss.str(), "application/json");

    num_drawn_pictures = num_pictures;
  });

  svr.Get("/stop", [&](const Request& req, Response& res) {
    svr.stop();
  });

  LOG(INFO) << "open http://localhost:3333/";
  svr.listen("localhost", 3333);

  return 0;
}
// vim:ts=2 sw=2 sts=2 et ci

