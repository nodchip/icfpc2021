#pragma once
#include <string>
#include <functional>
#include <map>
#include <optional>
#include <cassert>

#include "contest_types.h"

struct SolverArguments {
  SProblemPtr problem;
  SSolutionPtr optional_initial_solution;

  bool visualize = false;

  /// <summary>
  /// パラメーターファイルのパス。OptunaAnnealingSolverのみで使用する。
  /// </summary>
  std::string parameters_file_path;
};

struct SolverOutputs {
  SSolutionPtr solution;
};

class SolverBase {
public:
  using Ptr = std::shared_ptr<SolverBase>;

  virtual ~SolverBase() {}
  virtual SolverOutputs solve(const SolverArguments &paramin) = 0;
};

#define CONCAT_SUB(a, b) a##b
#define CONCAT(a, b) CONCAT_SUB(a, b)
#define REGISTER_SOLVER(name, cls) \
  static SolverRegistry CONCAT(_register_solver_, __LINE__) = {name, {__FILE__, [] { return std::make_shared<cls>(); } }}

struct SolverRegistry {
  struct SolverEntry {
    std::string file_name;
    std::function<SolverBase::Ptr(void)> factory;
  };
  static std::map<std::string, SolverEntry>& getRegistry() {
    static std::map<std::string, SolverEntry> s_solver_registry;
    return s_solver_registry;
  }
  static SolverBase::Ptr getSolver(std::string name) {
    auto reg = getRegistry();
    auto it = reg.find(name);
    if (it == reg.end()) {
      return {};
    }
    return it->second.factory();
  }

  SolverRegistry(std::string name, SolverEntry entry) {
    getRegistry()[name] = entry;
  }

  static void displaySolvers();
};

// vim:ts=2 sw=2 sts=2 et ci
