#pragma once
#include <string>
#include <functional>
#include <map>
#include <optional>
#include <cassert>

#include "contest_types.h"

struct SolverArguments {
  SolverArguments(SProblemPtr problem) : problem(problem) {}

  SProblemPtr problem;
  SSolutionPtr optional_initial_solution;

  bool visualize = false;
  std::optional<double> timeout_s;

  /// <summary>
  /// パラメーターファイルのパス。OptunaAnnealingSolverのみで使用する。
  /// </summary>
  std::string parameters_file_path;

  std::optional<int64_t> num_iters;
  std::optional<unsigned int> random_seed;
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
  static SolverBase::Ptr getSolver(std::string name);
  static std::string getCanonicalSolverName(std::string name);

  SolverRegistry(std::string name, SolverEntry entry) {
    getRegistry()[name] = entry;
  }

  static void displaySolvers();
};

SolverOutputs solve_with(const std::string& solver_name, SProblemPtr problem, SSolutionPtr initial_solution);

// vim:ts=2 sw=2 sts=2 et ci
