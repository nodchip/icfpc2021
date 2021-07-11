#include "stdafx.h"
#include "solver_registry.h"
#include <iostream>
#include <fmt/format.h>
#include "timer.h"

SolverBase::Ptr SolverRegistry::getSolver(std::string name) {
  auto reg = getRegistry();
  auto it = reg.find(name);
  if (it != reg.end()) return it->second.factory();
  return {};
}

std::string SolverRegistry::getCanonicalSolverName(std::string name) {
  auto reg = getRegistry();
  {
    auto it = reg.find(name);
    if (it != reg.end()) return name;
  }
  {
    auto it = reg.find(name + "Solver");
    if (it != reg.end()) {
      LOG(INFO) << fmt::format("requested [{}] -> found [{}Solver]", name, name);
      return name + "Solver";
    }
  }
  return name;
}

void SolverRegistry::displaySolvers() {
  for (auto it = getRegistry().begin(); it != getRegistry().end(); ++it) {
    std::cout << "solver: " << it->first << " @ " << it->second.file_name << std::endl;
  }
}

SolverOutputs solve_with(const std::string& solver_name, SProblemPtr problem, SSolutionPtr initial_solution) {
  CHECK(problem);
  CHECK(initial_solution);
  auto solver = SolverRegistry::getSolver(solver_name);
  CHECK(solver);
  SolverArguments args { problem, initial_solution, false /* visualize */ };
  LOG(INFO) << fmt::format("solve_with({})..", solver_name);
  Timer t;
  SolverOutputs out = solver->solve(args);
  LOG(INFO) << fmt::format("solve_with({}).. elapsed {} ms", solver_name, t.elapsed_ms());
  return out;
}

// vim:ts=2 sw=2 sts=2 et ci
