#include "stdafx.h"
#include "solver_registry.h"
#include <iostream>
#include <fmt/format.h>

SolverBase::Ptr SolverRegistry::getSolver(std::string name) {
  auto reg = getRegistry();
  {
    auto it = reg.find(name);
    if (it != reg.end()) return it->second.factory();
  }
  {
    auto it = reg.find(name + "Solver");
    if (it != reg.end()) {
      LOG(INFO) << fmt::format("requested [{}] -> found [{}Solver]", name, name);
      return it->second.factory();
    }
  }
  return {};
}

void SolverRegistry::displaySolvers() {
  for (auto it = getRegistry().begin(); it != getRegistry().end(); ++it) {
    std::cout << "solver: " << it->first << " @ " << it->second.file_name << std::endl;
  }
}

// vim:ts=2 sw=2 sts=2 et ci
