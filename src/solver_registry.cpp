#include "stdafx.h"
#include "solver_registry.h"
#include <iostream>

void SolverRegistry::displaySolvers() {
  for (auto it = getRegistry().begin(); it != getRegistry().end(); ++it) {
    std::cout << "solver: " << it->first << " @ " << it->second.file_name << std::endl;
  }
}

// vim:ts=2 sw=2 sts=2 et ci
