#include "stdafx.h"

#include "solver_registry.h"

class EmptySolver : public SolverBase {
public:
  EmptySolver() { }
  SolverParamOut next(const SolverParamIn &gameinfo) override {
    SolverParamOut ret;

    // just detonate!
    ret.appliedCommands.push_back(
      std::make_shared<DetonateCommand>());

    return ret;
  }
};

REGISTER_SOLVER("EmptySolver", EmptySolver);
// vim:ts=2 sw=2 sts=2 et ci

