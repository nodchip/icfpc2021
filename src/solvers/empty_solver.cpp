#include "stdafx.h"

#include <thread>

#include "contest_types.h"
#include "solver_registry.h"

class EmptySolver : public SolverBase {
public:
  EmptySolver() { }
  SolverOutputs solve(const SolverArguments &args) override {
    SolverOutputs ret;

    // do nothing.
    ret.solution = std::make_shared<SSolution>();
    ret.solution->vertices = args.problem->vertices;

    // dummy.
    std::this_thread::sleep_for(std::chrono::milliseconds(500));

    return ret;
  }
};

REGISTER_SOLVER("EmptySolver", EmptySolver);
// vim:ts=2 sw=2 sts=2 et ci

