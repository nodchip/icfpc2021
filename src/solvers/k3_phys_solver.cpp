#include "stdafx.h"

#include <thread>
#include "fmt/format.h"
#include "util.h"

#include "contest_types.h"
#include "solver_registry.h"
#include "solver_util.h"
#include "visual_editor.h"
#include "judge.h"
#include "layout_editor.h"
#include "visual_editor.h"



class K3PhysSolver : public SolverBase {
public:
  using Xorshift = NLayoutEditor::Xorshift;
  K3PhysSolver() {}

  using Pose = std::vector<Point>;

  SolverOutputs solve(const SolverArguments& args) override {
    using namespace NLayoutEditor;

    auto pose = (args.optional_initial_solution ? args.optional_initial_solution->vertices : args.problem->vertices);

    SLayoutEditor layout_editor(args.problem, "K3PhysSolver", "visualize");

    // do nothing.
    SolverOutputs ret;
    ret.solution = layout_editor.force_directed_layout();

    return ret;
  }

};

REGISTER_SOLVER("K3PhysSolver", K3PhysSolver);
// vim:ts=2 sw=2 sts=2 et ci

