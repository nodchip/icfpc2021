#include "stdafx.h"

#include <opencv2/core.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/highgui.hpp>
#include <fmt/format.h>

#include "util.h"
#include "contest_types.h"
#include "solver_registry.h"
#include "judge.h"
#include "visual_editor.h"

class ManualSolver : public SolverBase {
public:
    ManualSolver() { }
    SolverOutputs solve(const SolverArguments &args) override {
        SolverOutputs ret;
        
        SVisualEditorPtr editor = std::make_shared<SVisualEditor>(args.problem, "ManualSolver", "manual");
        if (args.optional_initial_solution) {
            editor->set_pose(args.optional_initial_solution);
        }

        while (true) {
            int c = editor->show(15);
            if (c == 27) {
                break;
            }
        }
        ret.solution = editor->get_pose();

        return ret;
    }
};

REGISTER_SOLVER("ManualSolver", ManualSolver);
// vim:ts=2 sw=2 sts=2 et ci

