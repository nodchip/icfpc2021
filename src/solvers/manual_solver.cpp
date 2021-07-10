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

struct SManualSolver {
    SVisualEditorPtr editor;

    SManualSolver(SProblemPtr problem, const std::string window_name = "manual")
      : editor(std::make_shared<SVisualEditor>(problem, window_name)) {
    }

    static void callback(int e, int x, int y, int f, void* param) {
        SManualSolver* s = static_cast<SManualSolver*>(param);
        s->callback(e, x, y, f, param);
    }

    SSolutionPtr solve() {
        while (true) {
            int c = editor->show(15);
            if (c == 27) {
                return editor->get_pose();
            }
            //if (c == 'd') {
            //    editor->canvas->draw_distant_hole_vertex = !editor->canvas->draw_distant_hole_vertex;
            //    editor->canvas->update(-1);
            //}
            //cv::imshow(editor->window_name, editor->canvas->img);
        }
        return nullptr;
    }
};
using SManualSolverPtr = std::shared_ptr<SManualSolver>;

class ManualSolver : public SolverBase {
public:
    ManualSolver() { }
    SolverOutputs solve(const SolverArguments &args) override {
        SolverOutputs ret;

        SManualSolverPtr manual_solver = std::make_shared<SManualSolver>(args.problem);

        ret.solution = manual_solver->solve();
        return ret;
    }
};

REGISTER_SOLVER("ManualSolver", ManualSolver);
// vim:ts=2 sw=2 sts=2 et ci

