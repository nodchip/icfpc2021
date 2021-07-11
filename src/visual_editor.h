#include "stdafx.h"
#include "contest_types.h"

struct SCanvas;
using SCanvasPtr = std::shared_ptr<SCanvas>;

struct SMouseParams;
using SMouseParamsPtr = std::shared_ptr<SMouseParams>;

struct SVisualEditor {
    SCanvasPtr canvas;

    const std::string window_name;
    const std::string solver_name;
    SMouseParamsPtr mp;

    int selected_vertex_id;

    SVisualEditor(SProblemPtr problem, const std::string& solver_name, const std::string window_name);
    ~SVisualEditor();

    int get_mouseover_node_id() const;
    static void callback(int e, int x, int y, int f, void* param);

    bool set_pose(SSolutionPtr pose);
    SSolutionPtr get_pose() const;
    int show(int wait);
};
using SVisualEditorPtr = std::shared_ptr<SVisualEditor>;

SSolutionPtr visualize_and_edit(SProblemPtr problem, SSolutionPtr solution, const std::string& base_solver_name);

// vim:ts=2 sw=2 sts=2 et ci

