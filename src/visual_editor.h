#include "stdafx.h"
#include <optional>
#include "contest_types.h"

struct SCanvas;
using SCanvasPtr = std::shared_ptr<SCanvas>;

struct SMouseParams;
using SMouseParamsPtr = std::shared_ptr<SMouseParams>;

struct SShowResult {
  SShowResult(int key) : key(key) {}
  int key = -1;

  operator int() const {
    return key;
  }

  struct SEditResult {
    SSolutionPtr pose_before_edit;
    SSolutionPtr pose_after_edit;
    std::vector<int> moved_vertex_indices;
  };
  std::optional<SEditResult> edit_result;
};

struct SVisualEditor {
    SCanvasPtr canvas;

    const std::string window_name;
    const std::string solver_name;
    SMouseParamsPtr mp;

    int selected_vertex_id;
    SShowResult::SEditResult* edit_info = nullptr;
    bool in_internal_edit_loop() const { return bool(edit_info); }

    SVisualEditor(SProblemPtr problem, const std::string& solver_name, const std::string window_name);
    ~SVisualEditor();

    int get_mouseover_node_id() const;
    static void callback(int e, int x, int y, int f, void* param);

    void set_oneshot_custom_stat(const std::string& stat_str);
    void set_persistent_custom_stat(const std::string& stat_str);

    bool set_pose(SSolutionPtr pose);
    SSolutionPtr get_pose() const;
    SShowResult show(int wait);
};
using SVisualEditorPtr = std::shared_ptr<SVisualEditor>;

SSolutionPtr visualize_and_edit(SProblemPtr problem, SSolutionPtr solution, const std::string& base_solver_name);

// vim:ts=2 sw=2 sts=2 et ci

