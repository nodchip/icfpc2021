#pragma once
#include "contest_types.h"
#include "visual_editor.h"

struct SPinnedIndex {
  SVisualEditorPtr editor;
  std::vector<int> movable_indices;
  std::mt19937& rng;
  const int N;
  SPinnedIndex(std::mt19937& rng, int N, SVisualEditorPtr editor);

  int sample_movable_index() const;

  void update_movable_index();
};

