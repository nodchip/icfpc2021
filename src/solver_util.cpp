#include "stdafx.h"
#include "solver_util.h"
#include "contest_types.h"

SPinnedIndex::SPinnedIndex(std::mt19937& rng, int N, SVisualEditorPtr editor) : editor(editor), rng(rng), N(N) {
  update_movable_index();
}

int SPinnedIndex::sample_movable_index() const {
  return movable_indices[std::uniform_int_distribution<int>(0, movable_indices.size() - 1)(rng)];
};

void SPinnedIndex::update_movable_index() {
  movable_indices.assign(N, 0);
  std::iota(movable_indices.begin(), movable_indices.end(), 0);
  if (editor) {
    for (auto i : editor->get_marked_indices()) {
      movable_indices[i] = -1;
    }
    movable_indices.erase(std::remove(movable_indices.begin(), movable_indices.end(), -1), movable_indices.end());
  }
}

