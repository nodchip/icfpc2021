#pragma once
#include <vector>
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

struct SZobristHash {
  using key_t = uint64_t;
  std::vector<key_t> table; 
  SZobristHash(size_t N) : table(N, key_t(0)) {
    std::mt19937_64 rng;
    for (size_t i = 0; i < N; ++i) {
      table[i] = std::uniform_int_distribution<key_t>(0, std::numeric_limits<key_t>::max())(rng);
    }
  }
  key_t operator[](size_t value) const { return table[value]; }
};

template <typename T>
bool chmin(T& accum_min, T test) {
  if (test < accum_min) {
    accum_min = test;
    return true;
  }
  return false;
}

template <typename T>
bool chmax(T& accum_max, T test) {
  if (test > accum_max) {
    accum_max = test;
    return true;
  }
  return false;
}

void save_solution(SProblemPtr problem, SSolutionPtr solution, const std::string& solver_name, const std::string& file_path);
