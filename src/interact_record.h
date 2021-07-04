#pragma once
#include <filesystem>
#include <fstream>
#include <vector>
//#include "picture.h"

struct InteractionSnapshot {
  int x = 0;
  int y = 0;
  //Picture picture; // TODO
};

struct InteractRecorder {
  InteractRecorder();
  ~InteractRecorder();
  InteractRecorder(std::filesystem::path file_path);

  void record(int x, int y);

  std::vector<InteractionSnapshot> snapshots;
  std::ofstream ofs;
};
// vim:ts=2 sw=2 sts=2 et ci

