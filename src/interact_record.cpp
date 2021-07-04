#include "stdafx.h"
#include <filesystem>
#include <vector>
#include <fmt/format.h>
#include "picture.h"
#include "interact_record.h"

InteractRecorder::InteractRecorder() {
}

InteractRecorder::~InteractRecorder() {
}

InteractRecorder::InteractRecorder(std::filesystem::path file_path) {
  ofs.open(file_path);
}

void InteractRecorder::record(int x, int y) {
  ofs << fmt::format("{},{}\n", x, y);
  ofs.flush();
}
// vim:ts=2 sw=2 sts=2 et ci

