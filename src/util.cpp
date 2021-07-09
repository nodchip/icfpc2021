#include "stdafx.h"
#include "util.h"
#include <optional>
#include <filesystem>

std::vector<std::string> split(std::string s, std::string delimiter) {
  std::vector<std::string> res;
  while (!s.empty()) {
    auto i = s.find(delimiter);
    if (i == std::string::npos) {
      res.push_back(s);
      break;
    }
    if (i > 0) {
      res.push_back(s.substr(0, i));
    }
    s = s.substr(i + delimiter.size());
  }
  return res;
}

std::string join(std::vector<std::string> tokens, std::string delimiter) {
  std::ostringstream oss;
  for (auto i = 0; i < tokens.size(); ++i) {
    if (i != 0) oss << delimiter;
    oss << tokens[i];
  }
  return oss.str();
}

std::pair<std::string, std::string> split_first(std::string s, std::string delimiter) {
  auto i = s.find(delimiter);
  if (i == std::string::npos) return {s, ""};
  return {s.substr(0, i), s.substr(i + delimiter.size())};
}

bool starts_with(std::string_view s, std::string prefix) {
  return s.find(prefix) == 0;
}

std::string strip(std::string s) {
  if (s.empty()) return s;
  auto l = s.find_first_not_of(" ");
  auto r = s.find_last_not_of(" ");
  return s.substr(l, r + 1 - l);
}

std::optional<std::string> read_file(std::filesystem::path file_path) {
  if (!std::filesystem::is_regular_file(file_path)) {
    return std::nullopt;
  }
  std::ifstream ifs(file_path.c_str(), std::ios::binary);
  std::string content((std::istreambuf_iterator<char>(ifs)), std::istreambuf_iterator<char>());
  return content;
}

void dump_func() {
    DUMPOUT << std::endl;
}

// vim:ts=2 sw=2 sts=2 et ci


