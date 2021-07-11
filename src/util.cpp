#include "stdafx.h"
#include "util.h"
#include <optional>
#include <sstream>
#include <iomanip>
#include <ctime>
#include <filesystem>
#include <fstream>
#include <fmt/core.h>

#ifdef ALLOW_UNUSED_RESULT
#undef ALLOW_UNUSED_RESULT
#endif
#define ALLOW_UNUSED_RESULT(x) { auto _ = x; (void)(_); }

std::filesystem::path default_data_path() {
#ifdef _MSC_VER
  return std::filesystem::current_path().parent_path().parent_path() / "data";
#else
  return std::filesystem::current_path().parent_path() / "data";
#endif
}

std::filesystem::path default_problem_path(int num) {
  return default_data_path() / "problems" / fmt::format("{}.problem.json", num);
}

std::string get_git_commit_id() {
  ALLOW_UNUSED_RESULT(std::system("git rev-parse HEAD > _HEAD.txt"));
  std::ifstream ifs("_HEAD.txt");
  std::string commit_id;
  ifs >> commit_id;
  return commit_id;
}

bool update_meta(nlohmann::json& solution_json, const std::string& solver_name) {
  auto now = std::chrono::system_clock::now();
  auto in_time_t = std::chrono::system_clock::to_time_t(now);
  std::ostringstream oss;
  oss << std::put_time(localtime(&in_time_t), "%Y/%m/%d %H:%M:%S");

  if (solution_json.find("meta") == solution_json.end()) solution_json["meta"] = {};
  solution_json["meta"]["created_at"] = oss.str();
  solution_json["meta"]["solver"] = solver_name;
  solution_json["meta"]["git_commit"] = get_git_commit_id();
  return true;
}

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

// vim:ts=2 sw=2 sts=2 et ci


