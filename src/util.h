#pragma once
#include <filesystem>
#include <optional>

std::vector<std::string> split(std::string s, std::string delimiter);
std::string join(std::vector<std::string> tokens, std::string delimiter);
std::pair<std::string, std::string> split_first(std::string s, std::string delimiter);
bool starts_with(std::string_view s, std::string prefix);
std::string strip(std::string s);
std::optional<std::string> read_file(std::filesystem::path file_path);

// vim:ts=2 sw=2 sts=2 et ci
