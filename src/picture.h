#pragma once

#include <string_view>
#include <map>
#include <utility>
#include <queue>
#include <nlohmann/json.hpp>

struct Picture {
  using Color = bool;
  using Point = std::pair<int, int>;

  Picture();
  void generateFile(const std::string_view filepath);
  void dumpJSON(const std::string& filepath,
                        const std::string& image_tag) const;

  void put(int x, int y, Color p=true);
  Color get(int x, int y) const;
  void print() const;
  nlohmann::json to_json() const;
  
  int width = 17;
  int height = 13;
  int offset_x = 0;
  int offset_y = 0;

  // (x, y) -> Color
  std::map<std::pair<int, int>, Color> pixels;

  static std::vector<Picture*>& get_history() {
    return history;
  }
private:
  static std::vector<Picture*> history;
};

