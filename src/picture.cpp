#include "stdafx.h"
#include "picture.h"

#include <iostream>
#include <fmt/format.h>
#include <nlohmann/json.hpp>

Picture::Picture() {
  history.push_back(this);
}

void Picture::put(int x, int y, Color p) {
  pixels[{x, y}] = p;

  // Resize
  if (x < offset_x) {
    int add_width = offset_x - x;
    width += add_width;
    offset_x -= add_width;
  } else if (x >= offset_x + width) {
    int rel_x = x - offset_x;
    width = rel_x + 1;
  }
  if (y < offset_y) {
    int add_height = offset_y - y;
    height += add_height;
    offset_y -= add_height;
  } else if (y >= offset_y + height) {
    int rel_y = y - offset_y;
    height = rel_y + 1;
  }
}

Picture::Color Picture::get(int x, int y) const {
  Point point(x, y);
  auto iter = pixels.find(point);
  if (iter == pixels.end())
    return false;
  return iter->second;
}

void Picture::generateFile(const std::string_view filepath) {
  LOG(INFO) << "Picture::generateFile() is not implemented yet";
}

void Picture::print() const {
  std::cout << "\n+(" << offset_x << "," << offset_y << ")-("
            << (offset_x + width - 1) << ","
            << (offset_y + height - 1) << ")"
            << std::string(std::max(0, width - 7), '-') << "+\n";
  for (int rel_y = 0; rel_y < height; ++rel_y) {
    std::cout << "|";
    int y = offset_y + rel_y;
    for (int rel_x = 0; rel_x < width; ++rel_x) {
      int x = offset_x + rel_x;
      std::cout << ".#"[static_cast<int>(get(x, y))];
    }
    std::cout << "|\n";
  }
  std::cout << "+" << std::string(width, '-') << "+\n";
}

void Picture::dumpJSON(const std::string& filepath,
  const std::string& image_tag) const {
  // open json file
  std::ifstream ifs(filepath.c_str());
  nlohmann::json js;
  if (ifs.is_open()) ifs >> js;
  ifs.close();

  auto coords = nlohmann::json::array();
  for (auto& px : pixels) {
    if (px.second) coords.push_back(px.first);
  }

  if (js.count(image_tag) == 0 || js[image_tag].is_null()) {
    js[image_tag] = coords;
  } else {
    if (js[image_tag].size() == 0) {
      js[image_tag] = coords;
    } else if ((!js[image_tag].at(0).is_null() &&
                js[image_tag].at(0).size() > 0) &&
              js[image_tag].at(0).at(0).is_number()) {
      // create layered image.
      auto layers = nlohmann::json::array();
      layers.push_back(js[image_tag]);
      layers.push_back(coords);
      js[image_tag] = layers;
    } else { // layered image
      js[image_tag].push_back(coords);
    }
  }

  if (js.size() > 0) {
    int el_cnts = 0;
    std::stringstream ss;
    ss << "{" << std::endl;
    for (auto& el : js.items()) {
      ss << "  ";
      ss << "\"" << el.key() << "\" : " << el.value();
      if (++el_cnts < js.size()) ss << ",";
      ss << std::endl;
    }
    ss << "}" << std::endl;
    std::ofstream ofs(filepath.c_str());
    ofs << ss.str();
    ofs.close();
  }
}

nlohmann::json Picture::to_json() const {
  nlohmann::json j = nlohmann::json::array();
  for (auto [pos, color] : pixels) {
    auto [x, y] = pos;
    auto jxy = nlohmann::json::array();
    jxy.push_back(x);
    jxy.push_back(y);
    j.push_back(jxy);
  }
  return j;
}

std::vector<Picture*> Picture::history;
