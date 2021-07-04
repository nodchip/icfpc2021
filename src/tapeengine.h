#pragma once
#include <map>
#include <stack>
#include <optional>

namespace NTapeEngine {

struct TapeEngine {
  struct node;
  struct ap_info {
    node* lhs = nullptr; 
    node* rhs = nullptr; 
  };
  struct node { 
    std::string op;
    bool evaluated = false;
    bool is_variable = false;
    int64_t num = 0;
    ap_info* ap = nullptr;
  };
  std::vector<node*> ops;
  int64_t cursor = 0;
  std::stack<node*> ap_stack;

  TapeEngine(std::string s);
  void print();
  void eval();

  std::optional<int64_t> get_value() {
    if (get_root()->evaluated && !get_root()->is_variable) return get_root()->num;
    return std::nullopt;
  }

  std::optional<int64_t> get_variable() {
    if (get_root()->evaluated && get_root()->is_variable) return get_root()->num;
    return std::nullopt;
  }

  node* get_root() {
    return ops[0];
  }
};

}
// vim:ts=2 sw=2 sts=2 et ci

