#include "stdafx.h"
#include "tapeengine.h"

#include <iostream>
#include <sstream>
#include <stack>
#include <string>

#include "util.h"

namespace NTapeEngine {

TapeEngine::TapeEngine(std::string s) {
  for (auto token : split(s, " ")) {
    ops.push_back(new node { token, false, false, 0 } );
  }
}

void TapeEngine::print() {
  for (auto token : ops) {
    if (token->evaluated) {
      if (token->is_variable) {
        std::cout << "x" << token->num << " ";
      } else if (!token->is_variable) {
        std::cout << token->num << " ";
      }
    } else {
        std::cout << token->num << " ";
    }
  }
  std::cout << std::endl;
}

void TapeEngine::eval() {
  // var
  if (ops[cursor]->op[0] == 'x') {
    try {
      int64_t num = std::stoll(ops[cursor]->op);
      ops[cursor]->evaluated = true;
      ops[cursor]->is_variable = true;
      ops[cursor]->num = num;
      return;
    } catch (...) {
    }
  }
  // num
  try {
    int64_t num = std::stoll(ops[cursor]->op);
    ops[cursor]->evaluated = true;
    ops[cursor]->is_variable = false;
    ops[cursor]->num = num;
    return;
  } catch (...) {
  }
  // ap
  if (ops[cursor]->op == "ap") {
    auto ap = ops[cursor];
    ap->ap = new ap_info;
    ap_stack.push(ap);
    ++cursor;
    eval();
    ap->ap->lhs = ops[cursor];
    std::cout << "ERASE:" << ops[cursor]->op << std::endl;
    ops.erase(ops.begin() + cursor);
    eval();
    ap->ap->rhs = ops[cursor];
    std::cout << "ERASE:" << ops[cursor]->op << std::endl;
    ops.erase(ops.begin() + cursor);
    --cursor;
    
    if (ap->ap->lhs->op == "inc") {
      ap->num = ap->ap->rhs->num + 1;
      ap->evaluated = true;
      ap->is_variable = false;
    }
    ap_stack.pop();

    return;
  }
}

}
// vim:ts=2 sw=2 sts=2 et ci


