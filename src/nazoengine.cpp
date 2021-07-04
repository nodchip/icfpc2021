#include "stdafx.h"
#include "nazoengine.h"

#include <filesystem>
#include <fstream>
#include <sstream>
#include <stack>

#include "contest_communication.h"
#include "interact_record.h"
#include "picture.h"
#include "util.h"

// HACK: set this to false (by !name_partial_off) to disable name_partial() recursive evaluation.
static bool g_enable_name_partial = true;

void Node::create_node(bool add_to_stat) {
  ++engine->n_create_node;
  engine->node_statistics.create<Node>(nullptr); // this is a partial node.
}
uint64_t Node::_get_serial() {
  return engine->n_create_node;
}
std::string Node::repr(bool with_brackets) const {
  if (engine->show_serial_number) {
    return custom_repr ? custom_repr(this, with_brackets) : fmt::format("{}:{:x}", get_name(), serial_number);
  } else {
    return custom_repr ? custom_repr(this, with_brackets) : get_name();
  }
}

Engine::Engine() : output(&std::cout) {
  sendrecv_func = [](std::string_view send_text) -> std::string {
    LOG(INFO) << fmt::format("DUMMY SEND: \"{}\"", send_text);
    LOG(INFO) << "INPUT DUMMY RECV> ";
    std::string recv_text;
    std::cin >> recv_text;
    LOG(INFO) << fmt::format("DUMMY RECV: \"{}\"", recv_text);
    return recv_text;
  };

  define_global_symbol::define(this);
  parse_statement("pwr2 = ap ap s ap ap c ap eq 0 1 ap ap b ap mul 2 ap ap b pwr2 ap add -1");
  parse_statement("checkerboard = ap ap s ap ap b s ap ap c ap ap b c ap ap b ap c ap c ap ap s ap ap b s ap ap b ap b ap ap s i i lt eq ap ap s mul i nil ap ap s ap ap b s ap ap b ap b cons ap ap s ap ap b s ap ap b ap b cons ap c div ap c ap ap s ap ap b b ap ap c ap ap b b add neg ap ap b ap s mul div ap ap c ap ap b b checkerboard ap ap c add 2");

  // send and receive objects.
  // ap send <obj> = ap ap ap b send_recv_binary mod <obj> = send_recv_binary(mod(<obj>))
  parse_statement("send = ap ap b send_recv_binary mod");

  // #32. Draw. looks like a synonym for the identity function...
  // parse_statement("draw = i");

  // #38. Interact.
  parse_statement("modem = ap ap b dem mod");

  // #40. stateless draw. (tested)
  parse_statement("statelessdraw = ap ap c ap ap b b ap ap b ap b ap cons 0 ap ap c ap ap b b cons ap ap c cons nil ap ap c ap ap b cons ap ap c cons nil nil");

  // #41. stateless draw. (tested)
  parse_statement(":67108929 = ap ap b ap b ap ap s ap ap b ap b ap cons 0 ap ap c ap ap b b cons ap ap c cons nil ap ap c cons nil ap c cons");
  parse_statement("statefuldraw = :67108929");

  special_commands["!help"] = {
    ".. show commands help",
    [this](Engine*, auto args) {
      for (auto [cmd, item] : special_commands) {
        (*output) << fmt::format("{} {}", cmd, item.first) << std::endl;
      }
    }};
  special_commands["!list"] = { ".. list all symbols",
    [this](Engine*, auto args) { special_list(args); }};
  special_commands["!load"] = {"(<file>, ..) .. load files",
    [this](Engine*, auto args) { special_load(args); }};
  special_commands["!perf"] = {"<expr> .. measure performance",
    [this](Engine*, auto args) { special_perf(args); }};
  special_commands["!verbose"] = {".. set verbose=ON",
    [this](Engine*, auto args) { verbose = true; }};
  special_commands["!quiet"] = {".. set verbose=OFF",
    [this](Engine*, auto args) { verbose = false; }};
  special_commands["!list_repr"] = {".. set list_repr=ON",
    [this](Engine*, auto args) { list_repr = true; }};
  special_commands["!cons_repr"] = {".. set list_repr=OFF",
    [this](Engine*, auto args) { list_repr = false; }};
  special_commands["!serial_number_show"] = {".. set show_serial_number=ON",
    [this](Engine*, auto args) { show_serial_number = true; }};
  special_commands["!serial_number_hide"] = {".. set show_serial_number=OFF",
    [this](Engine*, auto args) { show_serial_number = false; }};
  special_commands["!rm"] = {"(<symbol_name>, ..) .. remove symbol from the table",
    [this](Engine*, auto args) {
    for (auto name : args) {
      auto it = table.find(name);
      if (it == table.end()) {
        (*output) << fmt::format("remove {} .. not found", name) << std::endl;
      } else {
        if (verbose) (*output) << fmt::format("removed {}", name) << std::endl;
        table.erase(it);
      }
    }
  }};
  special_commands["!interact"] = {"<protocol> .. interactive protocol",
    [this](Engine*, auto args) { special_interact(args); }};
  special_commands["!dot"] = {"<name> <eval:t/f> <expr> .. draw <expr> to a file",
    [this](Engine*, auto args) {
      if (args.size() >= 3) {
        std::filesystem::path file_path = args[0];
        file_path = file_path.replace_extension(".dot");
        auto do_eval = args[1] == "t";
        args.erase(args.begin(), args.begin() + 2);
        auto root = parse_expr(join(args, " "));
        if (do_eval) root->eval();
        {
          std::ofstream ofs(file_path);
          ofs << root->repr_dot();
          (*output) << "output to: " << std::filesystem::absolute(file_path) << std::endl;
        }
      }
  }};
  special_commands["!name_partial_on"] = {" .. start adding detailed name to partial application nodes",
    [this](Engine*, auto args) { 
      (*output) << "enabled name_partial()" << std::endl;
      g_enable_name_partial = true;
  }};
  special_commands["!name_partial_off"] = {" .. stop adding detailed name to partial application nodes",
    [this](Engine*, auto args) { 
      (*output) << "disabled name_partial()" << std::endl;
      g_enable_name_partial = false;
  }};

}

struct ap final : public Node {
  ap(const ap& o) = delete;
  ap(Engine* engine_) : Node(engine_) { 
    node_name = "ap";
    engine_->node_statistics.create(this);
  }
  ap(Engine* engine_, Node *lhs_, Node *rhs_) : Node(engine_) {
    if (!lhs_) {
      LOG(ERROR) << "ap::ctor lhs_ is null!";
    }
    if (!rhs_) {
      LOG(ERROR) << "ap::ctor rhs_ is null!";
    }
    lhs = lhs_; rhs = rhs_; node_name = "ap";
    engine_->node_statistics.create(this);
  }
  bool is_valid() const override {
    if (evaluated && evaluated != this) {
      return evaluated->is_valid();
    }
    return lhs && rhs;
  }

  // @return not null
  Node* eval() override {
    if (engine->verbose) LOG(INFO) << "eval: " << repr(true);
    int iter = 0;
    while (!evaluated) {
      if (false) {
        std::ofstream ofs(fmt::format("dot_{:04d}.dot", iter));
        ofs << repr_dot();
        ++iter;
      }

      if (!lhs) {
        LOG(ERROR) << "ap::eval lhs is NULL";
      }
      if (lhs->apply) {
        auto apply_res = lhs->apply(engine, rhs);
        if (!apply_res)
          return this;
        evaluated = apply_res->eval();
      } else {
        lhs = lhs->eval();
        if (!lhs) {
          LOG(ERROR) << "NULL";
        }
      }
      ++engine->n_eval;
    }

    while (true) {
      if (auto reevaluated = evaluated->eval(); reevaluated != evaluated) {
        evaluated = reevaluated;
      } else {
        CHECK(evaluated);
        return evaluated;
      }
    }
  }
  std::string repr(bool with_brackets) const override {
    if (evaluated && evaluated != this) {
      return evaluated->repr(with_brackets);
    }

    std::string s;
    if (engine->show_serial_number) {
      s = fmt::format("ap:{:x} ", serial_number);
    } else {
      s = "ap ";
    }
    if (lhs) { s += lhs->repr(with_brackets) + " "; }
    else     { s += "NULLLHS "; }
    if (rhs) { s += rhs->repr(with_brackets); }
    else     { s += "NULLRHS"; }

    if (with_brackets) {
      return "{" + s + "}";
    } else {
      return s;
    }
  }
  void repr_dot_impl(std::ostream& os, const Node* parent, std::string label) const override {
    if (evaluated && evaluated != this) {
      evaluated->repr_dot_impl(os, parent, "");
      return;
    }
    os << "\"" << fmt::format("{}<{}>", parent->get_name(), uintptr_t(parent)) << "\" -> \"" << fmt::format("{}<{}>", get_name(), uintptr_t(this)) << "\";" << std::endl;
    lhs->repr_dot_impl(os, this, "LHS");
    rhs->repr_dot_impl(os, this, "RHS");
  }
};

number::number(Engine* engine_, int64_t value_) : Node(engine_), value(value_) {
  node_name = "number";
  custom_repr = [](const Node* this_, bool) -> std::string {
    return fmt::format("{}", dynamic_cast<const number*>(this_)->value);
  };
  evaluated = this;
  engine_->node_statistics.create(this);
}

struct variable final : public Node {
  int64_t index;
  variable(Engine* engine_, int64_t index_) : Node(engine_), index(index_) {
    node_name = "variable";
    custom_repr = [](const Node* this_, bool) -> std::string {
      return fmt::format("x{}", dynamic_cast<const variable*>(this_)->index);
    };
    evaluated = this;
    engine_->node_statistics.create(this);
  }
};

struct binary final : public Node {
  std::string bits;
  binary(Engine* engine_, const std::string& bits_) : Node(engine_), bits(bits_) {
    node_name = "binary";
    custom_repr = [](const Node* this_, bool) -> std::string {
      return fmt::format("[{}]", dynamic_cast<const binary*>(this_)->bits);
    };
    evaluated = this;
    engine_->node_statistics.create(this);
  }
};

struct list final : public Node {
  std::vector<Node*> elements;
  bool closed = false;
  list(Engine* engine_) : Node(engine_) {
    node_name = "list";
    engine_->node_statistics.create(this);
  }
  void append(Node* node) {
    elements.push_back(node);
  }
  Node* eval() override {
    Engine* e = engine;
    Node* head = e->create_op("nil");
    for (int i = int(elements.size()) - 1; i >= 0; --i) {
      head = new ap(e, new ap(e, e->create_op("cons"), elements[i]), head);
    }
    return head->eval();
  }
  std::string get_name() const override {
    std::ostringstream oss;
    oss << "(";
    bool is_first = true;
    for (auto elem : elements) {
      if (!is_first)
        oss << " ,";
      is_first = false;
      oss << " " << elem->get_name();
    }
    oss << " )";
    return oss.str();
  }
  std::string repr(bool with_brackets) const override {
    std::ostringstream oss;
    oss << "(";
    bool is_first = true;
    for (auto elem : elements) {
      if (!is_first)
        oss << " ,";
      is_first = false;
      oss << " " << elem->repr(with_brackets);
    }
    oss << " )";
    return oss.str();
  }
};

std::map<std::string, std::tuple<int, ApplyFunc, bool>> define_global_symbol::defs;

template <typename... Args>
std::string name_partial(const char* base_name, const Args&... args) {
  if (!g_enable_name_partial) {
    return base_name;
  }
  std::string name = base_name;
  for (const auto& p : std::vector<std::string> { args... } ) {
    if (true) {
      // s<add><inc>
      name = fmt::format("{}<{}>", name, p);
    } else {
      // s(add)(inc)
      name = fmt::format("{}({})", name, p);
    }
  }
  return name;
}

define_global_symbol __neg("neg", 1, [](Engine* e, Node* x) -> Node* {
    auto ev = x->eval();
    if (auto ev_num = dynamic_cast<number*>(ev)) {
      return new number(e, -ev_num->value);
    } else if (dynamic_cast<variable*>(ev)) {
      return nullptr;
    }
    LOG(ERROR) << "LOGICAL ERROR";
    return nullptr;
  });
define_global_symbol __inc("inc", 1, [](Engine* e, Node* x) -> Node* {
    auto ev = x->eval();
    if (auto ev_num = dynamic_cast<number*>(ev)) {
      return new number(e, ev_num->value + 1);
    } else if (dynamic_cast<variable*>(ev)) {
      return nullptr;
    }
    LOG(ERROR) << "LOGICAL ERROR";
    return nullptr;
  });
define_global_symbol __dec("dec", 1, [](Engine* e, Node* x) -> Node* {
    auto ev = x->eval();
    if (auto ev_num = dynamic_cast<number*>(ev)) {
      return new number(e, ev_num->value - 1);
    } else if (dynamic_cast<variable*>(ev)) {
      return nullptr;
    }
    LOG(ERROR) << "LOGICAL ERROR";
    return nullptr;
  });
define_global_symbol __add("add", 2, [](Engine* e, Node* lhs) -> Node* {
    return new Node("$add.1", e, [lhs](Engine* e, Node* rhs) -> Node* {
      auto lhs_eval = lhs->eval();
      auto rhs_eval = rhs->eval();
      auto lhs_num = dynamic_cast<number*>(lhs_eval);
      auto rhs_num = dynamic_cast<number*>(rhs_eval);
      if (lhs_num && rhs_num) {
        return new number(e, lhs_num->value + rhs_num->value);
      } else if (lhs_num && lhs_num->value == 0 && !rhs_num) {
        return rhs_eval;
      } else if (rhs_num && rhs_num->value == 0 && !lhs_num) {
        return lhs_eval;
      } else {
        return nullptr;
      }
    });
  });
define_global_symbol __mul("mul", 2, [](Engine* e, Node* lhs) -> Node* {
    return new Node("$mul.1", e, [lhs](Engine* e, Node* rhs) -> Node* {
      auto lhs_eval = lhs->eval();
      auto rhs_eval = rhs->eval();
      auto lhs_num = dynamic_cast<number*>(lhs_eval);
      auto rhs_num = dynamic_cast<number*>(rhs_eval);
      if (lhs_num && rhs_num) {
        return new number(e, lhs_num->value * rhs_num->value);
      } else if (lhs_num && lhs_num->value == 0 && !rhs_num) {
        return new number(e, 0);
      } else if (lhs_num && lhs_num->value == 1 && !rhs_num) {
        return rhs_eval;
      } else if (rhs_num && rhs_num->value == 0 && !lhs_num) {
        return new number(e, 0);
      } else if (rhs_num && rhs_num->value == 1 && !lhs_num) {
        return lhs_eval;
      } else {
        return nullptr;
      }
    });
  });
define_global_symbol __div("div", 2, [](Engine* e, Node* lhs) -> Node* {
    return new Node("$div.1", e, [lhs](Engine* e, Node* rhs) -> Node* {
      auto lhs_eval = lhs->eval();
      auto rhs_eval = rhs->eval();
      auto lhs_num = dynamic_cast<number*>(lhs_eval);
      auto rhs_num = dynamic_cast<number*>(rhs_eval);
      if (lhs_num && rhs_num) {
        return new number(e, lhs_num->value / rhs_num->value);
      } else if (rhs_num && rhs_num->value == 1 && !lhs_num) {
        return lhs_eval;
      } else {
        return nullptr;
      }
    });
  });
define_global_symbol __scomb("s", 3, [](Engine* e, Node* x0) -> Node* {
    return new Node("$s.1", e, [x0](Engine* e, Node* x1) -> Node* {
      return new Node("$s.2", e, [x0, x1](Engine* e, Node* x2) -> Node* {
        return new ap(e, new ap(e, x0, x2), new ap(e, x1, x2));
      });
    });
  });
define_global_symbol __ccomb("c", 3, [](Engine* e, Node* x0) -> Node* {
    return new Node("$c.1", e, [x0](Engine* e, Node* x1) -> Node* {
      return new Node("$c.2", e, [x0, x1](Engine* e, Node* x2) -> Node* {
        return new ap(e, new ap(e, x0, x2), x1);
      });
    });
  });
define_global_symbol __bcomb("b", 3, [](Engine* e, Node* x0) -> Node* {
    return new Node("$b.1", e, [x0](Engine* e, Node* x1) -> Node* {
      return new Node("$b.2", e, [x0, x1](Engine* e, Node* x2) -> Node* {
        return new ap(e, x0, new ap(e, x1, x2));
        // return e->parse_expr_with_locals("ap x0 ap x1 x2", {{"x0", x0}, {"x1", x1}, {"x2", x2}}); // also possible.
      });
    });
  });
define_global_symbol __icomb("i", 1, [](Engine* e, Node* x0) -> Node* {
    return x0;
  });
define_global_symbol __t("t", 2, [](Engine* e, Node* x0) -> Node* {
    return new Node("$t.1", e, [x0](Engine* e, Node*) -> Node* { // ignore second argument.
      return x0;
    });
  });
define_global_symbol __f("f", 2, [](Engine* e, Node* x0) -> Node* {
    return new Node("$f.1", e, [](Engine* e, Node* x1) -> Node* { // ignore first argument.
      return x1;
    });
  });
define_global_symbol __eq("eq", 2, [](Engine* e, Node* x0) -> Node* {
    const auto x0_eval = x0->eval();
    return new Node("$eq.1", e, [x0_eval](Engine* e, Node* x1) -> Node* {
      const auto x1_eval = x1->eval();
      if (const auto x0_value = dynamic_cast<number*>(x0_eval)) {
        if (const auto x1_value = dynamic_cast<number*>(x1_eval)) {
          // number
          return x0_value->value == x1_value->value ? e->create_op("t") : e->create_op("f");
        }
      }
      if (const auto x0_variable = dynamic_cast<variable*>(x0_eval)) {
        if (const auto x1_variable = dynamic_cast<variable*>(x1_eval)) {
          // variable
          return x0_variable->index == x1_variable->index ? e->create_op("t") : e->create_op("f");
        }
      }
      if (const auto x0_binary = dynamic_cast<binary*>(x0_eval)) {
        if (const auto x1_binary = dynamic_cast<binary*>(x1_eval)) {
          // binary
          return x0_binary->bits == x1_binary->bits ? e->create_op("t") : e->create_op("f");
        }
      }
      // unknown.
      return nullptr;
    });
  });
define_global_symbol __lt("lt", 2, [](Engine* e, Node* x0) -> Node* {
    const auto x0_eval = x0->eval();
    if (auto x0_num = dynamic_cast<number*>(x0_eval)) {
      return new Node("$lt.1", e, [x0_num](Engine* e, Node* x1) -> Node* {
        const auto x1_eval = x1->eval();
        if (auto x1_num = dynamic_cast<number*>(x1_eval)) {
          return x0_num->value < x1_num->value ? e->create_op("t") : e->create_op("f");
        } else {
          return nullptr;
        }
      });
    } else {
      return nullptr;
    }
  });
inline bool is_cons(Node* n) {
  CHECK(n);
  return n->get_name() == "cons" || n->get_name() == "vec";
}
static std::string cons_repr(const Node* this_, bool with_brackets, Node* x0, Node* x1) {
  if (this_->engine->list_repr) {
    if (x1->is_list) {
      auto head = "( " + x0->repr(with_brackets);
      if (x1->get_name() == "nil") {
        return head + " )";
      } else {
        return head + " ," + x1->repr(with_brackets).substr(1);
      }
    }
    if (auto x0_num = dynamic_cast<number*>(x0)) {
      if (auto x1_num = dynamic_cast<number*>(x1)) {
        return fmt::format("[{},{}]", x0_num->value, x1_num->value);
      }
    }
  }
  return this_->get_name();
}
define_global_symbol __cons("cons", 3, [](Engine* e, Node* x0) -> Node* {
    return new Node("$cons.1", e, [x0](Engine* e, Node* x1) -> Node* {
      // https://message-from-space.readthedocs.io/en/latest/implementation.html
      // see evalCons()
      auto x0_eval = x0->eval();
      auto x1_eval = x1->eval();
      return new Node("$cons.2", e, [x0_eval, x1_eval](Engine* e, Node* x2) -> Node* {
        return new ap(e, new ap(e, x2, x0_eval), x1_eval);
      }, [x0_eval, x1_eval](const Node* this_, bool with_brackets) -> std::string {
        return cons_repr(this_, with_brackets, x0_eval, x1_eval);
      }, x1_eval->is_list);
    });
  });
define_global_symbol __vec("vec", 3, [](Engine* e, Node* x0) -> Node* {
    return new Node("$vec.1", e, [x0](Engine* e, Node* x1) -> Node* {
      // https://message-from-space.readthedocs.io/en/latest/implementation.html
      // see evalCons()
      auto x0_eval = x0->eval();
      auto x1_eval = x1->eval();
      return new Node("$vec.2", e, [x0_eval, x1_eval](Engine* e, Node* x2) -> Node* {
        return new ap(e, new ap(e, x2, x0_eval), x1_eval);
      }, [x0_eval, x1_eval](const Node* this_, bool with_brackets) -> std::string {
        return cons_repr(this_, with_brackets, x0_eval, x1_eval);
      }, x1_eval->is_list);
    });
  });
define_global_symbol __car("car", 1, [](Engine* e, Node* ap_ap_cons_x0_x1) -> Node* {
    if (auto ap0 = dynamic_cast<ap*>(ap_ap_cons_x0_x1->eval())) {
      if (auto ap1 = dynamic_cast<ap*>(ap0->lhs->eval())) {
        if (is_cons(ap1->lhs->eval())) {
          return ap1->rhs; // x0
        }
      }
    }
    return new ap(e, ap_ap_cons_x0_x1, e->create_op("t"));
  });
define_global_symbol __cdr("cdr", 1, [](Engine* e, Node* ap_ap_cons_x0_x1) -> Node* {
    if (auto ap0 = dynamic_cast<ap*>(ap_ap_cons_x0_x1->eval())) {
      if (auto ap1 = dynamic_cast<ap*>(ap0->lhs->eval())) {
        if (is_cons(ap1->lhs->eval())) {
          return ap0->rhs; // x1
        }
      }
    }
    return new ap(e, ap_ap_cons_x0_x1, e->create_op("f"));
  });
define_global_symbol __nil("nil", 1, [](Engine* e, Node*) -> Node* {
    return e->create_op("t");
  }, true);
define_global_symbol __isnil("isnil", 1, [](Engine* e, Node* x0) -> Node* {
    if (x0->eval()->get_name() == "nil")
      return e->create_op("t");
    return e->create_op("f");
  });
define_global_symbol __if0("if0", 3, [](Engine* e, Node* pred) -> Node* {
    pred = pred->eval();
    if (auto pred_num = dynamic_cast<number*>(pred)) {
      // resolved to a concrete number.
      return new Node("$if0.1", e, [pred_num](Engine* e, Node* x0) -> Node* {
        return new Node("$if0.2", e, [pred_num, x0](Engine* e, Node* x1) -> Node* {
          if (pred_num->value == 0) {
            return x0;
          } else {
            return x1;
          }
        });
      });
    } else {
      // unknown.
      return nullptr;
    }
  });
define_global_symbol __mod("mod", 1, [](Engine* e, Node* x) -> Node* {
    auto ev = x->eval();
    if (auto ev_num = dynamic_cast<number*>(ev)) {
      std::string bits = (ev_num->value >= 0) ? "01" : "10";
      auto magnitude = std::abs(ev_num->value);
      int num_bits = 0;
      while (magnitude >> num_bits) {
        num_bits += 4;
        bits.push_back('1');
        if (num_bits >= 64) break;
      }
      bits.push_back('0');
      for (int i = num_bits - 1; i >= 0; --i) {
        bits.push_back('0' + ((magnitude >> i) & 1));
      }
      return new binary(e, bits);
    } else if (ev->get_name() == "nil") {
      return new binary(e, "00");
    }

    // Modulate a list
    return new ap(e, ev, new Node("$mod-list.1", e, [x](Engine* e, Node* x0) -> Node* {
      return new Node("$mod-list.2", e, [x0 = x0->eval()](Engine* e, Node* x1) -> Node* {
        auto ap0 = new ap(e, e->create_op("mod"), x0);
        auto ap1 = new ap(e, e->create_op("mod"), x1->eval());
        if (auto binary0 = dynamic_cast<binary*>(ap0->eval())) {
          if (auto binary1 = dynamic_cast<binary*>(ap1->eval())) {
            return new binary(e, "11" + binary0->bits + binary1->bits);
          }
        }
        return nullptr;
      });
    }));
  });
static Node* demodulate(Engine* e, std::string_view& bits) {
  if (bits.size() < 2) return nullptr;
  if (bits[0] != bits[1]) {
    int sign = (bits[0] == '0') ? 1 : -1;
    bits = bits.substr(2);
    int num_bits = bits.find('0') * 4;
    bits = bits.substr(num_bits / 4 + 1);
    std::int64_t magnitude = 0;
    for (int i = 0; i < num_bits; ++i) {
      magnitude = (magnitude << 1) | (bits[i] == '1');
    }
    bits = bits.substr(num_bits);
    return new number(e, sign * magnitude);
  }
  if (starts_with(bits, "00")) {
    bits = bits.substr(2);
    return e->create_op("nil");
  }
  bits = bits.substr(2);
  auto x0 = demodulate(e, bits);
  auto x1 = demodulate(e, bits);
  return new ap(e, new ap(e, e->create_op("cons"), x0), x1);
}
define_global_symbol __dem("dem", 1, [](Engine* e, Node* x) -> Node* {
    if (auto bin = dynamic_cast<binary*>(x->eval())) {
      std::string_view bits = bin->bits;
      return demodulate(e, bits);
    }
    return nullptr;
  });
struct putpixel final : public Node {
  Picture* picture = nullptr;
  putpixel(Engine* engine_, Picture* picture_): Node(engine_), picture(picture_) {
    node_name = "putpixel";
    apply = [this](Engine* e, Node* ap_ap_vec_x_y) -> Node* {
      // essentially a identy function but put a pixel on the picture.
      auto ev = ap_ap_vec_x_y->eval();
      if (ev->get_name() == "nil") {
        return e->create_op("nil");
      }
      return new ap(e, ev, new Node("$putpixel.1", e, [this, ev](Engine* e, Node* x0) -> Node* {
        return new Node("$putpixel.1", e, [this, ev, x0](Engine* e, Node* x1) -> Node* {
          int64_t x = dynamic_cast<number*>(x0->eval())->value;
          int64_t y = dynamic_cast<number*>(x1->eval())->value;
          if (e->verbose) LOG(INFO) << "put(" << x << "," << y << ")";
          picture->put(x, y);
          return ev;
        });
      }));
    };
  }
};
define_global_symbol __putpixel("debug_putpixel_op", 1, [](Engine* e, Node* x) -> Node* {
  return new ap(e, new putpixel(e, new Picture), x);
  });
struct draw final : public Node {
  draw(Engine* engine_, Picture* picture): Node(engine_) {
    node_name = "draw";
    apply = [picture](Engine* e, Node* x) -> Node* {
      auto ev = x->eval();
      if (ev->get_name() == "nil") {
        // finished drawing.
        if (!e->is_gui)
          picture->print();
        return e->create_op("nil");
      }
      return new ap(e, ev, new Node("$draw-list.1", e, [picture](Engine* e, Node* x0) -> Node* {
        return new Node("$draw-list.2", e, [picture, x0](Engine* e, Node* x1) -> Node* {
          auto next_draw = new draw(e, picture); // do not create_op(). keep the picture!
          return new ap(e, new ap(e, e->create_op("cons"), new ap(e, new putpixel(e, picture), x0)), new ap(e, next_draw, x1));
        });
      }));
    };
  }
};
define_global_symbol __draw("draw", 1, [](Engine* e, Node* x) -> Node* {
  auto d = new draw(e, new Picture());
  return new ap(e, d, x);
  });
define_global_symbol __multipledraw("multipledraw", 1, [](Engine* e, Node* x) -> Node* {
    auto ev = x->eval();
    if (ev->get_name() == "nil") {
      return e->create_op("nil");
    }
    return new ap(e, ev, new Node("$multipledraw-list.1", e, [x](Engine* e, Node* x0) -> Node* {
      return new Node("$multipledraw-list.2", e, [x0](Engine* e, Node* x1) -> Node* {
        return new ap(e, new ap(e, e->create_op("cons"), new ap(e, new draw(e, new Picture()), x0)), new ap(e, e->create_op("multipledraw"), x1));
        //return new ap(e, new ap(e, e->create_op("cons"), new ap(e, e->create_op("draw"), x0)), new ap(e, e->create_op("multipledraw"), x1));
        //return new ap(e, new ap(e, e->create_op("cons"), new ap(e, e->create_op("i"), x0)), new ap(e, e->create_op("multipledraw"), x1));
      });
    }));
  });
// send and receive modulated binaries.
define_global_symbol __send_recv_binary("send_recv_binary", 1, [](Engine* e, Node* x0) -> Node* {
    if (auto bin = dynamic_cast<binary*>(x0->eval())) {
      std::string_view bits = bin->bits;
      std::string recv_text = e->send_recv(bits);
      std::string_view recv_view = recv_text;
      auto received = demodulate(e, recv_view);
      auto sent = demodulate(e, bits);
      bool list_repr = e->list_repr;
      e->list_repr = true;
      LOG(INFO) << "sent: " << sent->eval()->repr(false);
      LOG(INFO) << "received: " << received->eval()->repr(false);
      e->list_repr = list_repr;
      return received;
    }
    LOG(ERROR) << "x0 is not binary";
    return nullptr;
  });

// used in "interact"
define_global_symbol __interact_f38("f38", 2, [](Engine* e, Node* x2) -> Node* {
    return new Node("$f38.1", e, [x2](Engine* e, Node* x0) -> Node* {
      return e->parse_expr_with_locals(
        // removed "ap modem" "ap multipledraw"
        //"ap ap ap if0 ap car x0 "
        //"( ap car ap cdr x0 , ap car ap cdr ap cdr x0 ) "
        //"ap ap ap interact x2 ap car ap cdr x0 ap send ap car ap cdr ap cdr x0", {
        // original!
        "ap ap ap if0 ap car x0 "
        "( ap modem ap car ap cdr x0 , ap multipledraw ap car ap cdr ap cdr x0 ) "
        "ap ap ap interact x2 ap modem ap car ap cdr x0 ap send ap car ap cdr ap cdr x0", {
        {"x0", x0}, {"x2", x2},
        });
    });
  });
define_global_symbol __interact("interact", 3, [](Engine* e, Node* x2) -> Node* {
    return new Node("$interact.1", e, [x2](Engine* e, Node* x4) -> Node* {
      return new Node("$interact.2", e, [x2, x4](Engine* e, Node* x3) -> Node* {
        return e->parse_expr_with_locals("ap ap f38 x2 ap ap x2 x4 x3", {
          {"x2", x2}, {"x3", x3}, {"x4", x4},
        });
      });
    });
  });


// TODO(test): interact(38-39)
// TODO(ok?): draw(32), multipledraw(34)
// TODO(optimize. implemented in Engine()): pwr2(23), checkerboard(33)

Node* Engine::create_op_with_locals(const std::string& name, const NamedExpr& local_nodes) {
  auto it = builtins.find(name);
  if (it != builtins.end()) {
    auto node = new Node(this);
    node->name = new std::string(name);
    std::tie(node->nary, node->apply, node->is_list) = it->second;
    node_statistics.create_name(name); // this is a builtin node. e.g.) nil, add, ..
    return node;
  }

  if (name == "ap")
    return new ap(this);
  if (local_nodes.find(name) != local_nodes.end()) {
    // reference to local variables. (before "real" variables)
    return local_nodes.find(name)->second;
  }
  if (name.size() >= 2 && name[0] == 'x') {
    // variable
    return new variable(this, std::stoi(name.substr(1)));
  }
  if (name.size() >= 1) {
    if (std::isdigit(name[0]) || name[0] == '-') {
      // number
      return new number(this, std::stoll(name));
    }
    if (name.front() == '[' && name.back() == ']') {
      // binary
      return new binary(this, name.substr(1, name.size() - 2));
    }
    if (name == "(") {
      return new list(this);
    }
    // otherwise: symbol
    return resolve(name);
  }

  std::cerr << "NOT IMPLEMENTED" << std::endl;
  return nullptr;
}

Node* Engine::parse_expr_with_locals(const std::string& nazogengo_text_code, const NamedExpr& local_nodes) {
  Node* root = nullptr;
  std::stack<Node*> stk;
  size_t prev = 0;
  for (size_t i = 0; i < nazogengo_text_code.size(); ++i) {
    if (i + 1 == nazogengo_text_code.size() || nazogengo_text_code[i + 1] == ' ') {
      auto token = nazogengo_text_code.substr(prev, i + 1 - prev);
      prev = i + 2;
      //std::cout << "[" << token << "]";
      Node* op = create_op_with_locals(token, local_nodes);
      if (!op) {
        LOG(ERROR) << "FAILED TO DECODE NAZOGENGO \"" << nazogengo_text_code << "\": op is null";
        break;
      }
      if (!root) {
        root = op;
      } else {
        if (auto ap_node = dynamic_cast<ap*>(stk.top())) {
          if (!ap_node->lhs) {
            if (!op) {
              LOG(ERROR) << "parse_expr_with_locals: lhs is NULL";
            }
            ap_node->lhs = op;
          } else if (!ap_node->rhs) {
            if (!op) {
              LOG(ERROR) << "parse_expr_with_locals: rhs is NULL";
            }
            ap_node->rhs = op;
          }
          if (ap_node->rhs) { stk.pop(); } // fullfilled
        } else if (auto list_node = dynamic_cast<list*>(stk.top())) {
          if (op->get_name() == ")") {
            list_node->closed = true;
            stk.pop();
          } else if (op->get_name() != ",") {
            list_node->append(op);
          }
        }
      }
      if (auto ap_node = dynamic_cast<ap*>(op); ap_node && !ap_node->lhs) {
        stk.push(op); // new ap on the top.
      }
      if (auto list_node = dynamic_cast<list*>(op); list_node && !list_node->closed) {
        stk.push(op); // new list on the top.
      }
    }
  }
  //std::cout << std::endl;
  if (!stk.empty()) {
    LOG(ERROR) << "FAILED TO DECODE NAZOGENGO \"" << nazogengo_text_code << "\": stk.size() = " << stk.size();
  }
  if (!root->is_valid()) {
    LOG(ERROR) << root->get_name() << " is invalid";
  }
  return root;
}


void Engine::parse_statement(std::string stmt) {
  if (!stmt.empty() && stmt[0] == '!') {
    auto tokens = split(strip(stmt), " ");
    if (!tokens.empty()) {
      auto it = special_commands.find(tokens[0]);
      tokens.erase(tokens.begin());
      if (it != special_commands.end()) {
        it->second.second(this, tokens);
      } else {
        (*output) << "unknown special command: " << stmt << "\n";
        parse_statement("!help"); // hope this exists!
      }
    }
  } else {
    auto ieq = stmt.find('=');
    bool quiet_statement = false;
    if (!stmt.empty() && *stmt.rbegin() == ';') {
      quiet_statement = true;
      stmt = stmt.substr(0, stmt.length() - 1);
    }
    if (ieq > 0 && ieq != std::string::npos) {
      // normal assignment statements
      auto symbol_name = stmt.substr(0, ieq - 1);
      auto expr_txt = stmt.substr(ieq + 2);
      if (symbol_name != "pwr2" && symbol_name != "checkerboard") {
        if (verbose) (*output) << "[" << symbol_name << "] <- [" << expr_txt << "]" << std::endl;
      }
      def(symbol_name, parse_expr(expr_txt));
    } else {
      // evaluate expression.
      Node* r = parse_expr(stmt);
      if (verbose) (*output) << "!! EXPR PARSED AS: " << r->repr(true) << std::endl;
      auto evaluated = r->eval();
      if (verbose) {
        (*output) << "!! EVALUATED TO  : " << evaluated->repr(true) << std::endl;
      } else if (!quiet_statement) {
        (*output) << evaluated->repr(false) << std::endl;
      }
    }
  }
}

void Engine::parse_code(std::string code) {
  std::string line;
  std::istringstream iss(code);
  while (std::getline(iss, line)) {
    parse_statement(line);
  }
}

void Engine::repl() {
  std::cout << "!! Nazo-lang processor REPL !!" << std::endl;
  std::cout << std::endl;

  std::map<std::string, Node*> thunks;
  while (true) {
    std::cout << "> ";
    std::cout.flush();
    std::string line;
    if (!std::getline(std::cin, line)) {
      break;
    }
    line = strip(line);
    if (line.empty()) {
      continue;
    }

    // some special commands only valid in REPL.
    if (line == "!q") {
      std::cout << "!! QUIT" << std::endl;
      break;
    } else if (starts_with(line, "!noeval ")) {
      // !noeval <name> = <expr>
      auto [name, expr_text] = split_first(line.substr(8), "=");
      name = strip(name);
      expr_text = strip(expr_text);
      thunks[name] = parse_expr(expr_text);
      std::cout << "!! EXPR PARSED AS: " << thunks[name]->repr(true) << std::endl;
    } else if (starts_with(line, "!step ")) {
      // step evaluation.
      // !step tmp     -> single step.
      // !step tmp 5   -> 5 steps.
      auto tokens = split(line.substr(5), " ");
      if (tokens.size() > 2) {
        LOG(ERROR) << "invalid command";
        continue;
      } else {
        auto name = strip(tokens[0]);
        int loop = 1;
        if (tokens.size() == 2) {
          try {
            loop = std::stol(tokens[1]);
          } catch(const std::invalid_argument&) {
            LOG(ERROR) << "invalid command";
            continue;
          }
        }
        auto node = thunks[name];
        std::cout << fmt::format("!! STEP EVALUATION OF {}. BEFORE: {}", name, node->repr(true)) << std::endl;
        for (int i = 0; i < loop; ++i) {
          if (node->evaluated) break;
          node = node->eval_step();
          std::cout << fmt::format("!! STEP EVALUATION OF {}. AFTER : {}", name, node->repr(true)) << std::endl;
        }
        thunks[name] = node;
      }
    } else {
      parse_statement(line);
    }
  }
}

void Engine::special_list(std::vector<std::string> args) {
  // list all symbols and builtins.
  std::cout << fmt::format("!! {} BUILTINS", builtins.size()) << std::endl;
  for (auto [name, applyfunc] : builtins) {
    std::cout << fmt::format("!!  {}", name) << std::endl;
  }
  std::cout << std::endl;
  std::cout << fmt::format("!! {} SYMBOLS", table.size()) << std::endl;
  for (auto [name, sym] : table) {
    std::cout << fmt::format("!!  {} = {}", name, sym->repr(true)) << std::endl;
  }
}

void Engine::special_load(std::vector<std::string> args) {
  // %load filename0 filename1 ..
  if (args.size() >= 1) {
    for (auto f : args) {
      std::filesystem::path path = f; // abs path
      if (!path.is_absolute()) { // rel path
        path = std::filesystem::current_path() / f;
      }
      if (std::filesystem::is_regular_file(path)) {
        LOG(INFO) << "Loading: " << path;
        auto ps = start_performance_snapshot();
        std::ifstream ifs(path);
        std::string line;
        int n = 0;
        while (std::getline(ifs, line)) {
          parse_statement(line);
          ++n;
        }
        ps.stop();
        LOG(INFO) << fmt::format("Loaded {} lines in {:.2f} ms, {} nodes, {} evals.", n, ps.elapsed_us() * 1e-3, ps.n_create_node, ps.n_eval);
      } else {
        LOG(ERROR) << "File not found: " << path;
      }
    }
  }
}

void Engine::special_perf(std::vector<std::string> args) {
  // %perf <expr>
  if (args.size() >= 1) {
    auto ps = start_performance_snapshot();
    parse_statement(join(args, " "));
    ps.stop();
    LOG(INFO) << fmt::format("PERFORMANCE : {:.2f} us, {} nodes, {} evals", ps.elapsed_us(), ps.n_create_node, ps.n_eval);
  }
}

void Engine::special_interact(std::vector<std::string> args) {
  // https://message-from-space.readthedocs.io/en/latest/implementation.html
  std::string protocol_name = "galaxy";
  if (!args.empty()) {
    protocol_name = args[0];
  }
  LOG(INFO) << "Protocol = " << protocol_name;

  InteractRecorder interact_recorder(fmt::format("interact_{}.log", protocol_name));

  parse_statement(fmt::format(":state0 = nil")); // init

  int i = 0;
  while (true) {
    std::cout << fmt::format("!! i={}. input. x,y or quit > ", i);
    std::cout.flush();
    std::string s;
    std::cin >> s;
    if (s == "quit") {
      break;
    }
    if (!s.empty() && s[0] == '!') {
      auto tokens = split(strip(s), " ");
      if (tokens[0] == "!verbose" || tokens[0] == "!quiet" ||
          tokens[0] == "!list_repr" || tokens[0] == "!cons_repr") {
        parse_statement(s);
        continue;
      }
    }
    std::replace(s.begin(), s.end(), ',', ' ');
    auto xy = split(s, " ");
    const int x = std::stol(xy[0]);
    const int y = std::stol(xy[1]);
    interact_recorder.record(x, y);

    parse_statement(fmt::format(":click{} = ap ap cons {}", i, s));
    parse_statement(fmt::format(":result{} = ap ap ap interact {} :state{} :click{}", i + 1, protocol_name, i, i));
    parse_statement(fmt::format(":state{} = ap car :result{}", i + 1, i + 1));
    parse_statement(fmt::format(":picture{} = ap car ap cdr :result{}", i + 1, i + 1));
    parse_statement(fmt::format("ap mod :picture{};", i + 1)); // ";" to ignore the value. just print the board.
    parse_statement(fmt::format("!rm :click{} :state{} :result{} :picture{}", i, i, i, i));
    ++i;
  }
  parse_statement(fmt::format("!rm :state{} :result{} :picture{}", i, i, i)); // final state.
}

std::vector<std::filesystem::path> enumerate_text_files(std::vector<std::filesystem::path> search_paths) {
  std::vector<std::filesystem::path> text_files;
  for (auto search_path : search_paths) {
    if (std::filesystem::is_directory(search_path)) {
      for (auto f : std::filesystem::directory_iterator(search_path)) {
        if (f.is_regular_file() && f.path().extension() == ".txt") {
          text_files.push_back(f.path());
        }
      }
    }
  }
  return text_files;
}

// vim:ts=2 sw=2 sts=2 et ci
