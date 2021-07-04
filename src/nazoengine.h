#pragma once

#include <filesystem>
#include <functional>
#include <iostream>
#include <map>
#include <sstream>

#include <glog/logging.h>
#include <fmt/format.h>

struct NodeStatistics {

  void create_name(std::string name) {
    auto it = node_count.find(name);
    if (it == node_count.end()) {
      node_count.insert({name, 1});
    } else {
      ++it->second;
    }
  }

  template <typename TNode>
  void create(const TNode* node) {
    create_name(typeid(TNode).name());
  }
  template <typename TNode>
  void destroy(const TNode* node) {
    auto name = typeid(TNode).name();
    --node_count[name];
  }

  std::map<std::string, int64_t> get_statistics() const {
    return node_count;
  }

private:
  std::map<std::string, int64_t> node_count;
};

struct Engine;
struct Node {
  Engine* engine;
  const char* node_name = "Node";
  std::string* name = nullptr;
  Node* evaluated = nullptr;
  Node* lhs = nullptr;
  Node* rhs = nullptr;
  int nary = 0;
  uint64_t serial_number = std::numeric_limits<uint64_t>::max();
  std::function<Node*(Engine*, Node*)> apply;
  std::function<std::string(const Node*, bool)> custom_repr;
  bool is_list = false;

  Node(const Node& o) : engine(o.engine), node_name(o.node_name), name(o.name), evaluated(o.evaluated), serial_number(_get_serial()), apply(o.apply) { create_node(); }
  Node(Engine* engine_) : engine(engine_), serial_number(_get_serial()) { create_node(); }
  Node(const char* node_name_, Engine* engine_, std::function<Node*(Engine*, Node*)> apply_,
    std::function<std::string(const Node*, bool)> custom_repr_ = nullptr, bool is_list_ = false)
    : engine(engine_), node_name(node_name_), serial_number(_get_serial()), apply(apply_), custom_repr(custom_repr_), is_list(is_list_) { 
    create_node(true);
  }
  virtual ~Node() {}

  void create_node(bool add_to_stat = false);
  uint64_t _get_serial(); // hack.

  // evaluate the leftmost unevaluated thunk.
  // @return nullptr: did not evaluate. otherwise: evaluated.
  virtual Node* eval_step(bool* progress = nullptr) { return this; }

  // evaluate as far as possible
  virtual Node* eval() { return evaluated ? evaluated : this; }
  virtual bool is_valid() const { return true; }

  virtual Node* clone() {
    return new Node(*this);
  }
  void print() const { std::cout << node_name << std::endl; }
  virtual std::string repr(bool with_brackets) const;
  std::string repr_dot() const {
    std::ostringstream oss;
    oss << "digraph nodes {" << std::endl;;
    repr_dot_impl(oss, this, "");
    oss << "}" << std::endl;
    return oss.str();
  }
  virtual void repr_dot_impl(std::ostream& os, const Node* parent, std::string label_name) const {
    os << "\"" << fmt::format("{}<{}>", parent->get_name(), uintptr_t(parent)) << "\" -> \"" << fmt::format("{}<{}>", get_name(), uintptr_t(this)) << "\"";
    if (!label_name.empty()) {
      os << "[ label = \"" + label_name + "\" ]";
    }
    os << ";" << std::endl;
  }
  virtual std::string get_name() const { return name ? *name : node_name; }
};
using ApplyFunc = std::function<Node*(Engine*, Node*)>;

struct number final : public Node {
  int64_t value;
  number(Engine* engine_, int64_t value_);
};

struct symbol final : public Node {
  Node* body = nullptr;
  symbol(const symbol& o) = delete;
  symbol(Engine* engine_) : Node(engine_) {}
  void set_body(Node* body_) {
    body = body_;
  }
  Node* eval() override {
    if (!evaluated) {
      evaluated = body;
    }
    if (evaluated) {
      while (true) {
        if (auto reevaluated = evaluated->eval(); reevaluated != evaluated) {
          evaluated = reevaluated;
        } else {
          return evaluated;
        }
      }
    }
    return this;
  }
};

struct Engine {
  std::map<std::string, std::tuple<int, ApplyFunc, bool>> builtins;
  std::map<std::string, std::pair<std::string /* help */, std::function<void(Engine*, std::vector<std::string>)>>> special_commands; // starts with "%". (engine, tokens)
  std::map<std::string, symbol*> table;
  std::function<std::string(std::string_view)> sendrecv_func;
  bool verbose = false;
  bool list_repr = false;
  bool show_serial_number = false;
  bool is_gui = false;
  std::ostream *output = nullptr;
  NodeStatistics node_statistics;

  // performance info.
  struct PerformanceSnapshot {
    std::chrono::system_clock::time_point tbegin;
    std::chrono::system_clock::time_point tend;
    size_t n_eval = 0;
    size_t n_create_node = 0;
    Engine* engine = nullptr;
    void stop() {
      n_eval = engine->n_eval - n_eval_begin;
      n_create_node = engine->n_create_node - n_create_node_begin;
      tend = std::chrono::system_clock::now();
    }
    double elapsed_us() {
      return std::chrono::duration_cast<std::chrono::nanoseconds>(tend - tbegin).count() * 1e-3;
    }
  private:
    size_t n_eval_begin = 0;
    size_t n_create_node_begin = 0;
    PerformanceSnapshot() {}
    friend struct Engine;
  };
  size_t n_eval = 0;
  size_t n_create_node = 0;

  Engine();

  PerformanceSnapshot start_performance_snapshot() {
    PerformanceSnapshot ps;
    ps.tend = ps.tbegin = std::chrono::system_clock::now();
    ps.n_eval_begin = n_eval;
    ps.n_create_node_begin = n_create_node;
    ps.engine = this;
    return ps;
  }

  void set_sendrecv_func(std::function<std::string(std::string_view)> sendrecv_func_) {
    sendrecv_func = sendrecv_func_;
  }
  std::string send_recv(std::string_view send_text) {
    if (sendrecv_func) {
      return sendrecv_func(send_text);
    }
    LOG(ERROR) << "call set_sendrecv_func()!";
    return "00";
  }

  symbol* def(const std::string& name, Node* body) {
    auto it = table.find(name);
    symbol* func = nullptr;
    if (it != table.end()) {
      if (it->second->evaluated) {
        std::cerr << "OVERWRITE SYMBOL BODY " << name << std::endl;
        it->second->evaluated = nullptr;
      }
      func = it->second;
    } else {
      func = new symbol(this);
    }
    func->name = new std::string(name);
    func->set_body(body);
    if (body && name != "pwr2" && name != "checkerboard") {
      if (verbose) std::cerr << "DEFINED [" << name << "] with a body" << std::endl;
    }
    table[name] = func;
    return func;
  }
  symbol* resolve(const std::string& name) {
    auto it = table.find(name);
    if (it != table.end()) {
      // already registered (with or without body).
      return it->second;
    }
    // placeholder.
    return def(name, nullptr);
  }

  void reset_output_stream() { output = &std::cout; }
  void set_output_stream(std::ostream& os) { output = &os; }

  using NamedExpr = std::map<std::string, Node*>;
  Node* parse_expr_with_locals(const std::string& nazogengo_text_code, const NamedExpr& local_nodes); // expression
  Node* parse_expr(const std::string& nazogengo_text_code) {
    return parse_expr_with_locals(nazogengo_text_code, {});
  }

  void parse_statement(std::string stmt); // single line 
  void parse_code(std::string code); // multiple lines

  Node* create_op_with_locals(const std::string& name, const NamedExpr& local_nodes);
  Node* create_op(const std::string& name) { return create_op_with_locals(name, {}); }

  void repl();

  void special_list(std::vector<std::string> args);
  void special_load(std::vector<std::string> args);
  void special_perf(std::vector<std::string> args);
  void special_interact(std::vector<std::string> args);
};

std::vector<std::filesystem::path> enumerate_text_files(std::vector<std::filesystem::path> search_paths);

struct define_global_symbol {
  define_global_symbol(std::string name, int nary, ApplyFunc apply, bool is_list = false) {
    defs[name] = {nary, apply, is_list};
  }
  static void define(Engine* engine) {
    engine->builtins = defs;
  }
  static std::map<std::string, std::tuple<int /*ary*/, ApplyFunc, bool /*is_list*/>> defs;
};
