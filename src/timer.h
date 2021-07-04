#pragma once
#include <chrono>
#include <iostream>

#include <fmt/format.h>

class Timer {
public:
  Timer() : name("unnamed"), start(std::chrono::system_clock::now()), output(false) {}
  Timer(const char *name_, bool output_=true) : name(name_), start(std::chrono::system_clock::now()), output(output_) {}
  ~Timer() {
    if (output) std::cout << fmt::format("[{1:>10.2f}ms] {0}", name, elapsed_ms()) << std::endl;
  }
  void tick(const char *text) {
    if (output) std::cout << fmt::format("[{2:>10s}ms] {0}:{1}", name, text, fmt::format("..{:.2f}", elapsed_ms())) << std::endl;
  }
  double elapsed_ms() const {
    return double(std::chrono::duration_cast<std::chrono::nanoseconds>(std::chrono::system_clock::now() - start).count()) * 1e-6;
  }
  double elapsed_us() const {
    return double(std::chrono::duration_cast<std::chrono::nanoseconds>(std::chrono::system_clock::now() - start).count()) * 1e-3;
  }
  double elapsed_ns() const {
    return double(std::chrono::duration_cast<std::chrono::nanoseconds>(std::chrono::system_clock::now() - start).count());
  }
private:
  const char *name;
  std::chrono::system_clock::time_point start;
  bool output;
};

void bench_timers();

// vim:ts=2 sw=2 sts=2 et ci
