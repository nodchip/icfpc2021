#include "stdafx.h"
#include "timer.h"
#include <chrono>
#include <iostream>

#include <fmt/format.h>

void bench_timers() {
  const int iter = 1000 * 1000;
#ifdef _WIN32
  {
    Timer t;
    LARGE_INTEGER li;
    for (int i = 0; i < iter; ++i) {
      QueryPerformanceCounter(&li);
    }
    std::cout << fmt::format("QueryPerformanceCounter: {0:.3f}ns/iter", t.elapsed_ns() / iter) << std::endl;
  }
#else
  {
    Timer t;
    timespec ts;
    for (int i = 0; i < iter; ++i) {
      clock_gettime(CLOCK_REALTIME, &ts);
    }
    std::cout << fmt::format("clock_gettime(CLOCK_REALTIME): {0:.3f}ns/iter", t.elapsed_ns() / iter) << std::endl;
  }
  {
    Timer t;
    timespec ts;
    for (int i = 0; i < iter; ++i) {
      clock_gettime(CLOCK_REALTIME_COARSE, &ts);
    }
    std::cout << fmt::format("clock_gettime(CLOCK_REALTIME_COARSE): {0:.3f}ns/iter", t.elapsed_ns() / iter) << std::endl;
  }
  {
    Timer t;
    timespec ts;
    for (int i = 0; i < iter; ++i) {
      clock_gettime(CLOCK_MONOTONIC_COARSE, &ts);
    }
    std::cout << fmt::format("clock_gettime(CLOCK_MONOTONIC_COARSE): {0:.3f}ns/iter", t.elapsed_ns() / iter) << std::endl;
  }
  {
    Timer t;
    timespec ts;
    for (int i = 0; i < iter; ++i) {
      clock_gettime(CLOCK_THREAD_CPUTIME_ID, &ts);
    }
    std::cout << fmt::format("clock_gettime(CLOCK_THREAD_CPUTIME_ID): {0:.3f}ns/iter", t.elapsed_ns() / iter) << std::endl;
  }
  {
    Timer t;
    timeval tv;
    for (int i = 0; i < iter; ++i) {
      gettimeofday(&tv, NULL);
    }
    std::cout << fmt::format("gettimeofday: {0:.3f}ns/iter", t.elapsed_ns() / iter) << std::endl;
  }
  {
    Timer t;
    clock_t c0 = clock();
    clock_t acc = 0;
    for (int i = 0; i < iter; ++i) {
      acc += clock();
    }
    clock_t c1 = clock();
    std::cout << fmt::format("clock: {0:.3f}ns/iter. CLOCKS_PER_SEC={1}, clock={2}us?, chrono={3:.1f}us",
                             t.elapsed_ns() / iter, CLOCKS_PER_SEC, c1 - c0, t.elapsed_us())
              << std::endl;
  }
#endif
  {
    Timer t;
    for (int i = 0; i < iter; ++i) {
      (void)std::chrono::system_clock::now();
    }
    std::cout << fmt::format("chrono::system_clock: {0:.3f}ns/iter",
                             t.elapsed_ns() / iter)
              << std::endl;
  }
}

// vim:ts=2 sw=2 sts=2 et ci
