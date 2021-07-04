#include "stdafx.h"
#include "tapeengine.h"

#include <string>
#include <vector>

#include <gtest/gtest.h>

using namespace NTapeEngine;

TEST(TapeEngineTest, TapeEngineTest) {
  {
    TapeEngine tape("ap inc 0");
    tape.eval();
    tape.print();
    EXPECT_EQ(*tape.get_value(), 1);
  }
  {
    TapeEngine tape("ap inc ap inc 0");
    tape.eval();
    tape.print();
    EXPECT_EQ(*tape.get_value(), 2);
  }
}

// vim:ts=2 sw=2 sts=2 et ci

