#include "stdafx.h"
#include "nazoengine.h"

#include <stack>
#include <sstream>
#include <string>

#include <gtest/gtest.h>

#include "util.h"

class TestNazogengo : public testing::Test {
 public:
  std::string eval(const std::string& expr) {
    return engine_.parse_expr(expr)->eval()->repr(false);
  }
  void parse(const std::string& expr) {
    return engine_.parse_statement(expr);
  }
  bool equivalent(const std::string& lhs, const std::string & rhs) {
    // test if (modulated lhs) == (modulated rhs)
    return eval(fmt::format("ap ap eq ap mod {} ap mod {}", lhs, rhs)) == "t";
  }
  Engine& engine() { return engine_; }

 private:
  Engine engine_;
};

TEST_F(TestNazogengo, Parsing) {
  std::string tests[] = {
    "ap inc 0",
    "ap inc ap inc 0",
    "ap ap add 1 2",
    "ap ap mul 4 2",
    "ap ap div 5 -3",
    "ap ap ap b inc dec 1",
  };
  for (auto&& input : tests) {
    auto r = engine().parse_expr(input);
    EXPECT_EQ(input, r->repr(false));
  }

  {
    auto r = engine().parse_expr("ap ap ap b inc dec 1");
    EXPECT_EQ(r->repr(true), "{ap {ap {ap b inc} dec} 1}");
  }
}

TEST_F(TestNazogengo, EvaluateBasicTreeStructure) {
  // for debugging eval(). please keep this as simple as possible.
  struct TestData {
    std::string input;
    std::string expect;
  } test_data[] = {
    {"ap inc 0", "1"},
    {"ap ap add 1 2", "3"},
    {"ap ap add 1 ap inc 1", "3"},
    {"ap ap ap s add inc 1", "3"},
  };
  for (auto&& test_case : test_data) {
    EXPECT_EQ(eval(test_case.input), test_case.expect);
  }
}

TEST_F(TestNazogengo, BasicExpression) {
  struct TestData {
    std::string input;
    std::string expect;
  } test_data[] = {
    // https://message-from-space.readthedocs.io/en/latest/message5.html
    {"ap inc 0", "1"},
    {"ap inc 1", "2"},
    {"ap inc 2", "3"},
    {"ap inc 3", "4"},
    {"ap inc 300", "301"},
    {"ap inc 301", "302"},
    {"ap inc -1", "0"},
    {"ap inc -2", "-1"},
    {"ap inc -3", "-2"},
    // https://message-from-space.readthedocs.io/en/latest/message6.html
    {"ap dec 1", "0"},
    {"ap dec 2", "1"},
    {"ap dec 3", "2"},
    {"ap dec 4", "3"},
    {"ap dec 1024", "1023"},
    {"ap dec 0", "-1"},
    {"ap dec -1", "-2"},
    {"ap dec -2", "-3"},
    // https://message-from-space.readthedocs.io/en/latest/message7.html
    {"ap ap add 1 2", "3"},
    {"ap ap add 2 1", "3"},
    {"ap ap add 0 1", "1"},
    {"ap ap add 2 3", "5"},
    {"ap ap add 3 5", "8"},
    // https://message-from-space.readthedocs.io/en/latest/message8.html
    {"ap ap add 0 x0", "x0"},
    {"ap ap add 0 x1", "x1"},
    {"ap ap add 0 x2", "x2"},
    {"ap ap add x0 0", "x0"},
    {"ap ap add x1 0", "x1"},
    {"ap ap add x2 0", "x2"},
    //{"ap ap add x0 x1", "ap ap add x1 x0"},
    // https://message-from-space.readthedocs.io/en/latest/message9.html
    {"ap ap mul 4 2", "8"},
    {"ap ap mul 3 4", "12"},
    {"ap ap mul 3 -2", "-6"},
    // {"ap ap mul x0 x1", "ap ap mul x1 x0"},
    {"ap ap mul x0 0", "0"},
    {"ap ap mul x0 1", "x0"},
    // https://message-from-space.readthedocs.io/en/latest/message10.html
    {"ap ap div 4 2", "2"},
    {"ap ap div 4 3", "1"},
    {"ap ap div 4 4", "1"},
    {"ap ap div 4 5", "0"},
    {"ap ap div 5 2", "2"},
    {"ap ap div 6 -2", "-3"},
    {"ap ap div 5 -3", "-1"},
    {"ap ap div -5 3", "-1"},
    {"ap ap div -5 -3", "1"},
    {"ap ap div x0 1", "x0"},
    // https://message-from-space.readthedocs.io/en/latest/message11.html
    {"ap ap eq x0 x0", "t"},
    {"ap ap eq 0 -2", "f"},
    {"ap ap eq 0 -1", "f"},
    {"ap ap eq 0 0", "t"},
    {"ap ap eq 0 1", "f"},
    {"ap ap eq 0 2", "f"},
    {"ap ap eq 1 -1", "f"},
    {"ap ap eq 1 0", "f"},
    {"ap ap eq 1 1", "t"},
    {"ap ap eq 1 2", "f"},
    {"ap ap eq 1 3", "f"},
    {"ap ap eq 2 0", "f"},
    {"ap ap eq 2 1", "f"},
    {"ap ap eq 2 2", "t"},
    {"ap ap eq 2 3", "f"},
    {"ap ap eq 2 4", "f"},
    {"ap ap eq 19 20", "f"},
    {"ap ap eq 20 20", "t"},
    {"ap ap eq 21 20", "f"},
    {"ap ap eq -19 -20", "f"},
    {"ap ap eq -20 -20", "t"},
    {"ap ap eq -21 -20", "f"},
    // https://message-from-space.readthedocs.io/en/latest/message12.html
    {"ap ap lt 0 -1", "f"},
    {"ap ap lt 0 0", "f"},
    {"ap ap lt 0 1", "t"},
    {"ap ap lt 0 2", "t"},
    {"ap ap lt 1 0", "f"},
    {"ap ap lt 1 1", "f"},
    {"ap ap lt 1 2", "t"},
    {"ap ap lt 1 3", "t"},
    {"ap ap lt 2 1", "f"},
    {"ap ap lt 2 2", "f"},
    {"ap ap lt 2 3", "t"},
    {"ap ap lt 2 4", "t"},
    {"ap ap lt 19 20", "t"},
    {"ap ap lt 20 20", "f"},
    {"ap ap lt 21 20", "f"},
    {"ap ap lt -19 -20", "f"},
    {"ap ap lt -20 -20", "f"},
    {"ap ap lt -21 -20", "t"},
    // https://message-from-space.readthedocs.io/en/latest/message13.html
    {"ap mod 0", "[010]"},
    {"ap mod 1", "[01100001]"},
    {"ap mod -1", "[10100001]"},
    {"ap mod 2", "[01100010]"},
    {"ap mod -2", "[10100010]"},
    {"ap mod 16", "[0111000010000]"},
    {"ap mod -16", "[1011000010000]"},
    {"ap mod 255", "[0111011111111]"},
    {"ap mod -255", "[1011011111111]"},
    {"ap mod 256", "[011110000100000000]"},
    {"ap mod -256", "[101110000100000000]"},
    {"ap dem [010]", "0"},
    {"ap dem [01100001]", "1"},
    {"ap dem [10100001]", "-1"},
    {"ap dem [01100010]", "2"},
    {"ap dem [10100010]", "-2"},
    {"ap dem [0111000010000]", "16"},
    {"ap dem [1011000010000]", "-16"},
    {"ap dem [0111011111111]", "255"},
    {"ap dem [1011011111111]", "-255"},
    {"ap dem [011110000100000000]", "256"},
    {"ap dem [101110000100000000]", "-256"},
    // https://message-from-space.readthedocs.io/en/latest/message16.html
    {"ap neg 0", "0"},
    {"ap neg 1", "-1"},
    {"ap neg -1", "1"},
    {"ap neg 2", "-2"},
    {"ap neg -2", "2"},
    // https://message-from-space.readthedocs.io/en/latest/message17.html
    {"ap inc ap inc 0", "2"},
    {"ap inc ap inc ap inc 0", "3"},
    // {"ap inc ap dec x0", "x0"},
    // {"ap dec ap inc x0", "x0"},
    // {"ap dec ap ap add x0 1", "x0"},
    {"ap ap add ap ap add 2 3 4", "9"},
    {"ap ap add 2 ap ap add 3 4", "9"},
    {"ap ap add ap ap mul 2 3 4", "10"},
    {"ap ap mul 2 ap ap add 3 4", "14"},
    // {"inc", "ap add 1"},
    // {"dec", "ap add ap neg 1"},
    // https://message-from-space.readthedocs.io/en/latest/message18.html
    // {"ap ap ap s x0 x1 x2", "ap ap x0 x2 ap x1 x2"},
    {"ap ap ap s add inc 1", "3"},
    {"ap ap ap s mul ap add 1 6", "42"},
    // https://message-from-space.readthedocs.io/en/latest/message19.html
    // {"ap ap ap c x0 x1 x2", "ap ap x0 x2 x1"},
    {"ap ap ap c add 1 2", "3"},
    // https://message-from-space.readthedocs.io/en/latest/message20.html
    // {"ap ap ap b x0 x1 x2", "ap x0 ap x1 x2"},
    // {"ap ap ap b inc dec x0", "x0"},
    // https://message-from-space.readthedocs.io/en/latest/message21.html
    {"ap ap t x0 x1", "x0"},
    {"ap ap t 1 5", "1"},
    {"ap ap t t i", "t"},
    {"ap ap t t ap inc 5", "t"},
    {"ap ap t ap inc 5 t", "6"},
    // https://message-from-space.readthedocs.io/en/latest/message22.html
    {"ap ap f x0 x1", "x1"},
    // {"f", "ap s t"},
    // https://message-from-space.readthedocs.io/en/latest/message24.html
    {"ap i x0", "x0"},
    {"ap i 1", "1"},
    {"ap i i", "i"},
    {"ap i add", "add"},
    // {"ap i ap add 1", "ap add 1"},
    // https://message-from-space.readthedocs.io/en/latest/message25.html
    // {"ap ap ap cons x0 x1 x2", "ap ap x2 x0 x1"},
    // https://message-from-space.readthedocs.io/en/latest/message26.html
    {"ap car ap ap cons x0 x1", "x0"},
    // {"ap car x2", "ap x2 t"},
    // https://message-from-space.readthedocs.io/en/latest/message27.html
    {"ap cdr ap ap cons x0 x1", "x1"},
    // {"ap cdr x2", "ap x2 f"},
    // https://message-from-space.readthedocs.io/en/latest/message28.html
    {"ap nil x0", "t"},
    // https://message-from-space.readthedocs.io/en/latest/message29.html
    {"ap isnil nil", "t"},
    {"ap isnil ap ap cons x0 x1", "f"},
    // https://message-from-space.readthedocs.io/en/latest/message35.html
    {"ap mod nil", "[00]"},
    {"ap mod ap ap cons nil nil", "[110000]"},
    {"ap mod ap ap cons 0 nil", "[1101000]"},
    {"ap mod ap ap cons 1 2", "[110110000101100010]"},
    {"ap mod ap ap cons 1 ap ap cons 2 nil", "[1101100001110110001000]"},
    {"ap mod ( 1 , 2 )", "[1101100001110110001000]"},
    {"ap mod ( 1 , ( 2 , 3 ) , 4 )", "[1101100001111101100010110110001100110110010000]"},
    {"ap mod ap dem ap mod nil", "[00]"},
    {"ap mod ap dem ap mod ap ap cons nil nil", "[110000]"},
    {"ap mod ap dem ap mod ap ap cons 0 nil", "[1101000]"},
    {"ap mod ap dem ap mod ap ap cons 1 2", "[110110000101100010]"},
    {"ap mod ap dem ap mod ap ap cons 1 ap ap cons 2 nil", "[1101100001110110001000]"},
    {"ap mod ap dem ap mod ( 1 , 2 )", "[1101100001110110001000]"},
    {"ap mod ap dem ap mod ( 1 , ( 2 , 3 ) , 4 )", "[1101100001111101100010110110001100110110010000]"},

    {"ap ap ap b inc dec 1", "1"},
    {"ap ap t 1 2", "1"},
    {"ap ap f 1 2", "2"},
    {"ap i 1", "1"},
    {"ap isnil nil", "t"},
    {"ap isnil ap i nil", "t"},
    {"ap car ap ap cons 1 2", "1"},
    {"ap cdr ap ap cons 1 2", "2"},
    {"ap ap ap b inc dec 1", "1"},
    {"ap cdr ap i ap ap cons ap inc 0 ap dec 3", "2"},
    // https://message-from-space.readthedocs.io/en/latest/message31.html
    {"ap car ap ap vec x0 x1", "x0"}, // not listed above. copied from 27
  };
  for (auto&& test_case : test_data) {
    EXPECT_EQ(eval(test_case.input), test_case.expect);
  }

#if 0
  // TODO: Filter test cases to check performance.
  for (auto&& test_case : test_data) {
    auto ps = engine().start_performance_snapshot();
    eval(test_case.input);
    ps.stop();
    LOG(INFO) << fmt::format("{} <=> {} : {:.2f} us, {} nodes, {} evals", , test_case.expect, ps.elapsed_us(), ps.n_create_node, ps.n_eval);
  }
#endif
}

TEST_F(TestNazogengo, PowerOf2) {
  // https://message-from-space.readthedocs.io/en/latest/message23.html
  struct TestData {
    std::string input;
    std::string expect;
  } test_data[] = {
    {"ap pwr2 0", "1"},
    {"ap pwr2 1", "2"},
    {"ap pwr2 2", "4"},
    {"ap pwr2 3", "8"},
    {"ap pwr2 4", "16"},
    {"ap pwr2 5", "32"},
    {"ap pwr2 6", "64"},
    {"ap pwr2 7", "128"},
    {"ap pwr2 8", "256"},
  };
  for (auto&& test_case : test_data) {
    EXPECT_EQ(eval(test_case.input), test_case.expect);
  }
}

TEST_F(TestNazogengo, RValueVariable) {
  { // evaluation is stopped by the variable 
    EXPECT_EQ(eval("ap inc x0"), "ap inc x0");
    EXPECT_EQ(eval("ap inc 0"), "1");
  }

  { // same variable.
    EXPECT_EQ(eval("ap ap eq x0 x0"), "t");
  }
}

TEST_F(TestNazogengo, List) {
  struct TestData {
    std::string input;
    std::string expect;
  } data[] = {
    {"( )", "nil"},
    //{"( x0 )", "ap ap cons x0 nil"},
    //{"( x0 , x1 )", "ap ap cons x0 ap ap cons x1 nil"},
    //{"( x0 , x1 , x2 )", "ap ap cons x0 ap ap cons x1 ap ap cons x2 nil"},
    //{"( x0 , x1 , x2 , x5 )", "ap ap cons x0 ap ap cons x1 ap ap cons x2 ap ap cons x5 nil"},
    {"( x0 )", "( x0 )"},
    {"( x0 , x1 )", "( x0 , x1 )"},
    {"( x0 , x1 , x2 )", "( x0 , x1 , x2 )"},
    {"( x0 , x1 , x2 , x5 )", "( x0 , x1 , x2 , x5 )"},
  };
  parse("!list_repr");
  for(auto test_case : data) {
    EXPECT_EQ(eval(test_case.input), test_case.expect);
  }
}

TEST_F(TestNazogengo, Recursion) {
  // video 2.
  parse(":2048 = ap f :2048");
  EXPECT_EQ(eval("ap :2048 42"), "42");
}

TEST_F(TestNazogengo, Checkerboard) {
  // #33
  // Note: "[111101001000]" is not explicitly known.
  EXPECT_EQ(eval("ap mod ap ap checkerboard 1 0"), "[111101001000]");
}

TEST_F(TestNazogengo, MultipleDraw) {
  // #34
  EXPECT_EQ(eval("ap multipledraw nil"), "nil");
  parse("ap mod ap multipledraw ( )");
  parse("ap mod ap multipledraw ( ( ap ap vec 1 1 ) )");
  parse("ap mod ap multipledraw ( ( ap ap vec 0 0 , ap ap vec 1 1 ) , ( ap ap vec 1 2 , ap ap vec 1 3 ) )");

  // test equivalence to identity function.
  EXPECT_TRUE(equivalent("nil", "ap multipledraw ( )")); 
  parse(":input1 = ( ( ap ap vec 1 1 ) )");
  EXPECT_TRUE(equivalent(":input1", "ap multipledraw :input1")); 
  parse(":input2 = ( ( ap ap vec 0 0 , ap ap vec 1 1 ) , ( ap ap vec 1 2 , ap ap vec 1 3 ) )");
  EXPECT_TRUE(equivalent(":input2", "ap multipledraw :input2")); 

  // TODO: resolve partial node
  // EXPECT_EQ(eval("ap multipledraw ap ap cons x0 x1"),
  //           "ap ap cons ap draw x0 ap multipledraw x1");
}

TEST_F(TestNazogengo, StatelessDraw) {
  // #40
  parse(":result = ap ap statelessdraw nil 101");
  EXPECT_EQ(eval("ap car :result"), "0");
  EXPECT_EQ(eval("ap car ap cdr :result"), "nil");
  EXPECT_EQ(eval("ap car ap car ap car ap cdr ap cdr :result"), "101");
  // EXPECT_EQ(eval("ap ap statelessdraw nil x1"),
  //           eval("( 0 , nil , ( ( x1 ) ) )"));
}

TEST_F(TestNazogengo, StatefulDraw) {
  // #41
  parse(":result = ap ap statefuldraw 100 101");
  parse(":t2 = ap car ap cdr :result");
  parse(":t3 = ap car ap car ap cdr ap cdr :result");
  EXPECT_EQ(eval("ap car :result"), "0");
  EXPECT_EQ(eval("ap car :t2"), "101");
  EXPECT_EQ(eval("ap cdr :t2"), "100");
  EXPECT_EQ(eval("ap car :t3"), "101");
  EXPECT_EQ(eval("ap cdr :t3"), "100");
  // EXPECT_EQ(eval("ap ap :67108929 x0 x1"),
  //           eval("( 0 , ap ap cons x1 x0 , ( ap ap cons x1 x0 ) )"));
}

TEST_F(TestNazogengo, DebugPutPixelOnDummyPicture) {
  parse(":input = ap ap vec 1 2");
  parse("ap debug_putpixel_op :input");
  EXPECT_TRUE(equivalent(":input", "ap debug_putpixel_op :input"));
}

TEST_F(TestNazogengo, Draw) {
  // see console for the pictures.
  parse("ap mod ap draw ( )");
  parse("ap mod ap draw ( ap ap vec 1 1 )");
  parse("ap mod ap draw ( ap ap vec 0 0 , ap ap vec 1 1 , ap ap vec 1 2 , ap ap vec 1 3 )");
  // test equivalence to identity function.
  EXPECT_TRUE(equivalent("nil", "ap draw ( )")); 
  parse(":input1 = ( ap ap vec 1 1 )");
  EXPECT_TRUE(equivalent(":input1", "ap draw :input1")); 
  parse(":input2 = ( ap ap vec 0 0 , ap ap vec 1 1 , ap ap vec 1 2 , ap ap vec 1 3 )");
  EXPECT_TRUE(equivalent(":input2", "ap draw :input2")); 
}

// Disabled in devlopping ICFPC2021, to avoid an ambiguous file path.
// $ ./test --gtest_also_run_disabled_tests
TEST_F(TestNazogengo, DISABLED_Interact) {
  parse(":statelessdraw_result = ap ap ap interact statelessdraw nil ap ap vec 0 0");
  parse("ap mod :statelessdraw_result");
  // The working directory must be the root of the repository.
  parse("!load data/galaxy.txt");
  parse(":galaxy_result = ap ap ap interact galaxy nil ap ap vec 0 0");
  parse("ap mod :galaxy_result");
}
