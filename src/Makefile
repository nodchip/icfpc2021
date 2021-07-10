LIB_PATH=$(abspath ../libs)
BUILD_PATH=build
GTEST_DIR=$(LIB_PATH)/googletest/googletest

CXXFLAGS+=-O2 -Wall -Wno-sign-compare -std=c++17 -march=native
CXXFLAGS+=-Wno-unused-variable -Wno-unused-function
CXXFLAGS+=-I.
CXXFLAGS+=-MMD -MP

# header-only libraries
CXXFLAGS+=-I$(LIB_PATH)/CLI11/include
CXXFLAGS+=-I$(LIB_PATH)/json/include
CXXFLAGS+=-I$(LIB_PATH)/cpp-httplib/include
CXXFLAGS+=-I$(LIB_PATH)/boost_1_76_0/

# enable this for HTTPS support.
#CXXFLAGS+=-DCPPHTTPLIB_OPENSSL_SUPPORT
#LDFLAGS+=-lssl -lcrypto

# pre-built libraries
CXXFLAGS+=-I$(LIB_PATH)/fmt/include
LDFLAGS+=-L$(LIB_PATH)/fmt/build -lfmt

CXXFLAGS+=-I$(LIB_PATH)/glog/src -I$(LIB_PATH)/glog_build
LDFLAGS+=-L$(LIB_PATH)/glog_build -Wl,-rpath $(LIB_PATH)/glog_build -lglog

#CXXFLAGS+=-I$(LIB_PATH)/mpir
#LDFLAGS+=-L$(LIB_PATH)/mpir/.libs -Wl,-rpath $(LIB_PATH)/mpir/.libs -lgmp -lgmpxx

CXXFLAGS+=-I/usr/include/opencv4
LDFLAGS+=-lopencv_core -lopencv_highgui -lopencv_imgproc

LDFLAGS+=-lpthread
#CXXFLAGS+=-g
#CXXFLAGS+=-pg
#CXXFLAGS+=-DNDEBUG

# add your cpp files here:
SRCS+=util.cpp contest_types.cpp judge.cpp solver_registry.cpp visual_editor.cpp
OBJS=$(SRCS:%.cpp=$(BUILD_PATH)/%.o)
DEPS=$(SRCS:%.cpp=$(BUILD_PATH)/%.d)

# add your stable header files here to include them in the precompiled header:
PCH_HEADERS=
PCH_SRC=
PCH_OUT=

# do not edit below:
MAIN_SRCS=$(SRCS) main.cpp
MAIN_OBJS=$(MAIN_SRCS:%.cpp=$(BUILD_PATH)/%.o)
MAIN_DEPS=$(MAIN_SRCS:%.cpp=$(BUILD_PATH)/%.d)

JUDGE_SRCS=$(SRCS) judge_main.cpp
JUDGE_OBJS=$(JUDGE_SRCS:%.cpp=$(BUILD_PATH)/%.o)
JUDGE_DEPS=$(JUDGE_SRCS:%.cpp=$(BUILD_PATH)/%.d)

SOLVER_SRCS=$(wildcard solvers/*.cpp)
SOLVER_OBJS=$(SOLVER_SRCS:%.cpp=$(BUILD_PATH)/%.o)
SOLVER_DEPS=$(SOLVER_SRCS:%.cpp=$(BUILD_PATH)/%.d)
SOLVER_BUILD_PATH=build/solvers

TEST_MPIR_SRCS=$(SRCS) test_mpir.cpp
TEST_MPIR_OBJS=$(TEST_MPIR_SRCS:%.cpp=$(BUILD_PATH)/%.o)
TEST_MPIR_DEPS=$(TEST_MPIR_SRCS:%.cpp=$(BUILD_PATH)/%.d)

GTEST_SRCS=$(GTEST_DIR)/src/gtest-all.cc $(GTEST_DIR)/src/gtest_main.cc
TEST_BUILD_PATH=build/tests
TEST_CXXFLAGS=-I$(GTEST_DIR) -I$(GTEST_DIR)/include
TEST_LDFLAGS=$(LDFLAGS) -lpthread
TEST_SRCS=$(wildcard tests/*.cpp)
TEST_OBJS=$(TEST_SRCS:%.cpp=$(BUILD_PATH)/%.o)
TEST_DEPS=$(TEST_SRCS:%.cpp=$(BUILD_PATH)/%.d)
$(TEST_OBJS): CXXFLAGS+=$(TEST_CXXFLAGS)

TARGETS=solver judge test # test_mpir

.PHONY: all
all: dirs $(TARGETS)

.PHONY: clean
clean:
	rm -fr $(BUILD_PATH) $(TARGETS) $(PCH_OUT)

.PHONY: logclean
logclean:
	rm -fr solver.log.*

.PHONY: dirs
dirs:
	@mkdir -p $(BUILD_PATH) $(SOLVER_BUILD_PATH) $(TEST_BUILD_PATH)

$(PCH_OUT): $(PCH_SRC) $(PCH_HEADERS)
	$(CXX) $(CXXFLAGS) -c $< -o $@
$(BUILD_PATH)/%.o: %.cpp $(PCH_OUT)
	$(CXX) $(CXXFLAGS) -c $< -o $@

solver: $(MAIN_OBJS) $(SOLVER_OBJS)
	$(CXX) $(CXXFLAGS) -Wall $^ -o $@ $(LDFLAGS)

judge: $(JUDGE_OBJS) $(SOLVER_OBJS)
	$(CXX) $(CXXFLAGS) -Wall $^ -o $@ $(LDFLAGS)

test_mpir: $(TEST_MPIR_OBJS)
	$(CXX) $(CXXFLAGS) -Wall $^ -o $@ $(LDFLAGS)

test: $(GTEST_SRCS) $(TEST_OBJS) $(OBJS)
	$(CXX) $(TEST_CXXFLAGS) $^ -o $@ $(TEST_LDFLAGS)

-include $(DEPS) $(MAIN_DEPS) $(JUDGE_DEPS) $(SOLVER_DEPS) $(TEST_DEPS)
