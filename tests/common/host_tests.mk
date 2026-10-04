# Shared rules for the host unit tests. A suite's Makefile sets NAME and
# FIRMWARE_SRCS, then includes this file.
#
#   make -C tests/<suite>              build and run all tests
#   make -C tests/<suite> FILTER=hold  run only tests whose name contains "hold"
#   make -C tests/<suite> clean
#
# The suites sit outside firmware/ so arduino-cli never compiles them.

CXXFLAGS := -std=c++20 -Wall -Wextra -Werror -O1 -g
FIRMWARE := ../../firmware
COMMON := ../common
CPPFLAGS := -I. -I$(COMMON) -I$(FIRMWARE)

TEST_SRCS := $(wildcard *.cpp)
BUILD := build
OBJS := $(TEST_SRCS:%.cpp=$(BUILD)/%.o) $(BUILD)/common/test_main.o \
        $(FIRMWARE_SRCS:%.cpp=$(BUILD)/fw/%.o)
BIN := $(BUILD)/$(NAME)

.PHONY: all test clean

all: test

test: $(BIN)
	./$(BIN) $(FILTER)

$(BIN): $(OBJS)
	$(CXX) $(CXXFLAGS) -o $@ $^

$(BUILD)/%.o: %.cpp
	@mkdir -p $(@D)
	$(CXX) $(CPPFLAGS) $(CXXFLAGS) -MMD -MP -c -o $@ $<

$(BUILD)/common/%.o: $(COMMON)/%.cpp
	@mkdir -p $(@D)
	$(CXX) $(CPPFLAGS) $(CXXFLAGS) -MMD -MP -c -o $@ $<

$(BUILD)/fw/%.o: $(FIRMWARE)/%.cpp
	@mkdir -p $(@D)
	$(CXX) $(CPPFLAGS) $(CXXFLAGS) -MMD -MP -c -o $@ $<

clean:
	rm -rf $(BUILD)

-include $(OBJS:.o=.d)
