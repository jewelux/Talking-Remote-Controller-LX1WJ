# Host unit tests for the Arduino-free keypad logic.
#
#   make -C tests/keypad              build and run all tests
#   make -C tests/keypad FILTER=hold  run only tests whose name contains "hold"
#   make -C tests/keypad clean
#
# This folder sits outside firmware/ so arduino-cli never compiles it.

CXXFLAGS := -std=c++17 -Wall -Wextra -Werror -O1 -g
FIRMWARE := ../../firmware
CPPFLAGS := -I. -I$(FIRMWARE)

# Pure firmware sources under test. They must not include Arduino.h.
FIRMWARE_SRCS := keypad_input.cpp

TEST_SRCS := $(wildcard *.cpp)
BUILD := build
OBJS := $(TEST_SRCS:%.cpp=$(BUILD)/%.o) $(FIRMWARE_SRCS:%.cpp=$(BUILD)/fw/%.o)
BIN := $(BUILD)/keypad_tests

.PHONY: all test clean

all: test

test: $(BIN)
	./$(BIN) $(FILTER)

$(BIN): $(OBJS)
	$(CXX) $(CXXFLAGS) -o $@ $^

$(BUILD)/%.o: %.cpp
	@mkdir -p $(@D)
	$(CXX) $(CPPFLAGS) $(CXXFLAGS) -MMD -MP -c -o $@ $<

$(BUILD)/fw/%.o: $(FIRMWARE)/%.cpp
	@mkdir -p $(@D)
	$(CXX) $(CPPFLAGS) $(CXXFLAGS) -MMD -MP -c -o $@ $<

clean:
	rm -rf $(BUILD)

-include $(OBJS:.o=.d)
