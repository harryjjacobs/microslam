BUILD_MODE ?= debug

ifeq ($(BUILD_MODE),debug)
	CFLAGS += -g
else ifeq ($(BUILD_MODE),release)
	CFLAGS += -O3
else ifeq ($(BUILD_MODE),profile)
	CFLAGS += -pg -O3
	EXTRA_CLEAN += gmon.out
else
    $(error Build mode $(BUILD_MODE) not supported by this Makefile)
endif

BUILD_DIR = build/$(BUILD_MODE)

CFLAGS += -Wall -Wextra -std=c99 # -Werror
CFLAGS += -Iinclude

# source files
SRC = $(wildcard src/*.c)
OBJ = $(filter-out $(BUILD_DIR)/main.o, $(SRC:src/%.c=$(BUILD_DIR)/%.o))

# test source files
TEST_SRC = $(wildcard test/*.c)
TEST_OBJ = $(TEST_SRC:test/%.c=$(BUILD_DIR)/%.o)

# microslam library
LIB = $(BUILD_DIR)/libmicroslam.a

# test executable
TEST = $(BUILD_DIR)/test

# linker flags
LD_FLAGS = -lm -lglfw -lGL -lGLEW

# microslam application target
APP = $(BUILD_DIR)/microslam

# test target
TEST = $(BUILD_DIR)/test

# phony targets
.PHONY: all clean test

# default target
all: $(APP)

# log library
LOG = thirdparty/log/log.c

$(BUILD_DIR)/log.o: $(LOG)
	@mkdir -p $(@D)
	$(CC) $(CFLAGS) -c -o $@ $<

# build the microslam library
$(LIB): $(OBJ) $(BUILD_DIR)/log.o
	@mkdir -p $(@D)
	$(AR) rcs $@ $^

# build the microslam application
$(APP): $(LIB) $(BUILD_DIR)/main.o
	$(CC) $(CFLAGS) -o $@ $^ $(LIB) $(LD_FLAGS)

# unity (unit testing) library
UNITY = thirdparty/unity/unity.c

$(BUILD_DIR)/unity.o: $(UNITY)
	@mkdir -p $(@D)
	$(CC) $(CFLAGS) -c -o $@ $<

# build the test executable
$(TEST): $(LIB) $(TEST_OBJ) $(BUILD_DIR)/unity.o
	$(CC) $(CFLAGS) -o $@ $^ $(LIB) $(LD_FLAGS) -Ithirdparty/unity

# build object files
$(BUILD_DIR)/%.o: src/%.c
	@mkdir -p $(@D)
	$(CC) $(CFLAGS) -c -o $@ $< -Ithirdparty

# build test object files
$(BUILD_DIR)/%.o: test/%.c
	@mkdir -p $(@D)
	$(CC) $(CFLAGS) -c -o $@ $< -Ithirdparty

# clean build artifacts
clean:
	rm -rf build $(EXTRA_CLEAN)

# run the test suite
test: $(TEST)
	$(TEST)
