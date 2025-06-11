# ==== Configuration ====

# Build mode: debug (default), release, profile
BUILD_MODE ?= debug

# Build directories
BUILD_DIR := build/$(BUILD_MODE)
TEST_DIR := test

# Source directories
SRC_DIR := src
INCLUDE_DIR := include
THIRDPARTY_DIR := thirdparty

# Third-party sources
UNITY := $(THIRDPARTY_DIR)/unity/unity.c
LOG := $(THIRDPARTY_DIR)/log/log.c

# Compiler and flags
CFLAGS := -Wall -Wextra -std=c99 -I$(INCLUDE_DIR)

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

LD_FLAGS := -lm -lglfw -lGL

# ==== Files ====

# Source and object files
SRC := $(wildcard $(SRC_DIR)/*.c)
OBJ := $(filter-out $(BUILD_DIR)/main.o, $(SRC:$(SRC_DIR)/%.c=$(BUILD_DIR)/%.o))

# Test sources and executables
TEST_SRC := $(wildcard $(TEST_DIR)/*.c)
TEST_BINS := $(TEST_SRC:$(TEST_DIR)/%.c=$(BUILD_DIR)/test/%)

# Targets
APP := $(BUILD_DIR)/microslam
LIB := $(BUILD_DIR)/libmicroslam.a

# ==== Build Rules ====

.PHONY: all
all: $(APP)

# Main app
$(APP): $(LIB) $(BUILD_DIR)/main.o
	$(CC) $(CFLAGS) -o $@ $^ $(LIB) $(LD_FLAGS)

# Static library
$(LIB): $(OBJ) $(BUILD_DIR)/log.o $(INCLUDE_DIR)/microslam/*.h
	@mkdir -p $(@D)
	$(AR) rcs $@ $^

# Object files
$(BUILD_DIR)/%.o: $(SRC_DIR)/%.c
	@mkdir -p $(@D)
	$(CC) $(CFLAGS) -I$(THIRDPARTY_DIR) -c -o $@ $<

$(BUILD_DIR)/main.o: $(SRC_DIR)/main.c
	@mkdir -p $(@D)
	$(CC) $(CFLAGS) -c -o $@ $<

$(BUILD_DIR)/log.o: $(LOG)
	@mkdir -p $(@D)
	$(CC) $(CFLAGS) -c -o $@ $<

$(BUILD_DIR)/unity.o: $(UNITY)
	@mkdir -p $(@D)
	$(CC) $(CFLAGS) -c -o $@ $<

# ==== Tests ====

.PHONY: test
test: $(TEST_BINS)
	@for t in $(TEST_BINS); do \
		echo "Running $$t..."; \
		$$t || exit 1; \
	done

$(BUILD_DIR)/test/%: $(TEST_DIR)/%.c $(BUILD_DIR)/unity.o $(LIB)
	@mkdir -p $(@D)
	$(CC) $(CFLAGS) -I$(THIRDPARTY_DIR) -o $@ $< $(BUILD_DIR)/unity.o $(LIB) $(LD_FLAGS)

# ==== Clean ====

.PHONY: clean
clean:
	rm -rf build $(EXTRA_CLEAN)
