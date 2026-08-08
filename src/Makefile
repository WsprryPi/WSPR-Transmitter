# Makefile for building most C++ or C++ and C projects

# Debug flag
DEBUG := -DDEBUG_WSPR_TRANSMIT
# C++ Version
CXXVER := 20
# Prefix for executing tests, "sudo" if required
SUDO := sudo
# Add any required submodules by their src path
SUBMODULE_SRCDIRS := $(wildcard ../../WSPR-Reference/src/wspr)
SUBMODULE_SRCDIRS += $(wildcard ../../Mailbox/src)
SUBMODULE_SRCDIRS += $(wildcard ../../Signal-Handler/src)
SUBMODULE_SRCDIRS += $(wildcard ../external)

# Get project name from Git, revert to directory name
#
# Project name from remote origin URL (fallback to 'unknown' if not available)
PRJ := $(shell git config --local remote.origin.url 2>/dev/null | sed -n 's#.*/\([^.]*\)\.git#\1#p')
# If PRJ is empty, fallback to current directory name
PRJ := $(or $(PRJ),$(shell basename $$(pwd)))
# If PRJ is "src", use the parent directory name
PRJ := $(shell [ "$(PRJ)" = "src" ] && basename $$(dirname $$(pwd)) || echo "$(PRJ)")
# Convert PRJ to lowercase for EXE_NAME
EXE_NAME := $(shell echo $(PRJ) | tr '[:upper:]' '[:lower:]')

# Output Items
OUT := $(EXE_NAME)					# Normal release binary
TEST_OUT :=	$(EXE_NAME)_test		# Debug/test binary
# Strip whitespace from comments
OUT := $(strip $(OUT))
TEST_OUT := $(strip $(TEST_OUT))

# Output directories
OBJ_DIR_RELEASE = build/obj/release
OBJ_DIR_DEBUG   = build/obj/debug
DEP_DIR         = build/dep
BIN_DIR		 	= build/bin

# ────────────────────────────────────────────────────────────────────────────────
# Find your project’s own .c/.cpp (this will pick up ./main.cpp or ./main.c):
LOCAL_C_SOURCES   := $(shell find . -type f -name '*.c')
LOCAL_CPP_SOURCES := $(shell find . -type f -name '*.cpp')
SI5351_TEST_SOURCE := ./si5351_backend_harness.cpp
SI5351_PLANNER_TEST_SOURCE := ./si5351_planner_test.cpp
SI5351_TRANSITION_TEST_SOURCE := ./si5351_transition_test.cpp
STARTUP_QUIESCE_TEST_SOURCE := ./startup_quiesce_test.cpp
SI5351_QUALIFICATION_SOURCE := ./si5351_startup_quiesce_qualification.cpp
SI5351_QUALIFICATION_MAIN_SOURCE := ./si5351_startup_quiesce_qualification_main.cpp
SI5351_QUALIFICATION_TEST_SOURCE := ./si5351_startup_quiesce_qualification_test.cpp
# Qualification sources use .cc intentionally: normal dependency and parent
# discovery include only .cpp files.  They are compiled solely by explicit
# guarded qualification targets below.
GPIO_QUALIFICATION_SOURCE := ./gpio_startup_quiesce_qualification.cc
GPIO_QUALIFICATION_MAIN_SOURCE := ./gpio_startup_quiesce_qualification_main.cc
GPIO_QUALIFICATION_TEST_SOURCE := ./gpio_startup_quiesce_qualification_test.cc
SI5351_MIN_DRIVE_SOURCE := ./si5351_min_drive_qualification.cc
SI5351_MIN_DRIVE_MAIN_SOURCE := ./si5351_min_drive_qualification_main.cc
SI5351_MIN_DRIVE_TEST_SOURCE := ./si5351_min_drive_qualification_test.cc
LOCAL_CPP_SOURCES := $(filter-out $(SI5351_TEST_SOURCE) $(SI5351_PLANNER_TEST_SOURCE) $(SI5351_TRANSITION_TEST_SOURCE) $(STARTUP_QUIESCE_TEST_SOURCE) $(SI5351_QUALIFICATION_SOURCE) $(SI5351_QUALIFICATION_MAIN_SOURCE) $(SI5351_QUALIFICATION_TEST_SOURCE),$(LOCAL_CPP_SOURCES))

# Find in each submodule—but explicitly skip any “main” there:
SUBMODULE_C_SOURCES   := $(shell find $(SUBMODULE_SRCDIRS) -type f -name '*.c'   ! -name 'main.c')
SUBMODULE_CPP_SOURCES := $(shell find $(SUBMODULE_SRCDIRS) -type f -name '*.cpp' ! -name 'main.cpp')

# Combine them:
C_SOURCES   := $(LOCAL_C_SOURCES)   $(SUBMODULE_C_SOURCES)
CPP_SOURCES := $(LOCAL_CPP_SOURCES) $(SUBMODULE_CPP_SOURCES)

# Strip any leading ../../ from the submodule paths so your object rules work:
REL_C_SOURCES   := $(patsubst ../../%,%,$(C_SOURCES))
REL_CPP_SOURCES := $(patsubst ../../%,%,$(CPP_SOURCES))

# Regenerate your object‐lists exactly as before:
C_OBJECTS         := $(patsubst %.c,   $(OBJ_DIR_RELEASE)/%.o,$(REL_C_SOURCES))
CPP_OBJECTS       := $(patsubst %.cpp, $(OBJ_DIR_RELEASE)/%.o,$(REL_CPP_SOURCES))
C_DEBUG_OBJECTS   := $(patsubst %.c,   $(OBJ_DIR_DEBUG)/%.o,$(REL_C_SOURCES))
CPP_DEBUG_OBJECTS := $(patsubst %.cpp, $(OBJ_DIR_DEBUG)/%.o,$(REL_CPP_SOURCES))
SI5351_TEST_OBJECTS := \
	$(OBJ_DIR_DEBUG)/./si5351_backend_harness.o \
	$(OBJ_DIR_DEBUG)/./wspr_transmit_backend_si5351.o \
	$(OBJ_DIR_DEBUG)/./si5351_device.o \
	$(OBJ_DIR_DEBUG)/./si5351_planner.o
SI5351_PLANNER_TEST_OBJECTS := \
	$(OBJ_DIR_DEBUG)/./si5351_planner_test.o \
	$(OBJ_DIR_DEBUG)/./si5351_planner.o
SI5351_TRANSITION_TEST_OBJECTS := \
	$(OBJ_DIR_DEBUG)/./si5351_transition_test.o \
	$(OBJ_DIR_DEBUG)/./wspr_transmit_backend_si5351.o \
	$(OBJ_DIR_DEBUG)/./si5351_device.o \
	$(OBJ_DIR_DEBUG)/./si5351_planner.o
STARTUP_QUIESCE_TEST_OBJECTS := \
	$(OBJ_DIR_DEBUG)/./startup_quiesce_test.o \
	$(OBJ_DIR_DEBUG)/./transmission_controller.o \
	$(OBJ_DIR_DEBUG)/./wspr_transmit_backend_rpi.o \
	$(OBJ_DIR_DEBUG)/./wspr_transmit_backend_si5351.o \
	$(OBJ_DIR_DEBUG)/./si5351_device.o \
	$(OBJ_DIR_DEBUG)/./si5351_planner.o \
	$(OBJ_DIR_DEBUG)/Mailbox/src/mailbox.o
SI5351_QUALIFICATION_OBJECTS := \
	$(OBJ_DIR_DEBUG)/./si5351_startup_quiesce_qualification.o \
	$(OBJ_DIR_DEBUG)/./wspr_transmit_backend_si5351.o \
	$(OBJ_DIR_DEBUG)/./si5351_device.o \
	$(OBJ_DIR_DEBUG)/./si5351_planner.o
SI5351_MIN_DRIVE_OBJECTS := \
	$(OBJ_DIR_DEBUG)/./si5351_min_drive_qualification.o \
	$(OBJ_DIR_DEBUG)/./si5351_device.o \
	$(OBJ_DIR_DEBUG)/./si5351_planner.o
$(OBJ_DIR_DEBUG)/./si5351_min_drive_qualification.o \
$(OBJ_DIR_DEBUG)/./si5351_min_drive_qualification_main.o \
$(OBJ_DIR_DEBUG)/./si5351_min_drive_qualification_test.o: \
	si5351_min_drive_qualification.hpp
GPIO_QUALIFICATION_OBJECTS := \
	$(OBJ_DIR_DEBUG)/./gpio_startup_quiesce_qualification.o \
	$(OBJ_DIR_DEBUG)/./wspr_transmit_backend_rpi.o \
	$(OBJ_DIR_DEBUG)/Mailbox/src/mailbox.o
# ────────────────────────────────────────────────────────────────────────────────

# Linker Flags
LDFLAGS := -lpthread  -latomic
# Get packages for linker from PKG_CONFIG_PATH
# LDFLAGS += $(shell pkg-config --cflags --libs libgpiod)
# LDFLAGS += $(shell pkg-config --libs libgpiodcxx)
# Strip whitespace from LDFLAGS
LDFLAGS := $(strip $(LDFLAGS))

# Collect dependency files
DEPFILES := $(shell find $(DEP_DIR) -type f -name '*.d' 2>/dev/null)
-include $(DEPFILES)

# Compiler Executables
CXX = g++
CC = gcc

# Allow threading using all available processors
MAKEFLAGS := -j$(nproc)

# Compiler Flags
#
# Common flags for both C and C++ compilers
COMMON_FLAGS := -Wall -Werror -fmax-errors=10 -MMD -MP

# C Flags
CFLAGS := $(COMMON_FLAGS)
# C Debug Flags
C_DEBUG_FLAGS := $(CFLAGS) -g $(DEBUG)	# Debug flags
# C Release Flags
C_RELEASE_FLAGS := $(CFLAGS) -O2		# Release optimized

# C++ Flags
CXXFLAGS := -Wno-psabi -lstdc++fs -std=c++$(CXXVER)
CXXFLAGS += $(COMMON_FLAGS) $(COMM_CXX_FLAGS)

# Include paths
CXXFLAGS += -I$(abspath .)
CXXFLAGS += -I$(abspath ../..)
CXXFLAGS += $(foreach dir,$(SUBMODULE_SRCDIRS),-I$(abspath $(dir)))
CXXFLAGS += -I$(abspath ../../WSPR-Reference/src/wspr)

# C++ Debug Flags
CXX_DEBUG_FLAGS := $(CXXFLAGS) -g $(DEBUG)	# Debug flags
# C++ Release Flags
CXX_RELEASE_FLAGS := $(CXXFLAGS) -O2		# Release optimized

# Strip whitespaces
C_DEBUG_FLAGS := $(strip $(C_DEBUG_FLAGS))
C_RELEASE_FLAGS := $(strip $(C_RELEASE_FLAGS))
CXX_DEBUG_FLAGS := $(strip $(CXX_DEBUG_FLAGS))
CXX_RELEASE_FLAGS := $(strip $(CXX_RELEASE_FLAGS))

# Enable verbose output if VERBOSE=1 is specified during the build
VERBOSE ?= 0
ifeq ($(VERBOSE), 1)
	Q :=
else
	Q := @
endif

##
# Compile & link rules
##

# Generic debug compile (your own code)
$(OBJ_DIR_DEBUG)/%.o: %.c
	$(Q)mkdir -p $(dir $@)
	$(Q)mkdir -p $(DEP_DIR)/$(dir $*)
	$(Q)echo "Compiling (debug) $< into $@"
	$(Q)$(CC) $(C_DEBUG_FLAGS) -MF $(DEP_DIR)/$*.d -c $< -o $@

$(OBJ_DIR_DEBUG)/%.o: %.cpp
	$(Q)mkdir -p $(dir $@)
	$(Q)mkdir -p $(DEP_DIR)/$(dir $*)
	$(Q)echo "Compiling (debug) $< into $@"
	$(Q)$(CXX) $(CXX_DEBUG_FLAGS) -MF $(DEP_DIR)/$*.d -c $< -o $@

# Explicitly named qualification-only .cc sources are never broadly discovered.
$(OBJ_DIR_DEBUG)/%.o: %.cc
	$(Q)mkdir -p $(dir $@)
	$(Q)mkdir -p $(DEP_DIR)/$(dir $*)
	$(Q)echo "Compiling qualification (debug) $< into $@"
	$(Q)$(CXX) $(CXX_DEBUG_FLAGS) -MF $(DEP_DIR)/$*.d -c $< -o $@

# Auto‑generate debug compile rules for each submodule
define MAKE_DEBUG_SUBMOD_RULES
$(OBJ_DIR_DEBUG)/$(patsubst ../../%,%,$(1))/%.o: $(1)/%.cpp
	$(Q)mkdir -p $$(@D)
	$(Q)mkdir -p $(DEP_DIR)/$(dir $$*)
	$(Q)echo "Compiling (debug) $$< into $$@"
	$(Q)$(CXX) $(CXX_DEBUG_FLAGS) -MF $(DEP_DIR)/$$*.d -c $$< -o $$@

$(OBJ_DIR_DEBUG)/$(patsubst ../../%,%,$(1))/%.o: $(1)/%.c
	$(Q)mkdir -p $$(@D)
	$(Q)mkdir -p $(DEP_DIR)/$(dir $$*)
	$(Q)echo "Compiling (debug) $$< into $$@"
	$(Q)$(CC)  $(C_DEBUG_FLAGS)   -MF $(DEP_DIR)/$$*.d -c $$< -o $$@
endef

$(foreach dir,$(SUBMODULE_SRCDIRS),$(eval $(call MAKE_DEBUG_SUBMOD_RULES,$(dir))))

# Link debug binary
build/bin/$(TEST_OUT): $(CPP_DEBUG_OBJECTS) $(C_DEBUG_OBJECTS)
	$(Q)mkdir -p $(BIN_DIR)
	$(Q)echo "Linking debug: $(TEST_OUT)"
	$(Q)$(CXX) $(CXX_DEBUG_FLAGS) $^ -o $@ $(LDFLAGS)

# Link temporary Si5351 backend harness
build/bin/si5351_backend_harness: $(SI5351_TEST_OBJECTS)
	$(Q)mkdir -p $(BIN_DIR)
	$(Q)echo "Linking debug: si5351_backend_harness"
	$(Q)$(CXX) $(CXX_DEBUG_FLAGS) $^ -o $@ $(LDFLAGS)

build/bin/si5351_planner_test: $(SI5351_PLANNER_TEST_OBJECTS)
	$(Q)mkdir -p $(BIN_DIR)
	$(Q)echo "Linking debug: si5351_planner_test"
	$(Q)$(CXX) $(CXX_DEBUG_FLAGS) $^ -o $@ $(LDFLAGS)

build/bin/si5351_transition_test: $(SI5351_TRANSITION_TEST_OBJECTS)
	$(Q)mkdir -p $(BIN_DIR)
	$(Q)echo "Linking debug: si5351_transition_test"
	$(Q)$(CXX) $(CXX_DEBUG_FLAGS) $^ -o $@ $(LDFLAGS)

build/bin/startup_quiesce_test: $(STARTUP_QUIESCE_TEST_OBJECTS)
	$(Q)mkdir -p $(BIN_DIR)
	$(Q)echo "Linking debug: startup_quiesce_test"
	$(Q)$(CXX) $(CXX_DEBUG_FLAGS) $^ -o $@ $(LDFLAGS)

build/bin/si5351_startup_quiesce_qualification: $(OBJ_DIR_DEBUG)/./si5351_startup_quiesce_qualification_main.o $(SI5351_QUALIFICATION_OBJECTS)

	$(Q)mkdir -p $(BIN_DIR)
	$(Q)$(CXX) $(CXX_DEBUG_FLAGS) $^ -o $@ $(LDFLAGS)

build/bin/si5351_startup_quiesce_qualification_test: $(OBJ_DIR_DEBUG)/./si5351_startup_quiesce_qualification_test.o $(SI5351_QUALIFICATION_OBJECTS)

	$(Q)mkdir -p $(BIN_DIR)
	$(Q)$(CXX) $(CXX_DEBUG_FLAGS) $^ -o $@ $(LDFLAGS)

build/bin/si5351_min_drive_qualification: $(OBJ_DIR_DEBUG)/./si5351_min_drive_qualification_main.o $(SI5351_MIN_DRIVE_OBJECTS)
	$(Q)mkdir -p $(BIN_DIR)
	$(Q)$(CXX) $(CXX_DEBUG_FLAGS) $^ -o $@ $(LDFLAGS)

build/bin/si5351_min_drive_qualification_test: $(OBJ_DIR_DEBUG)/./si5351_min_drive_qualification_test.o $(SI5351_MIN_DRIVE_OBJECTS)
	$(Q)mkdir -p $(BIN_DIR)
	$(Q)$(CXX) $(CXX_DEBUG_FLAGS) $^ -o $@ $(LDFLAGS)

build/bin/gpio_startup_quiesce_qualification: $(OBJ_DIR_DEBUG)/./gpio_startup_quiesce_qualification_main.o $(GPIO_QUALIFICATION_OBJECTS)
	$(Q)mkdir -p $(BIN_DIR)
	$(Q)echo "Linking guarded live GPIO startup-quiesce qualification executable"
	$(Q)$(CXX) $(CXX_DEBUG_FLAGS) $^ -o $@ $(LDFLAGS)

build/bin/gpio_startup_quiesce_qualification_test: $(OBJ_DIR_DEBUG)/./gpio_startup_quiesce_qualification_test.o $(GPIO_QUALIFICATION_OBJECTS)
	$(Q)mkdir -p $(BIN_DIR)
	$(Q)echo "Linking fake GPIO startup-quiesce qualification tests"
	$(Q)$(CXX) $(CXX_DEBUG_FLAGS) $^ -o $@ $(LDFLAGS)

# Generic release compile (your own code)
$(OBJ_DIR_RELEASE)/%.o: %.cpp
	$(Q)mkdir -p $(dir $@)
	$(Q)mkdir -p $(DEP_DIR)/$(dir $*)
	$(Q)echo "Compiling (release) $< into $@"
	$(Q)$(CXX) $(CXX_RELEASE_FLAGS) -MF $(DEP_DIR)/$*.d -c $< -o $@

$(OBJ_DIR_RELEASE)/%.o: %.c
	$(Q)mkdir -p $(dir $@)
	$(Q)mkdir -p $(DEP_DIR)/$(dir $*)
	$(Q)echo "Compiling (release) $< into $@"
	$(Q)$(CC) $(C_RELEASE_FLAGS) -MF $(DEP_DIR)/$*.d -c $< -o $@

# Auto‑generate release compile rules for each submodule
define MAKE_RELEASE_SUBMOD_RULES
$(OBJ_DIR_RELEASE)/$(patsubst ../../%,%,$(1))/%.o: $(1)/%.cpp
	$(Q)mkdir -p $$(@D)
	$(Q)mkdir -p $(DEP_DIR)/$(dir $$*)
	$(Q)echo "Compiling (release) $$< into $$@"
	$(Q)$(CXX) $(CXX_RELEASE_FLAGS) -MF $(DEP_DIR)/$$*.d -c $$< -o $$@

$(OBJ_DIR_RELEASE)/$(patsubst ../../%,%,$(1))/%.o: $(1)/%.c
	$(Q)mkdir -p $$(@D)
	$(Q)mkdir -p $(DEP_DIR)/$(dir $$*)
	$(Q)mkdir -p $(DEP_DIR)/$(dir $$*)
	$(Q)echo "Compiling (release) $$< into $$@"
	$(Q)$(CC)  $(C_RELEASE_FLAGS)   -MF $(DEP_DIR)/$$*.d -c $$< -o $$@
endef

$(foreach dir,$(SUBMODULE_SRCDIRS),$(eval $(call MAKE_RELEASE_SUBMOD_RULES,$(dir))))

# Link release binary
build/bin/$(OUT): $(CPP_OBJECTS) $(C_OBJECTS)
	$(Q)mkdir -p $(BIN_DIR)
	$(Q)echo "Linking release: $(OUT)"
	$(Q)$(CXX) $(CXX_RELEASE_FLAGS) $^ -o $@ $(LDFLAGS)

##
# Phony targets
##
.PHONY: clean release debug test si5351-test si5351-planner-test si5351-transition-test startup-quiesce-test si5351-startup-quiesce-qualification si5351-startup-quiesce-qualification-test si5351-min-drive-qualification si5351-min-drive-qualification-test gpio-startup-quiesce-qualification gpio-startup-quiesce-qualification-test gdb lint macros help

clean:
	$(Q)echo "Cleaning up build artifacts."
	$(Q)rm -rf build

release: build/bin/$(OUT)
	$(Q)echo "Release build completed successfully."

debug: build/bin/$(TEST_OUT)
	$(Q)echo "Debug build completed successfully."

test: debug
	$(Q)$(SUDO) ./build/bin/$(TEST_OUT) --now --oneshot

si5351-test: build/bin/si5351_backend_harness
	$(Q)echo "Si5351 backend harness built successfully."

si5351-planner-test: build/bin/si5351_planner_test
	$(Q)./build/bin/si5351_planner_test

si5351-transition-test: build/bin/si5351_transition_test
	$(Q)./build/bin/si5351_transition_test

startup-quiesce-test: build/bin/startup_quiesce_test
	$(Q)./build/bin/startup_quiesce_test

si5351-startup-quiesce-qualification: build/bin/si5351_startup_quiesce_qualification
	$(Q)echo "Built live qualification executable; it is not run by this target."

si5351-startup-quiesce-qualification-test: build/bin/si5351_startup_quiesce_qualification_test
	$(Q)./build/bin/si5351_startup_quiesce_qualification_test

si5351-min-drive-qualification: build/bin/si5351_min_drive_qualification
	$(Q)echo "Built guarded one-burst live qualification executable; it is not run by this target."

si5351-min-drive-qualification-test: build/bin/si5351_min_drive_qualification_test
	$(Q)./build/bin/si5351_min_drive_qualification_test

gpio-startup-quiesce-qualification: build/bin/gpio_startup_quiesce_qualification
	$(Q)echo "Built guarded live GPIO startup-quiesce qualification executable; it is not run by this target."

gpio-startup-quiesce-qualification-test: build/bin/gpio_startup_quiesce_qualification_test
	$(Q)./build/bin/gpio_startup_quiesce_qualification_test

watchdog: debug
	$(Q)$(SUDO) env WSPR_TX_INJECT_WD_STALL=5 ./build/bin/$(TEST_OUT) --now --oneshot

gdb: debug
	$(Q)$(SUDO) gdb --args ./build/bin/$(TEST_OUT)

lint:
	$(Q)command -v cppcheck >/dev/null 2>&1 || { \
	  echo "Warning: cppcheck not installed."; exit 1; }
	$(Q)echo "Running cppcheck…"
	$(Q)cppcheck --std=c++$(CXXVER) --enable=all --inconclusive \
	      --force --inline-suppr --quiet $(CPP_SOURCES)

macros:
	$(Q)echo "Project macros:"
	$(Q)$(CXX) $(CXXFLAGS) -dM -E -x c++ /dev/null \
	      | grep -v '^#define _' || true

help:
	$(Q)echo "Available targets:"
	$(Q)echo "  release    Build optimized binary"
	$(Q)echo "  debug      Build debug binary"
	$(Q)echo "  test       Run tests"
	$(Q)echo "  si5351-test Build temporary Si5351 backend harness"
	$(Q)echo "  si5351-planner-test Build and run non-hardware planner tests"
	$(Q)echo "  si5351-transition-test Build and run fake-I2C transition tests"
	$(Q)echo "  startup-quiesce-test Build and run non-hardware startup-quiesce tests"
	$(Q)echo "  gpio-startup-quiesce-qualification Build guarded live qualification executable only"
	$(Q)echo "  gpio-startup-quiesce-qualification-test Run fake qualification tests"
	$(Q)echo "  watchdog   Run tests with watchdog fail"
	$(Q)echo "  gdb        Debug with gdb"
	$(Q)echo "  lint       Static analysis"
	$(Q)echo "  macros     Show macros"
	$(Q)echo "  clean      Remove build artifacts"
	$(Q)echo "  help       This message"
