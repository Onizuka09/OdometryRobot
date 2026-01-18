###############################################################################
# Project Configuration
###############################################################################

PROJECT_NAME  = Template_proj
TARGET        = $(PROJECT_NAME).elf
BINARY        = $(PROJECT_NAME).bin

# MCU Configuration
MCU           = cortex-m0plus
FLOAT_ABI     = soft

###############################################################################
# Toolchain Configuration
###############################################################################

CC            = arm-none-eabi-gcc
OBJCOPY       = arm-none-eabi-objcopy
FLASH_TOOL    = st-flash

###############################################################################
# Directory Structure
###############################################################################

BUILD_DIR     = ./Build
SRC_DIR       = ./Src
TEST_DIR      = ./test
INC_DIR       = ./Inc
CMSIS_INC     = ./CMSIS/Include
CMSIS_HEADERS = ./CMSIS/STM32G0_Headers
STARTUP_DIR   = ./CMSIS/Startup
LINKER_DIR    = ./CMSIS

###############################################################################
# Source Files
###############################################################################

# C Source Files
C_SRCS = \
	$(SRC_DIR)/main.c \
	$(SRC_DIR)/pwm.c \
	$(SRC_DIR)/bsp.c \
	$(SRC_DIR)/syscalls.c \
	$(SRC_DIR)/sysmem.c \
	$(SRC_DIR)/system_stm32g0xx.c \
	$(SRC_DIR)/motor.c \
	$(SRC_DIR)/delay.c \
	$(SRC_DIR)/GPIO.c \
	$(SRC_DIR)/odometry.c \
	$(SRC_DIR)/encoder.c \
	$(SRC_DIR)/USART.c \
	$(SRC_DIR)/test.c \
	$(SRC_DIR)/odometry.c \
	$(SRC_DIR)/Debug_dirver.c \
	$(SRC_DIR)/navigation.c \
	$(SRC_DIR)/trapezoid.c \
	$(SRC_DIR)/system_clk.c

# Assembly Source Files
S_SRCS = \
	$(STARTUP_DIR)/startup_stm32g070rbtx.s

# Test Files
TEST_FILE = $(TEST_DIR)/test_Encoder.c

###############################################################################
# Object Files
###############################################################################

OBJS       = $(C_SRCS:$(SRC_DIR)/%.c=$(BUILD_DIR)/%.o)
OBJS_s     = $(S_SRCS:$(STARTUP_DIR)/%.s=$(BUILD_DIR)/%.o)
TEST_OBJ   = $(TEST_FILE:$(TEST_DIR)/%.c=$(BUILD_DIR)/%.o)

# Dependency files
C_DEPS = $(OBJS:.o=.d)

###############################################################################
# Compiler and Linker Flags
###############################################################################

# Common Flags
COMMON_FLAGS = -mcpu=$(MCU) -mthumb -mfloat-abi=$(FLOAT_ABI)

#          -O0 \
# Compiler Flags
CFLAGS = $(COMMON_FLAGS) \
         -std=gnu11 \
         -g3 \
         -DDEBUG \
         -DSTM32G070xx \
         -ffunction-sections \
         -fdata-sections \
         -Wall \
         -fstack-usage \
         -MMD \
         -MP \
         --specs=nano.specs

# Include Directories
CFLAGS += -I$(INC_DIR) \
          -I$(CMSIS_INC) \
          -I$(CMSIS_HEADERS)

# Assembly Flags
ASFLAGS = $(COMMON_FLAGS) \
          -g3 \
          -DDEBUG \
          -c \
          -x assembler-with-cpp \
          -MMD \
          -MP \
          -MF"$(@:%.o=%.d)" \
          -MT"$@" \
          --specs=nano.specs

# Linker Flags
LDFLAGS = $(COMMON_FLAGS) \
          -T"$(LINKER_DIR)/STM32G070RBTX_FLASH.ld" \
          --specs=nosys.specs \
          -Wl,--gc-sections \
          -static \
          --specs=nano.specs \
          -Wl,--start-group \
          -lc \
          -lm \
          -Wl,--end-group

###############################################################################
# Build Targets
###############################################################################

.PHONY: all bin test flash clean

# Default target
all: $(TARGET)

# Create binary
bin: $(BINARY)

# Test target
test: $(TEST_OBJ) $(OBJS_s)
	$(CC) $^ -o $(TARGET) $(LDFLAGS)
	@echo 'Finished building test target'

# Flash to device
flash: $(BINARY)
	$(FLASH_TOOL) --reset write $(BINARY) 0x8000000

# Clean build artifacts
clean:
	rm -f $(OBJS) $(C_DEPS) $(OBJS_s) $(TEST_OBJ) $(TARGET) $(BINARY)
	@echo 'Clean complete'

###############################################################################
# Build Rules
###############################################################################

# Main executable
$(TARGET): $(OBJS) $(OBJS_s)
	$(CC) $^ -o $@ $(LDFLAGS)
	@echo 'Finished building target: $@'

# Binary file
$(BINARY): $(TARGET)
	$(OBJCOPY) -O binary $< $@
	@echo 'Created binary: $@'

# Build C source files
$(BUILD_DIR)/%.o: $(SRC_DIR)/%.c | $(BUILD_DIR)
	$(CC) $(CFLAGS) -c $< -o $@

# Build test files
$(BUILD_DIR)/%.o: $(TEST_DIR)/%.c | $(BUILD_DIR)
	@echo "Building test file: $<"
	$(CC) $(CFLAGS) -c $< -o $@

# Build assembly files
$(BUILD_DIR)/%.o: $(STARTUP_DIR)/%.s | $(BUILD_DIR)
	$(CC) $(ASFLAGS) -o $@ $<

# Create build directory
$(BUILD_DIR):
	mkdir -p $(BUILD_DIR)

###############################################################################
# Include Dependencies
###############################################################################

-include $(C_DEPS)