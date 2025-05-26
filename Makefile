# Source files
C_SRCS += \
	./Src/main.c \
	./Src/pwm.c \
	./Src/led.c \
	./Src/syscalls.c \
	./Src/sysmem.c \
	./Src/system_stm32g0xx.c \
	./Src/motor.c \
	./Src/delay.c \
    ./Src/GPIO.c \
	./Src/timer.c \
	./Src/USART.c \
	./Src/test.c \
	./Src/Debug_dirver.c



S_SRCS += \
	./CMSIS/Startup/startup_stm32g070rbtx.s

OBJS_s += \
	./Build/startup_stm32g070rbtx.o

# Object files
OBJS = $(C_SRCS:./Src/%.c=./Build/%.o)

TEST_FILE = ./test/test_Encoder.c
TEST_OBJ = $(TEST_FILE:./test/%.c=./Build/%.o)
# Dependency files
C_DEPS = $(OBJS:.o=.d)
#inlcue path 
Inc = ./Inc/
# # Compiler flags
# CFLAGS = -mcpu=cortex-m0 -std=gnu11 -g3 -DDEBUG -DSTM32G0  -DSTM32G070xx \
# 		 -I./Inc -I./CMSIS/Include -I./CMSIS/STM32G0_Headers -O0 -ffunction-sections -fdata-sections \
# 		 -Wall -fstack-usage -MMD -MP --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb


# LD_flags =  -mcpu=cortex-m0 -g3 -DDEBUG -c -x assembler-with-cpp -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@" "$<"
 
# # Compiler command
# CC = arm-none-eabi-gcc
#
# ---- 
CFLAGS = -mcpu=cortex-m0plus -std=gnu11 -g3 -DDEBUG -DSTM32G070xx \
		 -I$(Inc) -I./CMSIS/Include -I./CMSIS/STM32G0_Headers -O0 -ffunction-sections -fdata-sections \
		 -Wall -fstack-usage -MMD -MP --specs=nano.specs -mthumb

ASFLAGS = -mcpu=cortex-m0plus -g3 -DDEBUG -c -x assembler-with-cpp -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mthumb

# Compiler command
CC = arm-none-eabi-gcc


# Rule to include all object files for the 'all' target
all: Template_proj.elf

bin: Template_proj.elf
	arm-none-eabi-objcopy -O binary Template_proj.elf Template_proj.bin 



.PHONY: test 
test: $(TEST_OBJ) $(OBJS_s)
	$(CC) $^ -o Template_proj.elf  -mcpu=cortex-m0plus -T"./CMSIS/STM32G070RBTX_FLASH.ld" \
	--specs=nosys.specs -Wl,--gc-sections -static --specs=nano.specs \
	-mthumb -Wl,--start-group -lc -lm -Wl,--end-group
	@echo 'Finished building target: $@'
	@echo ' '
$(TEST_OBJ): ./Build/%.o: ./test/%.c
	echo "called "
	$(CC) $(CFLAGS) -c $< -o $@
# target 
Template_proj.elf : $(OBJS) $(OBJS_s)
	$(CC) $^ -o $@ -mcpu=cortex-m0plus -T"./CMSIS/STM32G070RBTX_FLASH.ld" \
	--specs=nosys.specs -Wl,--gc-sections -static --specs=nano.specs \
	-mthumb -Wl,--start-group -lc -lm -Wl,--end-group
	@echo 'Finished building target: $@'
	@echo ' '
Flash: bin
	st-flash --reset write Template_proj.bin 0x8000000


# Rule to build object files from source files
$(OBJS): ./Build/%.o: ./Src/%.c
	$(CC) $(CFLAGS) -c $< -o $@

# Rule to build object files from assembly files
$(OBJS_s): $(S_SRCS)
	arm-none-eabi-gcc -mcpu=cortex-m0plus -g3 -DDEBUG -c -x assembler-with-cpp -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfloat-abi=soft -mthumb -o "$@" "$<"


# Phony target to prevent conflicts with files named 'clean'
.PHONY: clean

clean:
	rm -f $(OBJS) $(C_DEPS) $(OBJS_s) Template_proj.elf
