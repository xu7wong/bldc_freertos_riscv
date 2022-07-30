################################################################################
# Automatically-generated file. Do not edit!
################################################################################

-include ../makefile.init

RM := rm -rf

# All of the sources participating in the build are defined here
-include sources.mk
-include vesc/subdir.mk
-include User/subdir.mk
-include Startup/subdir.mk
-include Peripheral/src/subdir.mk
-include Debug/subdir.mk
-include Core/subdir.mk
-include subdir.mk
-include objects.mk

ifneq ($(MAKECMDGOALS),clean)
ifneq ($(strip $(ASM_UPPER_DEPS)),)
-include $(ASM_UPPER_DEPS)
endif
ifneq ($(strip $(ASM_DEPS)),)
-include $(ASM_DEPS)
endif
ifneq ($(strip $(S_DEPS)),)
-include $(S_DEPS)
endif
ifneq ($(strip $(S_UPPER_DEPS)),)
-include $(S_UPPER_DEPS)
endif
ifneq ($(strip $(C_DEPS)),)
-include $(C_DEPS)
endif
endif

-include ../makefile.defs

# Add inputs and outputs from these tool invocations to the build variables 
SECONDARY_FLASH += \
bldc_riscv.hex \

SECONDARY_LIST += \
bldc_riscv.lst \

SECONDARY_SIZE += \
bldc_riscv.siz \


# All Target
all: bldc_riscv.elf secondary-outputs

# Tool invocations
bldc_riscv.elf: $(OBJS) $(USER_OBJS)
	@	@	riscv-none-embed-gcc -march=rv32imafc -mabi=ilp32f -msmall-data-limit=8 -mno-save-restore -Os -fmessage-length=0 -fsigned-char -ffunction-sections -fdata-sections -fsingle-precision-constant -Wunused -Wuninitialized  -g -T "C:\Users\Carl\Documents\WCH\bldc_freertos_rv\Ld\Link.ld" -nostartfiles -Xlinker --gc-sections -Wl,-Map,"bldc_riscv.map" --specs=nano.specs --specs=nosys.specs -o "bldc_riscv.elf" $(OBJS) $(USER_OBJS) $(LIBS)
	@	@
bldc_riscv.hex: bldc_riscv.elf
	@	riscv-none-embed-objcopy -O ihex "bldc_riscv.elf"  "bldc_riscv.hex"
	@	@
bldc_riscv.lst: bldc_riscv.elf
	@	riscv-none-embed-objdump --all-headers --demangle --disassemble "bldc_riscv.elf" > "bldc_riscv.lst"
	@	@
bldc_riscv.siz: bldc_riscv.elf
	@	riscv-none-embed-size --format=berkeley "bldc_riscv.elf"
	@	@
# Other Targets
clean:
	-$(RM) $(ASM_UPPER_DEPS)$(OBJS)$(SECONDARY_FLASH)$(SECONDARY_LIST)$(SECONDARY_SIZE)$(ASM_DEPS)$(S_DEPS)$(S_UPPER_DEPS)$(C_DEPS) bldc_riscv.elf
	-@
secondary-outputs: $(SECONDARY_FLASH) $(SECONDARY_LIST) $(SECONDARY_SIZE)

.PHONY: all clean dependents

-include ../makefile.targets
