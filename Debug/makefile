################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (11.3.rel1)
################################################################################

-include ../makefile.init

RM := rm -rf

# All of the sources participating in the build are defined here
-include sources.mk
-include Middlewares/Third_Party/FreeRTOS/Source/portable/MemMang/subdir.mk
-include Middlewares/Third_Party/FreeRTOS/Source/portable/GCC/ARM_CM4F/subdir.mk
-include Middlewares/Third_Party/FreeRTOS/Source/CMSIS_RTOS/subdir.mk
-include Middlewares/Third_Party/FreeRTOS/Source/subdir.mk
-include Drivers/STM32G4xx_HAL_Driver/Src/subdir.mk
-include Core/ThreadSafe/subdir.mk
-include Core/Startup/subdir.mk
-include Core/Src/subdir.mk
-include objects.mk

ifneq ($(MAKECMDGOALS),clean)
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

OPTIONAL_TOOL_DEPS := \
$(wildcard ../makefile.defs) \
$(wildcard ../makefile.init) \
$(wildcard ../makefile.targets) \


BUILD_ARTIFACT_NAME := HEPA_UV_DVT_Proto
BUILD_ARTIFACT_EXTENSION := elf
BUILD_ARTIFACT_PREFIX :=
BUILD_ARTIFACT := $(BUILD_ARTIFACT_PREFIX)$(BUILD_ARTIFACT_NAME)$(if $(BUILD_ARTIFACT_EXTENSION),.$(BUILD_ARTIFACT_EXTENSION),)

# Add inputs and outputs from these tool invocations to the build variables 
EXECUTABLES += \
HEPA_UV_DVT_Proto.elf \

MAP_FILES += \
HEPA_UV_DVT_Proto.map \

SIZE_OUTPUT += \
default.size.stdout \

OBJDUMP_LIST += \
HEPA_UV_DVT_Proto.list \

OBJCOPY_HEX += \
HEPA_UV_DVT_Proto.hex \

OBJCOPY_BIN += \
HEPA_UV_DVT_Proto.bin \


# All Target
all: main-build

# Main-build Target
main-build: HEPA_UV_DVT_Proto.elf secondary-outputs

# Tool invocations
HEPA_UV_DVT_Proto.elf HEPA_UV_DVT_Proto.map: $(OBJS) $(USER_OBJS) G:\Shared\ drives\Opentrons\Hardware\1\ -\ Hardware\ Engineering\1\ -\ NPI\ Programs\12\ -\ OT3\3\ -\ Electrical\15\ -\ HEPA_UV\Test\ Code\DVT\ PWM\ HEPA\STM32G491RETX_FLASH.ld makefile objects.list $(OPTIONAL_TOOL_DEPS)
	arm-none-eabi-gcc -o "HEPA_UV_DVT_Proto.elf" @"objects.list" $(USER_OBJS) $(LIBS) -mcpu=cortex-m4 -T"G:\Shared drives\Opentrons\Hardware\1 - Hardware Engineering\1 - NPI Programs\12 - OT3\3 - Electrical\15 - HEPA_UV\Test Code\DVT PWM HEPA\STM32G491RETX_FLASH.ld" --specs=nosys.specs -Wl,-Map="HEPA_UV_DVT_Proto.map" -Wl,--gc-sections -static --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -Wl,--start-group -lc -lm -Wl,--end-group
	@echo 'Finished building target: $@'
	@echo ' '

default.size.stdout: $(EXECUTABLES) makefile objects.list $(OPTIONAL_TOOL_DEPS)
	arm-none-eabi-size  $(EXECUTABLES)
	@echo 'Finished building: $@'
	@echo ' '

HEPA_UV_DVT_Proto.list: $(EXECUTABLES) makefile objects.list $(OPTIONAL_TOOL_DEPS)
	arm-none-eabi-objdump -h -S $(EXECUTABLES) > "HEPA_UV_DVT_Proto.list"
	@echo 'Finished building: $@'
	@echo ' '

HEPA_UV_DVT_Proto.hex: $(EXECUTABLES) makefile objects.list $(OPTIONAL_TOOL_DEPS)
	arm-none-eabi-objcopy  -O ihex $(EXECUTABLES) "HEPA_UV_DVT_Proto.hex"
	@echo 'Finished building: $@'
	@echo ' '

HEPA_UV_DVT_Proto.bin: $(EXECUTABLES) makefile objects.list $(OPTIONAL_TOOL_DEPS)
	arm-none-eabi-objcopy  -O binary $(EXECUTABLES) "HEPA_UV_DVT_Proto.bin"
	@echo 'Finished building: $@'
	@echo ' '

# Other Targets
clean:
	-$(RM) HEPA_UV_DVT_Proto.bin HEPA_UV_DVT_Proto.elf HEPA_UV_DVT_Proto.hex HEPA_UV_DVT_Proto.list HEPA_UV_DVT_Proto.map default.size.stdout
	-@echo ' '

secondary-outputs: $(SIZE_OUTPUT) $(OBJDUMP_LIST) $(OBJCOPY_HEX) $(OBJCOPY_BIN)

fail-specified-linker-script-missing:
	@echo 'Error: Cannot find the specified linker script. Check the linker settings in the build configuration.'
	@exit 2

warn-no-linker-script-specified:
	@echo 'Warning: No linker script specified. Check the linker settings in the build configuration.'

.PHONY: all clean dependents main-build fail-specified-linker-script-missing warn-no-linker-script-specified

-include ../makefile.targets
