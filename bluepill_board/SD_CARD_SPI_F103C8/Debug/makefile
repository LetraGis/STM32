################################################################################
# Automatically-generated file. Do not edit!
################################################################################

-include ../makefile.init

RM := rm -rf

# All of the sources participating in the build are defined here
-include sources.mk
-include Middlewares/Third_Party/FatFs/src/option/subdir.mk
-include Middlewares/Third_Party/FatFs/src/subdir.mk
-include FATFS/Target/subdir.mk
-include FATFS/App/subdir.mk
-include Drivers/STM32F1xx_HAL_Driver/Src/subdir.mk
-include Core/Startup/subdir.mk
-include Core/Src/subdir.mk
-include subdir.mk
-include objects.mk

ifneq ($(MAKECMDGOALS),clean)
ifneq ($(strip $(C_DEPS)),)
-include $(C_DEPS)
endif
endif

-include ../makefile.defs

# Add inputs and outputs from these tool invocations to the build variables 
EXECUTABLES += \
SD_CARD_SPI_F103C8.elf \

SIZE_OUTPUT += \
default.size.stdout \

OBJDUMP_LIST += \
SD_CARD_SPI_F103C8.list \

OBJCOPY_BIN += \
SD_CARD_SPI_F103C8.bin \


# All Target
all: SD_CARD_SPI_F103C8.elf secondary-outputs

# Tool invocations
SD_CARD_SPI_F103C8.elf: $(OBJS) $(USER_OBJS) C:\Users\diana\Documents\Embedded\SD_CARD_SPI_F103C8\STM32F103C8TX_FLASH.ld
	arm-none-eabi-gcc -o "SD_CARD_SPI_F103C8.elf" @"objects.list" $(USER_OBJS) $(LIBS) -mcpu=cortex-m3 -T"C:\Users\diana\Documents\Embedded\SD_CARD_SPI_F103C8\STM32F103C8TX_FLASH.ld" --specs=nosys.specs -Wl,-Map="SD_CARD_SPI_F103C8.map" -Wl,--gc-sections -static --specs=nano.specs -mfloat-abi=soft -mthumb -Wl,--start-group -lc -lm -Wl,--end-group
	@echo 'Finished building target: $@'
	@echo ' '

default.size.stdout: $(EXECUTABLES)
	arm-none-eabi-size  $(EXECUTABLES)
	@echo 'Finished building: $@'
	@echo ' '

SD_CARD_SPI_F103C8.list: $(EXECUTABLES)
	arm-none-eabi-objdump -h -S $(EXECUTABLES) > "SD_CARD_SPI_F103C8.list"
	@echo 'Finished building: $@'
	@echo ' '

SD_CARD_SPI_F103C8.bin: $(EXECUTABLES)
	arm-none-eabi-objcopy  -O binary $(EXECUTABLES) "SD_CARD_SPI_F103C8.bin"
	@echo 'Finished building: $@'
	@echo ' '

# Other Targets
clean:
	-$(RM) *
	-@echo ' '

secondary-outputs: $(SIZE_OUTPUT) $(OBJDUMP_LIST) $(OBJCOPY_BIN)

.PHONY: all clean dependents
.SECONDARY:

-include ../makefile.targets
