################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (9-2020-q2-update)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Core/Src/acc_driver.c \
../Core/Src/ess_helper.c \
../Core/Src/led_driver.c \
../Core/Src/main.c \
../Core/Src/pwm_driver.c \
../Core/Src/spi_driver.c \
../Core/Src/stm32f4xx_hal_msp.c \
../Core/Src/stm32f4xx_it.c \
../Core/Src/syscalls.c \
../Core/Src/sysmem.c \
../Core/Src/system_stm32f4xx.c \
../Core/Src/tmp_driver.c 

OBJS += \
./Core/Src/acc_driver.o \
./Core/Src/ess_helper.o \
./Core/Src/led_driver.o \
./Core/Src/main.o \
./Core/Src/pwm_driver.o \
./Core/Src/spi_driver.o \
./Core/Src/stm32f4xx_hal_msp.o \
./Core/Src/stm32f4xx_it.o \
./Core/Src/syscalls.o \
./Core/Src/sysmem.o \
./Core/Src/system_stm32f4xx.o \
./Core/Src/tmp_driver.o 

C_DEPS += \
./Core/Src/acc_driver.d \
./Core/Src/ess_helper.d \
./Core/Src/led_driver.d \
./Core/Src/main.d \
./Core/Src/pwm_driver.d \
./Core/Src/spi_driver.d \
./Core/Src/stm32f4xx_hal_msp.d \
./Core/Src/stm32f4xx_it.d \
./Core/Src/syscalls.d \
./Core/Src/sysmem.d \
./Core/Src/system_stm32f4xx.d \
./Core/Src/tmp_driver.d 


# Each subdirectory must supply rules for building sources it contributes
Core/Src/%.o: ../Core/Src/%.c Core/Src/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DUSE_HAL_DRIVER -DDEBUG -DSTM32F407xx -c -I../Core/Inc -I../Drivers/STM32F4xx_HAL_Driver/Inc -I../Drivers/STM32F4xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32F4xx/Include -I../Drivers/CMSIS/Include -O2 -ffunction-sections -fdata-sections -Wall -fstack-usage -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-Core-2f-Src

clean-Core-2f-Src:
	-$(RM) ./Core/Src/acc_driver.d ./Core/Src/acc_driver.o ./Core/Src/ess_helper.d ./Core/Src/ess_helper.o ./Core/Src/led_driver.d ./Core/Src/led_driver.o ./Core/Src/main.d ./Core/Src/main.o ./Core/Src/pwm_driver.d ./Core/Src/pwm_driver.o ./Core/Src/spi_driver.d ./Core/Src/spi_driver.o ./Core/Src/stm32f4xx_hal_msp.d ./Core/Src/stm32f4xx_hal_msp.o ./Core/Src/stm32f4xx_it.d ./Core/Src/stm32f4xx_it.o ./Core/Src/syscalls.d ./Core/Src/syscalls.o ./Core/Src/sysmem.d ./Core/Src/sysmem.o ./Core/Src/system_stm32f4xx.d ./Core/Src/system_stm32f4xx.o ./Core/Src/tmp_driver.d ./Core/Src/tmp_driver.o

.PHONY: clean-Core-2f-Src

