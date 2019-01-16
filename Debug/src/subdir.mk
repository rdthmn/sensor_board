################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../src/board.c \
../src/main.c \
../src/stm32f4xx_it.c \
../src/syscalls.c \
../src/system_stm32f4xx.c 

OBJS += \
./src/board.o \
./src/main.o \
./src/stm32f4xx_it.o \
./src/syscalls.o \
./src/system_stm32f4xx.o 

C_DEPS += \
./src/board.d \
./src/main.d \
./src/stm32f4xx_it.d \
./src/syscalls.d \
./src/system_stm32f4xx.d 


# Each subdirectory must supply rules for building sources it contributes
src/%.o: ../src/%.c
	@echo 'Building file: $<'
	@echo 'Invoking: MCU GCC Compiler'
	@echo $(PWD)
	arm-none-eabi-gcc -mcpu=cortex-m4 -mthumb -mfloat-abi=hard -mfpu=fpv4-sp-d16 -DSTM32 -DARM_MATH_CM4 -DSTM32F4 -DSTM32F446RETx -DNUCLEO_F446RE -DDEBUG -DSTM32F446xx -DUSE_HAL_DRIVER -I"/Users/rad/Dropbox/Research/sensor_board/CMSIS/core" -I"/Users/rad/Dropbox/Research/sensor_board/HAL_Driver/Inc/Legacy" -I"/Users/rad/Dropbox/Research/sensor_board/Utilities/STM32F4xx-Nucleo" -I"/Users/rad/Dropbox/Research/sensor_board/inc" -I"/Users/rad/Dropbox/Research/sensor_board/CMSIS/device" -I"/Users/rad/Dropbox/Research/sensor_board/HAL_Driver/Inc" -O0 -g3 -Wall -fmessage-length=0 -ffunction-sections -c -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '

src/main.o: ../src/main.c
	@echo 'Building file: $<'
	@echo 'Invoking: MCU GCC Compiler'
	@echo $(PWD)
	arm-none-eabi-gcc -mcpu=cortex-m4 -mthumb -mfloat-abi=hard -mfpu=fpv4-sp-d16 -DSTM32 -DARM_MATH_CM4 -DSTM32F4 -DSTM32F446RETx -DNUCLEO_F446RE -DDEBUG -DSTM32F446xx -DUSE_HAL_DRIVER -I"/Users/rad/Dropbox/Research/sensor_board/CMSIS/core" -I"/Users/rad/Dropbox/Research/sensor_board/HAL_Driver/Inc/Legacy" -I"/Users/rad/Dropbox/Research/sensor_board/Utilities/STM32F4xx-Nucleo" -I"/Users/rad/Dropbox/Research/sensor_board/inc" -I"/Users/rad/Dropbox/Research/sensor_board/CMSIS/device" -I"/Users/rad/Dropbox/Research/sensor_board/HAL_Driver/Inc" -O0 -g3 -Wall -fmessage-length=0 -ffunction-sections -c -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '


