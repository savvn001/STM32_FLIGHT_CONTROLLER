################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Src/arm_add_f32.c \
../Src/arm_mult_f32.c \
../Src/arm_pid_init_f32.c \
../Src/arm_pid_reset_f32.c \
../Src/arm_sub_f32.c \
../Src/main.c \
../Src/stm32f4xx_hal_msp.c \
../Src/stm32f4xx_it.c \
../Src/syscalls.c \
../Src/sysmem.c \
../Src/system_stm32f4xx.c 

OBJS += \
./Src/arm_add_f32.o \
./Src/arm_mult_f32.o \
./Src/arm_pid_init_f32.o \
./Src/arm_pid_reset_f32.o \
./Src/arm_sub_f32.o \
./Src/main.o \
./Src/stm32f4xx_hal_msp.o \
./Src/stm32f4xx_it.o \
./Src/syscalls.o \
./Src/sysmem.o \
./Src/system_stm32f4xx.o 

C_DEPS += \
./Src/arm_add_f32.d \
./Src/arm_mult_f32.d \
./Src/arm_pid_init_f32.d \
./Src/arm_pid_reset_f32.d \
./Src/arm_sub_f32.d \
./Src/main.d \
./Src/stm32f4xx_hal_msp.d \
./Src/stm32f4xx_it.d \
./Src/syscalls.d \
./Src/sysmem.d \
./Src/system_stm32f4xx.d 


# Each subdirectory must supply rules for building sources it contributes
Src/arm_add_f32.o: ../Src/arm_add_f32.c
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 '-DMPL_LOG_NDEBUG=1' -DMPU9250 -DAK8963_SECONDARY -DCOMPASS_ENABLED -DUSE_DMP -DUSE_HAL_DRIVER -DEMPL -DSTM32F411xE -DDEBUG -c -I"/Users/nick_savva/STM32CubeIDE/workspace_1.0.1/STM32_FLIGHT_CONTROLLER/dsp_library" -I"/Users/nick_savva/STM32CubeIDE/workspace_1.0.1/STM32_FLIGHT_CONTROLLER/Drivers/util" -I../Inc -I../Drivers/CMSIS/Include -I../Drivers/STM32F4xx_HAL_Driver/Inc -I../Drivers/CMSIS/Device/ST/STM32F4xx/Include -I../Drivers/STM32F4xx_HAL_Driver/Inc/Legacy -I/Users/nick_savva/Downloads/CMSIS_5-develop/CMSIS/DSP/Include -Og -ffunction-sections -fdata-sections -Wall -fstack-usage -MMD -MP -MF"Src/arm_add_f32.d" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"
Src/arm_mult_f32.o: ../Src/arm_mult_f32.c
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 '-DMPL_LOG_NDEBUG=1' -DMPU9250 -DAK8963_SECONDARY -DCOMPASS_ENABLED -DUSE_DMP -DUSE_HAL_DRIVER -DEMPL -DSTM32F411xE -DDEBUG -c -I"/Users/nick_savva/STM32CubeIDE/workspace_1.0.1/STM32_FLIGHT_CONTROLLER/dsp_library" -I"/Users/nick_savva/STM32CubeIDE/workspace_1.0.1/STM32_FLIGHT_CONTROLLER/Drivers/util" -I../Inc -I../Drivers/CMSIS/Include -I../Drivers/STM32F4xx_HAL_Driver/Inc -I../Drivers/CMSIS/Device/ST/STM32F4xx/Include -I../Drivers/STM32F4xx_HAL_Driver/Inc/Legacy -I/Users/nick_savva/Downloads/CMSIS_5-develop/CMSIS/DSP/Include -Og -ffunction-sections -fdata-sections -Wall -fstack-usage -MMD -MP -MF"Src/arm_mult_f32.d" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"
Src/arm_pid_init_f32.o: ../Src/arm_pid_init_f32.c
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 '-DMPL_LOG_NDEBUG=1' -DMPU9250 -DAK8963_SECONDARY -DCOMPASS_ENABLED -DUSE_DMP -DUSE_HAL_DRIVER -DEMPL -DSTM32F411xE -DDEBUG -c -I"/Users/nick_savva/STM32CubeIDE/workspace_1.0.1/STM32_FLIGHT_CONTROLLER/dsp_library" -I"/Users/nick_savva/STM32CubeIDE/workspace_1.0.1/STM32_FLIGHT_CONTROLLER/Drivers/util" -I../Inc -I../Drivers/CMSIS/Include -I../Drivers/STM32F4xx_HAL_Driver/Inc -I../Drivers/CMSIS/Device/ST/STM32F4xx/Include -I../Drivers/STM32F4xx_HAL_Driver/Inc/Legacy -I/Users/nick_savva/Downloads/CMSIS_5-develop/CMSIS/DSP/Include -Og -ffunction-sections -fdata-sections -Wall -fstack-usage -MMD -MP -MF"Src/arm_pid_init_f32.d" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"
Src/arm_pid_reset_f32.o: ../Src/arm_pid_reset_f32.c
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 '-DMPL_LOG_NDEBUG=1' -DMPU9250 -DAK8963_SECONDARY -DCOMPASS_ENABLED -DUSE_DMP -DUSE_HAL_DRIVER -DEMPL -DSTM32F411xE -DDEBUG -c -I"/Users/nick_savva/STM32CubeIDE/workspace_1.0.1/STM32_FLIGHT_CONTROLLER/dsp_library" -I"/Users/nick_savva/STM32CubeIDE/workspace_1.0.1/STM32_FLIGHT_CONTROLLER/Drivers/util" -I../Inc -I../Drivers/CMSIS/Include -I../Drivers/STM32F4xx_HAL_Driver/Inc -I../Drivers/CMSIS/Device/ST/STM32F4xx/Include -I../Drivers/STM32F4xx_HAL_Driver/Inc/Legacy -I/Users/nick_savva/Downloads/CMSIS_5-develop/CMSIS/DSP/Include -Og -ffunction-sections -fdata-sections -Wall -fstack-usage -MMD -MP -MF"Src/arm_pid_reset_f32.d" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"
Src/arm_sub_f32.o: ../Src/arm_sub_f32.c
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 '-DMPL_LOG_NDEBUG=1' -DMPU9250 -DAK8963_SECONDARY -DCOMPASS_ENABLED -DUSE_DMP -DUSE_HAL_DRIVER -DEMPL -DSTM32F411xE -DDEBUG -c -I"/Users/nick_savva/STM32CubeIDE/workspace_1.0.1/STM32_FLIGHT_CONTROLLER/dsp_library" -I"/Users/nick_savva/STM32CubeIDE/workspace_1.0.1/STM32_FLIGHT_CONTROLLER/Drivers/util" -I../Inc -I../Drivers/CMSIS/Include -I../Drivers/STM32F4xx_HAL_Driver/Inc -I../Drivers/CMSIS/Device/ST/STM32F4xx/Include -I../Drivers/STM32F4xx_HAL_Driver/Inc/Legacy -I/Users/nick_savva/Downloads/CMSIS_5-develop/CMSIS/DSP/Include -Og -ffunction-sections -fdata-sections -Wall -fstack-usage -MMD -MP -MF"Src/arm_sub_f32.d" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"
Src/main.o: ../Src/main.c
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 '-DMPL_LOG_NDEBUG=1' -DMPU9250 -DAK8963_SECONDARY -DCOMPASS_ENABLED -DUSE_DMP -DUSE_HAL_DRIVER -DEMPL -DSTM32F411xE -DDEBUG -c -I"/Users/nick_savva/STM32CubeIDE/workspace_1.0.1/STM32_FLIGHT_CONTROLLER/dsp_library" -I"/Users/nick_savva/STM32CubeIDE/workspace_1.0.1/STM32_FLIGHT_CONTROLLER/Drivers/util" -I../Inc -I../Drivers/CMSIS/Include -I../Drivers/STM32F4xx_HAL_Driver/Inc -I../Drivers/CMSIS/Device/ST/STM32F4xx/Include -I../Drivers/STM32F4xx_HAL_Driver/Inc/Legacy -I/Users/nick_savva/Downloads/CMSIS_5-develop/CMSIS/DSP/Include -Og -ffunction-sections -fdata-sections -Wall -fstack-usage -MMD -MP -MF"Src/main.d" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"
Src/stm32f4xx_hal_msp.o: ../Src/stm32f4xx_hal_msp.c
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 '-DMPL_LOG_NDEBUG=1' -DMPU9250 -DAK8963_SECONDARY -DCOMPASS_ENABLED -DUSE_DMP -DUSE_HAL_DRIVER -DEMPL -DSTM32F411xE -DDEBUG -c -I"/Users/nick_savva/STM32CubeIDE/workspace_1.0.1/STM32_FLIGHT_CONTROLLER/dsp_library" -I"/Users/nick_savva/STM32CubeIDE/workspace_1.0.1/STM32_FLIGHT_CONTROLLER/Drivers/util" -I../Inc -I../Drivers/CMSIS/Include -I../Drivers/STM32F4xx_HAL_Driver/Inc -I../Drivers/CMSIS/Device/ST/STM32F4xx/Include -I../Drivers/STM32F4xx_HAL_Driver/Inc/Legacy -I/Users/nick_savva/Downloads/CMSIS_5-develop/CMSIS/DSP/Include -Og -ffunction-sections -fdata-sections -Wall -fstack-usage -MMD -MP -MF"Src/stm32f4xx_hal_msp.d" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"
Src/stm32f4xx_it.o: ../Src/stm32f4xx_it.c
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 '-DMPL_LOG_NDEBUG=1' -DMPU9250 -DAK8963_SECONDARY -DCOMPASS_ENABLED -DUSE_DMP -DUSE_HAL_DRIVER -DEMPL -DSTM32F411xE -DDEBUG -c -I"/Users/nick_savva/STM32CubeIDE/workspace_1.0.1/STM32_FLIGHT_CONTROLLER/dsp_library" -I"/Users/nick_savva/STM32CubeIDE/workspace_1.0.1/STM32_FLIGHT_CONTROLLER/Drivers/util" -I../Inc -I../Drivers/CMSIS/Include -I../Drivers/STM32F4xx_HAL_Driver/Inc -I../Drivers/CMSIS/Device/ST/STM32F4xx/Include -I../Drivers/STM32F4xx_HAL_Driver/Inc/Legacy -I/Users/nick_savva/Downloads/CMSIS_5-develop/CMSIS/DSP/Include -Og -ffunction-sections -fdata-sections -Wall -fstack-usage -MMD -MP -MF"Src/stm32f4xx_it.d" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"
Src/syscalls.o: ../Src/syscalls.c
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 '-DMPL_LOG_NDEBUG=1' -DMPU9250 -DAK8963_SECONDARY -DCOMPASS_ENABLED -DUSE_DMP -DUSE_HAL_DRIVER -DEMPL -DSTM32F411xE -DDEBUG -c -I"/Users/nick_savva/STM32CubeIDE/workspace_1.0.1/STM32_FLIGHT_CONTROLLER/dsp_library" -I"/Users/nick_savva/STM32CubeIDE/workspace_1.0.1/STM32_FLIGHT_CONTROLLER/Drivers/util" -I../Inc -I../Drivers/CMSIS/Include -I../Drivers/STM32F4xx_HAL_Driver/Inc -I../Drivers/CMSIS/Device/ST/STM32F4xx/Include -I../Drivers/STM32F4xx_HAL_Driver/Inc/Legacy -I/Users/nick_savva/Downloads/CMSIS_5-develop/CMSIS/DSP/Include -Og -ffunction-sections -fdata-sections -Wall -fstack-usage -MMD -MP -MF"Src/syscalls.d" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"
Src/sysmem.o: ../Src/sysmem.c
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 '-DMPL_LOG_NDEBUG=1' -DMPU9250 -DAK8963_SECONDARY -DCOMPASS_ENABLED -DUSE_DMP -DUSE_HAL_DRIVER -DEMPL -DSTM32F411xE -DDEBUG -c -I"/Users/nick_savva/STM32CubeIDE/workspace_1.0.1/STM32_FLIGHT_CONTROLLER/dsp_library" -I"/Users/nick_savva/STM32CubeIDE/workspace_1.0.1/STM32_FLIGHT_CONTROLLER/Drivers/util" -I../Inc -I../Drivers/CMSIS/Include -I../Drivers/STM32F4xx_HAL_Driver/Inc -I../Drivers/CMSIS/Device/ST/STM32F4xx/Include -I../Drivers/STM32F4xx_HAL_Driver/Inc/Legacy -I/Users/nick_savva/Downloads/CMSIS_5-develop/CMSIS/DSP/Include -Og -ffunction-sections -fdata-sections -Wall -fstack-usage -MMD -MP -MF"Src/sysmem.d" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"
Src/system_stm32f4xx.o: ../Src/system_stm32f4xx.c
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 '-DMPL_LOG_NDEBUG=1' -DMPU9250 -DAK8963_SECONDARY -DCOMPASS_ENABLED -DUSE_DMP -DUSE_HAL_DRIVER -DEMPL -DSTM32F411xE -DDEBUG -c -I"/Users/nick_savva/STM32CubeIDE/workspace_1.0.1/STM32_FLIGHT_CONTROLLER/dsp_library" -I"/Users/nick_savva/STM32CubeIDE/workspace_1.0.1/STM32_FLIGHT_CONTROLLER/Drivers/util" -I../Inc -I../Drivers/CMSIS/Include -I../Drivers/STM32F4xx_HAL_Driver/Inc -I../Drivers/CMSIS/Device/ST/STM32F4xx/Include -I../Drivers/STM32F4xx_HAL_Driver/Inc/Legacy -I/Users/nick_savva/Downloads/CMSIS_5-develop/CMSIS/DSP/Include -Og -ffunction-sections -fdata-sections -Wall -fstack-usage -MMD -MP -MF"Src/system_stm32f4xx.d" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"

