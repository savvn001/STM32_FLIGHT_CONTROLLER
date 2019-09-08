################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Drivers/util/arduino_mpu9250_clk.c \
../Drivers/util/arduino_mpu9250_i2c.c \
../Drivers/util/arduino_mpu9250_log.c \
../Drivers/util/inv_mpu.c \
../Drivers/util/inv_mpu_dmp_motion_driver.c 

OBJS += \
./Drivers/util/arduino_mpu9250_clk.o \
./Drivers/util/arduino_mpu9250_i2c.o \
./Drivers/util/arduino_mpu9250_log.o \
./Drivers/util/inv_mpu.o \
./Drivers/util/inv_mpu_dmp_motion_driver.o 

C_DEPS += \
./Drivers/util/arduino_mpu9250_clk.d \
./Drivers/util/arduino_mpu9250_i2c.d \
./Drivers/util/arduino_mpu9250_log.d \
./Drivers/util/inv_mpu.d \
./Drivers/util/inv_mpu_dmp_motion_driver.d 


# Each subdirectory must supply rules for building sources it contributes
Drivers/util/arduino_mpu9250_clk.o: ../Drivers/util/arduino_mpu9250_clk.c
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 '-DMPL_LOG_NDEBUG=1' -DMPU9250 -DAK8963_SECONDARY -DCOMPASS_ENABLED -DUSE_DMP -DUSE_HAL_DRIVER -DEMPL -DSTM32F411xE -DDEBUG -c -I"/Users/nick_savva/STM32CubeIDE/workspace_1.0.1/STM32_FLIGHT_CONTROLLER/dsp_library" -I"/Users/nick_savva/STM32CubeIDE/workspace_1.0.1/STM32_FLIGHT_CONTROLLER/Drivers/util" -I../Inc -I../Drivers/CMSIS/Include -I../Drivers/STM32F4xx_HAL_Driver/Inc -I../Drivers/CMSIS/Device/ST/STM32F4xx/Include -I../Drivers/STM32F4xx_HAL_Driver/Inc/Legacy -I/Users/nick_savva/Downloads/CMSIS_5-develop/CMSIS/DSP/Include -Og -ffunction-sections -fdata-sections -Wall -fstack-usage -MMD -MP -MF"Drivers/util/arduino_mpu9250_clk.d" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"
Drivers/util/arduino_mpu9250_i2c.o: ../Drivers/util/arduino_mpu9250_i2c.c
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 '-DMPL_LOG_NDEBUG=1' -DMPU9250 -DAK8963_SECONDARY -DCOMPASS_ENABLED -DUSE_DMP -DUSE_HAL_DRIVER -DEMPL -DSTM32F411xE -DDEBUG -c -I"/Users/nick_savva/STM32CubeIDE/workspace_1.0.1/STM32_FLIGHT_CONTROLLER/dsp_library" -I"/Users/nick_savva/STM32CubeIDE/workspace_1.0.1/STM32_FLIGHT_CONTROLLER/Drivers/util" -I../Inc -I../Drivers/CMSIS/Include -I../Drivers/STM32F4xx_HAL_Driver/Inc -I../Drivers/CMSIS/Device/ST/STM32F4xx/Include -I../Drivers/STM32F4xx_HAL_Driver/Inc/Legacy -I/Users/nick_savva/Downloads/CMSIS_5-develop/CMSIS/DSP/Include -Og -ffunction-sections -fdata-sections -Wall -fstack-usage -MMD -MP -MF"Drivers/util/arduino_mpu9250_i2c.d" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"
Drivers/util/arduino_mpu9250_log.o: ../Drivers/util/arduino_mpu9250_log.c
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 '-DMPL_LOG_NDEBUG=1' -DMPU9250 -DAK8963_SECONDARY -DCOMPASS_ENABLED -DUSE_DMP -DUSE_HAL_DRIVER -DEMPL -DSTM32F411xE -DDEBUG -c -I"/Users/nick_savva/STM32CubeIDE/workspace_1.0.1/STM32_FLIGHT_CONTROLLER/dsp_library" -I"/Users/nick_savva/STM32CubeIDE/workspace_1.0.1/STM32_FLIGHT_CONTROLLER/Drivers/util" -I../Inc -I../Drivers/CMSIS/Include -I../Drivers/STM32F4xx_HAL_Driver/Inc -I../Drivers/CMSIS/Device/ST/STM32F4xx/Include -I../Drivers/STM32F4xx_HAL_Driver/Inc/Legacy -I/Users/nick_savva/Downloads/CMSIS_5-develop/CMSIS/DSP/Include -Og -ffunction-sections -fdata-sections -Wall -fstack-usage -MMD -MP -MF"Drivers/util/arduino_mpu9250_log.d" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"
Drivers/util/inv_mpu.o: ../Drivers/util/inv_mpu.c
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 '-DMPL_LOG_NDEBUG=1' -DMPU9250 -DAK8963_SECONDARY -DCOMPASS_ENABLED -DUSE_DMP -DUSE_HAL_DRIVER -DEMPL -DSTM32F411xE -DDEBUG -c -I"/Users/nick_savva/STM32CubeIDE/workspace_1.0.1/STM32_FLIGHT_CONTROLLER/dsp_library" -I"/Users/nick_savva/STM32CubeIDE/workspace_1.0.1/STM32_FLIGHT_CONTROLLER/Drivers/util" -I../Inc -I../Drivers/CMSIS/Include -I../Drivers/STM32F4xx_HAL_Driver/Inc -I../Drivers/CMSIS/Device/ST/STM32F4xx/Include -I../Drivers/STM32F4xx_HAL_Driver/Inc/Legacy -I/Users/nick_savva/Downloads/CMSIS_5-develop/CMSIS/DSP/Include -Og -ffunction-sections -fdata-sections -Wall -fstack-usage -MMD -MP -MF"Drivers/util/inv_mpu.d" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"
Drivers/util/inv_mpu_dmp_motion_driver.o: ../Drivers/util/inv_mpu_dmp_motion_driver.c
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 '-DMPL_LOG_NDEBUG=1' -DMPU9250 -DAK8963_SECONDARY -DCOMPASS_ENABLED -DUSE_DMP -DUSE_HAL_DRIVER -DEMPL -DSTM32F411xE -DDEBUG -c -I"/Users/nick_savva/STM32CubeIDE/workspace_1.0.1/STM32_FLIGHT_CONTROLLER/dsp_library" -I"/Users/nick_savva/STM32CubeIDE/workspace_1.0.1/STM32_FLIGHT_CONTROLLER/Drivers/util" -I../Inc -I../Drivers/CMSIS/Include -I../Drivers/STM32F4xx_HAL_Driver/Inc -I../Drivers/CMSIS/Device/ST/STM32F4xx/Include -I../Drivers/STM32F4xx_HAL_Driver/Inc/Legacy -I/Users/nick_savva/Downloads/CMSIS_5-develop/CMSIS/DSP/Include -Og -ffunction-sections -fdata-sections -Wall -fstack-usage -MMD -MP -MF"Drivers/util/inv_mpu_dmp_motion_driver.d" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"

