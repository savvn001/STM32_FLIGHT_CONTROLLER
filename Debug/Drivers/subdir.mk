################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Drivers/GPS.c \
../Drivers/IMU.c \
../Drivers/MY_NRF24.c \
../Drivers/PID.c \
../Drivers/bno055.c \
../Drivers/dwt_delay.c 

OBJS += \
./Drivers/GPS.o \
./Drivers/IMU.o \
./Drivers/MY_NRF24.o \
./Drivers/PID.o \
./Drivers/bno055.o \
./Drivers/dwt_delay.o 

C_DEPS += \
./Drivers/GPS.d \
./Drivers/IMU.d \
./Drivers/MY_NRF24.d \
./Drivers/PID.d \
./Drivers/bno055.d \
./Drivers/dwt_delay.d 


# Each subdirectory must supply rules for building sources it contributes
Drivers/GPS.o: ../Drivers/GPS.c
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DCOMPASS_ENABLED -DUSE_DMP -DUSE_HAL_DRIVER -DEMPL -DI2C2 -DSTM32F411xE -DAK8963_SECONDARY '-DMPL_LOG_NDEBUG=1' -DMPU9250 -DDEBUG -c -I../Middlewares/Third_Party/FreeRTOS/Source/include -I"/Users/nick_savva/STM32CubeIDE/workspace_1.0.1/STM32_FLIGHT_CONTROLLER/dsp_library" -I../Middlewares/Third_Party/FreeRTOS/Source/portable/GCC/ARM_CM4F -I../Inc -I"/Users/nick_savva/STM32CubeIDE/workspace_1.0.1/STM32_FLIGHT_CONTROLLER/Drivers/STM32F4xx_HAL_Driver/Inc" -I../Drivers/CMSIS/Include -I../Drivers/STM32F4xx_HAL_Driver/Inc -I../Drivers/CMSIS/Device/ST/STM32F4xx/Include -I../Middlewares/Third_Party/FreeRTOS/Source/CMSIS_RTOS -I../Drivers/STM32F4xx_HAL_Driver/Inc/Legacy -I/Users/nick_savva/Downloads/CMSIS_5-develop/CMSIS/DSP/Include -Og -ffunction-sections -fdata-sections -Wall -fstack-usage -MMD -MP -MF"Drivers/GPS.d" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"
Drivers/IMU.o: ../Drivers/IMU.c
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DCOMPASS_ENABLED -DUSE_DMP -DUSE_HAL_DRIVER -DEMPL -DI2C2 -DSTM32F411xE -DAK8963_SECONDARY '-DMPL_LOG_NDEBUG=1' -DMPU9250 -DDEBUG -c -I../Middlewares/Third_Party/FreeRTOS/Source/include -I"/Users/nick_savva/STM32CubeIDE/workspace_1.0.1/STM32_FLIGHT_CONTROLLER/dsp_library" -I../Middlewares/Third_Party/FreeRTOS/Source/portable/GCC/ARM_CM4F -I../Inc -I"/Users/nick_savva/STM32CubeIDE/workspace_1.0.1/STM32_FLIGHT_CONTROLLER/Drivers/STM32F4xx_HAL_Driver/Inc" -I../Drivers/CMSIS/Include -I../Drivers/STM32F4xx_HAL_Driver/Inc -I../Drivers/CMSIS/Device/ST/STM32F4xx/Include -I../Middlewares/Third_Party/FreeRTOS/Source/CMSIS_RTOS -I../Drivers/STM32F4xx_HAL_Driver/Inc/Legacy -I/Users/nick_savva/Downloads/CMSIS_5-develop/CMSIS/DSP/Include -Og -ffunction-sections -fdata-sections -Wall -fstack-usage -MMD -MP -MF"Drivers/IMU.d" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"
Drivers/MY_NRF24.o: ../Drivers/MY_NRF24.c
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DCOMPASS_ENABLED -DUSE_DMP -DUSE_HAL_DRIVER -DEMPL -DI2C2 -DSTM32F411xE -DAK8963_SECONDARY '-DMPL_LOG_NDEBUG=1' -DMPU9250 -DDEBUG -c -I../Middlewares/Third_Party/FreeRTOS/Source/include -I"/Users/nick_savva/STM32CubeIDE/workspace_1.0.1/STM32_FLIGHT_CONTROLLER/dsp_library" -I../Middlewares/Third_Party/FreeRTOS/Source/portable/GCC/ARM_CM4F -I../Inc -I"/Users/nick_savva/STM32CubeIDE/workspace_1.0.1/STM32_FLIGHT_CONTROLLER/Drivers/STM32F4xx_HAL_Driver/Inc" -I../Drivers/CMSIS/Include -I../Drivers/STM32F4xx_HAL_Driver/Inc -I../Drivers/CMSIS/Device/ST/STM32F4xx/Include -I../Middlewares/Third_Party/FreeRTOS/Source/CMSIS_RTOS -I../Drivers/STM32F4xx_HAL_Driver/Inc/Legacy -I/Users/nick_savva/Downloads/CMSIS_5-develop/CMSIS/DSP/Include -Og -ffunction-sections -fdata-sections -Wall -fstack-usage -MMD -MP -MF"Drivers/MY_NRF24.d" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"
Drivers/PID.o: ../Drivers/PID.c
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DCOMPASS_ENABLED -DUSE_DMP -DUSE_HAL_DRIVER -DEMPL -DI2C2 -DSTM32F411xE -DAK8963_SECONDARY '-DMPL_LOG_NDEBUG=1' -DMPU9250 -DDEBUG -c -I../Middlewares/Third_Party/FreeRTOS/Source/include -I"/Users/nick_savva/STM32CubeIDE/workspace_1.0.1/STM32_FLIGHT_CONTROLLER/dsp_library" -I../Middlewares/Third_Party/FreeRTOS/Source/portable/GCC/ARM_CM4F -I../Inc -I"/Users/nick_savva/STM32CubeIDE/workspace_1.0.1/STM32_FLIGHT_CONTROLLER/Drivers/STM32F4xx_HAL_Driver/Inc" -I../Drivers/CMSIS/Include -I../Drivers/STM32F4xx_HAL_Driver/Inc -I../Drivers/CMSIS/Device/ST/STM32F4xx/Include -I../Middlewares/Third_Party/FreeRTOS/Source/CMSIS_RTOS -I../Drivers/STM32F4xx_HAL_Driver/Inc/Legacy -I/Users/nick_savva/Downloads/CMSIS_5-develop/CMSIS/DSP/Include -Og -ffunction-sections -fdata-sections -Wall -fstack-usage -MMD -MP -MF"Drivers/PID.d" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"
Drivers/bno055.o: ../Drivers/bno055.c
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DCOMPASS_ENABLED -DUSE_DMP -DUSE_HAL_DRIVER -DEMPL -DI2C2 -DSTM32F411xE -DAK8963_SECONDARY '-DMPL_LOG_NDEBUG=1' -DMPU9250 -DDEBUG -c -I../Middlewares/Third_Party/FreeRTOS/Source/include -I"/Users/nick_savva/STM32CubeIDE/workspace_1.0.1/STM32_FLIGHT_CONTROLLER/dsp_library" -I../Middlewares/Third_Party/FreeRTOS/Source/portable/GCC/ARM_CM4F -I../Inc -I"/Users/nick_savva/STM32CubeIDE/workspace_1.0.1/STM32_FLIGHT_CONTROLLER/Drivers/STM32F4xx_HAL_Driver/Inc" -I../Drivers/CMSIS/Include -I../Drivers/STM32F4xx_HAL_Driver/Inc -I../Drivers/CMSIS/Device/ST/STM32F4xx/Include -I../Middlewares/Third_Party/FreeRTOS/Source/CMSIS_RTOS -I../Drivers/STM32F4xx_HAL_Driver/Inc/Legacy -I/Users/nick_savva/Downloads/CMSIS_5-develop/CMSIS/DSP/Include -Og -ffunction-sections -fdata-sections -Wall -fstack-usage -MMD -MP -MF"Drivers/bno055.d" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"
Drivers/dwt_delay.o: ../Drivers/dwt_delay.c
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DCOMPASS_ENABLED -DUSE_DMP -DUSE_HAL_DRIVER -DEMPL -DI2C2 -DSTM32F411xE -DAK8963_SECONDARY '-DMPL_LOG_NDEBUG=1' -DMPU9250 -DDEBUG -c -I../Middlewares/Third_Party/FreeRTOS/Source/include -I"/Users/nick_savva/STM32CubeIDE/workspace_1.0.1/STM32_FLIGHT_CONTROLLER/dsp_library" -I../Middlewares/Third_Party/FreeRTOS/Source/portable/GCC/ARM_CM4F -I../Inc -I"/Users/nick_savva/STM32CubeIDE/workspace_1.0.1/STM32_FLIGHT_CONTROLLER/Drivers/STM32F4xx_HAL_Driver/Inc" -I../Drivers/CMSIS/Include -I../Drivers/STM32F4xx_HAL_Driver/Inc -I../Drivers/CMSIS/Device/ST/STM32F4xx/Include -I../Middlewares/Third_Party/FreeRTOS/Source/CMSIS_RTOS -I../Drivers/STM32F4xx_HAL_Driver/Inc/Legacy -I/Users/nick_savva/Downloads/CMSIS_5-develop/CMSIS/DSP/Include -Og -ffunction-sections -fdata-sections -Wall -fstack-usage -MMD -MP -MF"Drivers/dwt_delay.d" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"

