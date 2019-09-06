################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Drivers/eMD/data_builder.c \
../Drivers/eMD/eMPL_outputs.c \
../Drivers/eMD/hal_outputs.c \
../Drivers/eMD/inv_mpu.c \
../Drivers/eMD/inv_mpu_dmp_motion_driver.c \
../Drivers/eMD/message_layer.c \
../Drivers/eMD/ml_math_func.c \
../Drivers/eMD/mlmath.c \
../Drivers/eMD/mpl.c \
../Drivers/eMD/results_holder.c \
../Drivers/eMD/start_manager.c \
../Drivers/eMD/storage_manager.c 

OBJS += \
./Drivers/eMD/data_builder.o \
./Drivers/eMD/eMPL_outputs.o \
./Drivers/eMD/hal_outputs.o \
./Drivers/eMD/inv_mpu.o \
./Drivers/eMD/inv_mpu_dmp_motion_driver.o \
./Drivers/eMD/message_layer.o \
./Drivers/eMD/ml_math_func.o \
./Drivers/eMD/mlmath.o \
./Drivers/eMD/mpl.o \
./Drivers/eMD/results_holder.o \
./Drivers/eMD/start_manager.o \
./Drivers/eMD/storage_manager.o 

C_DEPS += \
./Drivers/eMD/data_builder.d \
./Drivers/eMD/eMPL_outputs.d \
./Drivers/eMD/hal_outputs.d \
./Drivers/eMD/inv_mpu.d \
./Drivers/eMD/inv_mpu_dmp_motion_driver.d \
./Drivers/eMD/message_layer.d \
./Drivers/eMD/ml_math_func.d \
./Drivers/eMD/mlmath.d \
./Drivers/eMD/mpl.d \
./Drivers/eMD/results_holder.d \
./Drivers/eMD/start_manager.d \
./Drivers/eMD/storage_manager.d 


# Each subdirectory must supply rules for building sources it contributes
Drivers/eMD/data_builder.o: ../Drivers/eMD/data_builder.c
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DUSE_DMP -DUSE_HAL_DRIVER -DEMPL -DSTM32F411xE -DEMPL_TARGET_STM32F4 '-DMPL_LOG_NDEBUG=1' -DMPU9250 -DDEBUG -c -I"/Users/nick_savva/STM32CubeIDE/workspace_1.0.1/STM32_FLIGHT_CONTROLLER/dsp_library" -I../Inc -I../Drivers/CMSIS/Include -I../Drivers/STM32F4xx_HAL_Driver/Inc -I../Drivers/CMSIS/Device/ST/STM32F4xx/Include -I../Drivers/STM32F4xx_HAL_Driver/Inc/Legacy -I/Users/nick_savva/Downloads/CMSIS_5-develop/CMSIS/DSP/Include -Og -ffunction-sections -fdata-sections -Wall -fstack-usage -MMD -MP -MF"Drivers/eMD/data_builder.d" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"
Drivers/eMD/eMPL_outputs.o: ../Drivers/eMD/eMPL_outputs.c
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DUSE_DMP -DUSE_HAL_DRIVER -DEMPL -DSTM32F411xE -DEMPL_TARGET_STM32F4 '-DMPL_LOG_NDEBUG=1' -DMPU9250 -DDEBUG -c -I"/Users/nick_savva/STM32CubeIDE/workspace_1.0.1/STM32_FLIGHT_CONTROLLER/dsp_library" -I../Inc -I../Drivers/CMSIS/Include -I../Drivers/STM32F4xx_HAL_Driver/Inc -I../Drivers/CMSIS/Device/ST/STM32F4xx/Include -I../Drivers/STM32F4xx_HAL_Driver/Inc/Legacy -I/Users/nick_savva/Downloads/CMSIS_5-develop/CMSIS/DSP/Include -Og -ffunction-sections -fdata-sections -Wall -fstack-usage -MMD -MP -MF"Drivers/eMD/eMPL_outputs.d" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"
Drivers/eMD/hal_outputs.o: ../Drivers/eMD/hal_outputs.c
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DUSE_DMP -DUSE_HAL_DRIVER -DEMPL -DSTM32F411xE -DEMPL_TARGET_STM32F4 '-DMPL_LOG_NDEBUG=1' -DMPU9250 -DDEBUG -c -I"/Users/nick_savva/STM32CubeIDE/workspace_1.0.1/STM32_FLIGHT_CONTROLLER/dsp_library" -I../Inc -I../Drivers/CMSIS/Include -I../Drivers/STM32F4xx_HAL_Driver/Inc -I../Drivers/CMSIS/Device/ST/STM32F4xx/Include -I../Drivers/STM32F4xx_HAL_Driver/Inc/Legacy -I/Users/nick_savva/Downloads/CMSIS_5-develop/CMSIS/DSP/Include -Og -ffunction-sections -fdata-sections -Wall -fstack-usage -MMD -MP -MF"Drivers/eMD/hal_outputs.d" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"
Drivers/eMD/inv_mpu.o: ../Drivers/eMD/inv_mpu.c
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DUSE_DMP -DUSE_HAL_DRIVER -DEMPL -DSTM32F411xE -DEMPL_TARGET_STM32F4 '-DMPL_LOG_NDEBUG=1' -DMPU9250 -DDEBUG -c -I"/Users/nick_savva/STM32CubeIDE/workspace_1.0.1/STM32_FLIGHT_CONTROLLER/dsp_library" -I../Inc -I../Drivers/CMSIS/Include -I../Drivers/STM32F4xx_HAL_Driver/Inc -I../Drivers/CMSIS/Device/ST/STM32F4xx/Include -I../Drivers/STM32F4xx_HAL_Driver/Inc/Legacy -I/Users/nick_savva/Downloads/CMSIS_5-develop/CMSIS/DSP/Include -Og -ffunction-sections -fdata-sections -Wall -fstack-usage -MMD -MP -MF"Drivers/eMD/inv_mpu.d" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"
Drivers/eMD/inv_mpu_dmp_motion_driver.o: ../Drivers/eMD/inv_mpu_dmp_motion_driver.c
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DUSE_DMP -DUSE_HAL_DRIVER -DEMPL -DSTM32F411xE -DEMPL_TARGET_STM32F4 '-DMPL_LOG_NDEBUG=1' -DMPU9250 -DDEBUG -c -I"/Users/nick_savva/STM32CubeIDE/workspace_1.0.1/STM32_FLIGHT_CONTROLLER/dsp_library" -I../Inc -I../Drivers/CMSIS/Include -I../Drivers/STM32F4xx_HAL_Driver/Inc -I../Drivers/CMSIS/Device/ST/STM32F4xx/Include -I../Drivers/STM32F4xx_HAL_Driver/Inc/Legacy -I/Users/nick_savva/Downloads/CMSIS_5-develop/CMSIS/DSP/Include -Og -ffunction-sections -fdata-sections -Wall -fstack-usage -MMD -MP -MF"Drivers/eMD/inv_mpu_dmp_motion_driver.d" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"
Drivers/eMD/message_layer.o: ../Drivers/eMD/message_layer.c
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DUSE_DMP -DUSE_HAL_DRIVER -DEMPL -DSTM32F411xE -DEMPL_TARGET_STM32F4 '-DMPL_LOG_NDEBUG=1' -DMPU9250 -DDEBUG -c -I"/Users/nick_savva/STM32CubeIDE/workspace_1.0.1/STM32_FLIGHT_CONTROLLER/dsp_library" -I../Inc -I../Drivers/CMSIS/Include -I../Drivers/STM32F4xx_HAL_Driver/Inc -I../Drivers/CMSIS/Device/ST/STM32F4xx/Include -I../Drivers/STM32F4xx_HAL_Driver/Inc/Legacy -I/Users/nick_savva/Downloads/CMSIS_5-develop/CMSIS/DSP/Include -Og -ffunction-sections -fdata-sections -Wall -fstack-usage -MMD -MP -MF"Drivers/eMD/message_layer.d" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"
Drivers/eMD/ml_math_func.o: ../Drivers/eMD/ml_math_func.c
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DUSE_DMP -DUSE_HAL_DRIVER -DEMPL -DSTM32F411xE -DEMPL_TARGET_STM32F4 '-DMPL_LOG_NDEBUG=1' -DMPU9250 -DDEBUG -c -I"/Users/nick_savva/STM32CubeIDE/workspace_1.0.1/STM32_FLIGHT_CONTROLLER/dsp_library" -I../Inc -I../Drivers/CMSIS/Include -I../Drivers/STM32F4xx_HAL_Driver/Inc -I../Drivers/CMSIS/Device/ST/STM32F4xx/Include -I../Drivers/STM32F4xx_HAL_Driver/Inc/Legacy -I/Users/nick_savva/Downloads/CMSIS_5-develop/CMSIS/DSP/Include -Og -ffunction-sections -fdata-sections -Wall -fstack-usage -MMD -MP -MF"Drivers/eMD/ml_math_func.d" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"
Drivers/eMD/mlmath.o: ../Drivers/eMD/mlmath.c
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DUSE_DMP -DUSE_HAL_DRIVER -DEMPL -DSTM32F411xE -DEMPL_TARGET_STM32F4 '-DMPL_LOG_NDEBUG=1' -DMPU9250 -DDEBUG -c -I"/Users/nick_savva/STM32CubeIDE/workspace_1.0.1/STM32_FLIGHT_CONTROLLER/dsp_library" -I../Inc -I../Drivers/CMSIS/Include -I../Drivers/STM32F4xx_HAL_Driver/Inc -I../Drivers/CMSIS/Device/ST/STM32F4xx/Include -I../Drivers/STM32F4xx_HAL_Driver/Inc/Legacy -I/Users/nick_savva/Downloads/CMSIS_5-develop/CMSIS/DSP/Include -Og -ffunction-sections -fdata-sections -Wall -fstack-usage -MMD -MP -MF"Drivers/eMD/mlmath.d" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"
Drivers/eMD/mpl.o: ../Drivers/eMD/mpl.c
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DUSE_DMP -DUSE_HAL_DRIVER -DEMPL -DSTM32F411xE -DEMPL_TARGET_STM32F4 '-DMPL_LOG_NDEBUG=1' -DMPU9250 -DDEBUG -c -I"/Users/nick_savva/STM32CubeIDE/workspace_1.0.1/STM32_FLIGHT_CONTROLLER/dsp_library" -I../Inc -I../Drivers/CMSIS/Include -I../Drivers/STM32F4xx_HAL_Driver/Inc -I../Drivers/CMSIS/Device/ST/STM32F4xx/Include -I../Drivers/STM32F4xx_HAL_Driver/Inc/Legacy -I/Users/nick_savva/Downloads/CMSIS_5-develop/CMSIS/DSP/Include -Og -ffunction-sections -fdata-sections -Wall -fstack-usage -MMD -MP -MF"Drivers/eMD/mpl.d" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"
Drivers/eMD/results_holder.o: ../Drivers/eMD/results_holder.c
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DUSE_DMP -DUSE_HAL_DRIVER -DEMPL -DSTM32F411xE -DEMPL_TARGET_STM32F4 '-DMPL_LOG_NDEBUG=1' -DMPU9250 -DDEBUG -c -I"/Users/nick_savva/STM32CubeIDE/workspace_1.0.1/STM32_FLIGHT_CONTROLLER/dsp_library" -I../Inc -I../Drivers/CMSIS/Include -I../Drivers/STM32F4xx_HAL_Driver/Inc -I../Drivers/CMSIS/Device/ST/STM32F4xx/Include -I../Drivers/STM32F4xx_HAL_Driver/Inc/Legacy -I/Users/nick_savva/Downloads/CMSIS_5-develop/CMSIS/DSP/Include -Og -ffunction-sections -fdata-sections -Wall -fstack-usage -MMD -MP -MF"Drivers/eMD/results_holder.d" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"
Drivers/eMD/start_manager.o: ../Drivers/eMD/start_manager.c
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DUSE_DMP -DUSE_HAL_DRIVER -DEMPL -DSTM32F411xE -DEMPL_TARGET_STM32F4 '-DMPL_LOG_NDEBUG=1' -DMPU9250 -DDEBUG -c -I"/Users/nick_savva/STM32CubeIDE/workspace_1.0.1/STM32_FLIGHT_CONTROLLER/dsp_library" -I../Inc -I../Drivers/CMSIS/Include -I../Drivers/STM32F4xx_HAL_Driver/Inc -I../Drivers/CMSIS/Device/ST/STM32F4xx/Include -I../Drivers/STM32F4xx_HAL_Driver/Inc/Legacy -I/Users/nick_savva/Downloads/CMSIS_5-develop/CMSIS/DSP/Include -Og -ffunction-sections -fdata-sections -Wall -fstack-usage -MMD -MP -MF"Drivers/eMD/start_manager.d" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"
Drivers/eMD/storage_manager.o: ../Drivers/eMD/storage_manager.c
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DUSE_DMP -DUSE_HAL_DRIVER -DEMPL -DSTM32F411xE -DEMPL_TARGET_STM32F4 '-DMPL_LOG_NDEBUG=1' -DMPU9250 -DDEBUG -c -I"/Users/nick_savva/STM32CubeIDE/workspace_1.0.1/STM32_FLIGHT_CONTROLLER/dsp_library" -I../Inc -I../Drivers/CMSIS/Include -I../Drivers/STM32F4xx_HAL_Driver/Inc -I../Drivers/CMSIS/Device/ST/STM32F4xx/Include -I../Drivers/STM32F4xx_HAL_Driver/Inc/Legacy -I/Users/nick_savva/Downloads/CMSIS_5-develop/CMSIS/DSP/Include -Og -ffunction-sections -fdata-sections -Wall -fstack-usage -MMD -MP -MF"Drivers/eMD/storage_manager.d" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"

