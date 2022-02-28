################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (9-2020-q2-update)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Core/Src/Library/AMT21.c \
../Core/Src/Library/ARMsProtocol.c \
../Core/Src/Library/KalmanFilter.c \
../Core/Src/Library/Kinematic.c \
../Core/Src/Library/Motor.c \
../Core/Src/Library/PID.c \
../Core/Src/Library/Trajectory.c 

OBJS += \
./Core/Src/Library/AMT21.o \
./Core/Src/Library/ARMsProtocol.o \
./Core/Src/Library/KalmanFilter.o \
./Core/Src/Library/Kinematic.o \
./Core/Src/Library/Motor.o \
./Core/Src/Library/PID.o \
./Core/Src/Library/Trajectory.o 

C_DEPS += \
./Core/Src/Library/AMT21.d \
./Core/Src/Library/ARMsProtocol.d \
./Core/Src/Library/KalmanFilter.d \
./Core/Src/Library/Kinematic.d \
./Core/Src/Library/Motor.d \
./Core/Src/Library/PID.d \
./Core/Src/Library/Trajectory.d 


# Each subdirectory must supply rules for building sources it contributes
Core/Src/Library/%.o: ../Core/Src/Library/%.c Core/Src/Library/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m7 -std=gnu11 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32H733xx -c -I../Core/Inc -I../Drivers/STM32H7xx_HAL_Driver/Inc -I../Drivers/STM32H7xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32H7xx/Include -I../Drivers/CMSIS/Include -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv5-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-Core-2f-Src-2f-Library

clean-Core-2f-Src-2f-Library:
	-$(RM) ./Core/Src/Library/AMT21.d ./Core/Src/Library/AMT21.o ./Core/Src/Library/ARMsProtocol.d ./Core/Src/Library/ARMsProtocol.o ./Core/Src/Library/KalmanFilter.d ./Core/Src/Library/KalmanFilter.o ./Core/Src/Library/Kinematic.d ./Core/Src/Library/Kinematic.o ./Core/Src/Library/Motor.d ./Core/Src/Library/Motor.o ./Core/Src/Library/PID.d ./Core/Src/Library/PID.o ./Core/Src/Library/Trajectory.d ./Core/Src/Library/Trajectory.o

.PHONY: clean-Core-2f-Src-2f-Library

