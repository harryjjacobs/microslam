################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (10.3-2021.10)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
CPP_SRCS += \
../Core/Src/vehicle/adc.cpp \
../Core/Src/vehicle/pwm.cpp \
../Core/Src/vehicle/ultrasound.cpp \
../Core/Src/vehicle/vehicle.cpp 

OBJS += \
./Core/Src/vehicle/adc.o \
./Core/Src/vehicle/pwm.o \
./Core/Src/vehicle/ultrasound.o \
./Core/Src/vehicle/vehicle.o 

CPP_DEPS += \
./Core/Src/vehicle/adc.d \
./Core/Src/vehicle/pwm.d \
./Core/Src/vehicle/ultrasound.d \
./Core/Src/vehicle/vehicle.d 


# Each subdirectory must supply rules for building sources it contributes
Core/Src/vehicle/%.o Core/Src/vehicle/%.su: ../Core/Src/vehicle/%.cpp Core/Src/vehicle/subdir.mk
	arm-none-eabi-g++ "$<" -mcpu=cortex-m4 -std=gnu++14 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32G431xx -c -I../Core/Inc -I../Drivers/STM32G4xx_HAL_Driver/Inc -I../Drivers/STM32G4xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32G4xx/Include -I../Drivers/CMSIS/Include -I"/home/harryjjacobs/projects/microslam/microslam/include" -O0 -ffunction-sections -fdata-sections -fno-exceptions -fno-rtti -fno-use-cxa-atexit -Wall -fstack-usage -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-Core-2f-Src-2f-vehicle

clean-Core-2f-Src-2f-vehicle:
	-$(RM) ./Core/Src/vehicle/adc.d ./Core/Src/vehicle/adc.o ./Core/Src/vehicle/adc.su ./Core/Src/vehicle/pwm.d ./Core/Src/vehicle/pwm.o ./Core/Src/vehicle/pwm.su ./Core/Src/vehicle/ultrasound.d ./Core/Src/vehicle/ultrasound.o ./Core/Src/vehicle/ultrasound.su ./Core/Src/vehicle/vehicle.d ./Core/Src/vehicle/vehicle.o ./Core/Src/vehicle/vehicle.su

.PHONY: clean-Core-2f-Src-2f-vehicle

