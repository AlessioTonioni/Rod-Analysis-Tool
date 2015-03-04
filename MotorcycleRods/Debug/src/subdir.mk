################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
CPP_SRCS += \
../src/Hole.cpp \
../src/Main.cpp \
../src/Rod.cpp \
../src/function.cpp 

OBJS += \
./src/Hole.o \
./src/Main.o \
./src/Rod.o \
./src/function.o 

CPP_DEPS += \
./src/Hole.d \
./src/Main.d \
./src/Rod.d \
./src/function.d 


# Each subdirectory must supply rules for building sources it contributes
src/%.o: ../src/%.cpp
	@echo 'Building file: $<'
	@echo 'Invoking: GCC C++ Compiler'
	g++ -I/usr/local/include/opencv -I"/home/alessio/cppWorkspace/MotorcycleRods/src/inc" -O0 -g3 -Wall -c -fmessage-length=0 -std=c++11 -MMD -MP -MF"$(@:%.o=%.d)" -MT"$(@:%.o=%.d)" -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '


