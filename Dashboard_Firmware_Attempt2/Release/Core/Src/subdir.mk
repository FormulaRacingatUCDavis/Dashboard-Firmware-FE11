################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (12.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Core/Src/can_manager.c \
../Core/Src/freertos.c \
../Core/Src/frucd_display.c \
../Core/Src/fsm.c \
../Core/Src/main.c \
../Core/Src/sd_card.c \
../Core/Src/sensors.c \
../Core/Src/serial.c \
../Core/Src/serial_print.c \
../Core/Src/stm32f7xx_hal_msp.c \
../Core/Src/stm32f7xx_hal_timebase_tim.c \
../Core/Src/stm32f7xx_it.c \
../Core/Src/syscalls.c \
../Core/Src/sysmem.c \
../Core/Src/system_stm32f7xx.c \
../Core/Src/telem.c \
../Core/Src/traction_control.c \
../Core/Src/ugui.c \
../Core/Src/ugui_SSD1963.c \
../Core/Src/wheel_speed.c \
../Core/Src/xsens.c 

OBJS += \
./Core/Src/can_manager.o \
./Core/Src/freertos.o \
./Core/Src/frucd_display.o \
./Core/Src/fsm.o \
./Core/Src/main.o \
./Core/Src/sd_card.o \
./Core/Src/sensors.o \
./Core/Src/serial.o \
./Core/Src/serial_print.o \
./Core/Src/stm32f7xx_hal_msp.o \
./Core/Src/stm32f7xx_hal_timebase_tim.o \
./Core/Src/stm32f7xx_it.o \
./Core/Src/syscalls.o \
./Core/Src/sysmem.o \
./Core/Src/system_stm32f7xx.o \
./Core/Src/telem.o \
./Core/Src/traction_control.o \
./Core/Src/ugui.o \
./Core/Src/ugui_SSD1963.o \
./Core/Src/wheel_speed.o \
./Core/Src/xsens.o 

C_DEPS += \
./Core/Src/can_manager.d \
./Core/Src/freertos.d \
./Core/Src/frucd_display.d \
./Core/Src/fsm.d \
./Core/Src/main.d \
./Core/Src/sd_card.d \
./Core/Src/sensors.d \
./Core/Src/serial.d \
./Core/Src/serial_print.d \
./Core/Src/stm32f7xx_hal_msp.d \
./Core/Src/stm32f7xx_hal_timebase_tim.d \
./Core/Src/stm32f7xx_it.d \
./Core/Src/syscalls.d \
./Core/Src/sysmem.d \
./Core/Src/system_stm32f7xx.d \
./Core/Src/telem.d \
./Core/Src/traction_control.d \
./Core/Src/ugui.d \
./Core/Src/ugui_SSD1963.d \
./Core/Src/wheel_speed.d \
./Core/Src/xsens.d 


# Each subdirectory must supply rules for building sources it contributes
Core/Src/%.o Core/Src/%.su Core/Src/%.cyclo: ../Core/Src/%.c Core/Src/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m7 -std=gnu11 -DUSE_HAL_DRIVER -DSTM32F746xx -c -I../Core/Inc -I../Drivers/STM32F7xx_HAL_Driver/Inc -I../Drivers/STM32F7xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32F7xx/Include -I../Drivers/CMSIS/Include -I../FATFS/Target -I../FATFS/App -I../Middlewares/Third_Party/FatFs/src -I../Middlewares/Third_Party/FreeRTOS/Source/include -I../Middlewares/Third_Party/FreeRTOS/Source/portable/GCC/ARM_CM7/r0p1 -I../Middlewares/Third_Party/FreeRTOS/Source/CMSIS_RTOS_V2 -Os -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv5-sp-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-Core-2f-Src

clean-Core-2f-Src:
	-$(RM) ./Core/Src/can_manager.cyclo ./Core/Src/can_manager.d ./Core/Src/can_manager.o ./Core/Src/can_manager.su ./Core/Src/freertos.cyclo ./Core/Src/freertos.d ./Core/Src/freertos.o ./Core/Src/freertos.su ./Core/Src/frucd_display.cyclo ./Core/Src/frucd_display.d ./Core/Src/frucd_display.o ./Core/Src/frucd_display.su ./Core/Src/fsm.cyclo ./Core/Src/fsm.d ./Core/Src/fsm.o ./Core/Src/fsm.su ./Core/Src/main.cyclo ./Core/Src/main.d ./Core/Src/main.o ./Core/Src/main.su ./Core/Src/sd_card.cyclo ./Core/Src/sd_card.d ./Core/Src/sd_card.o ./Core/Src/sd_card.su ./Core/Src/sensors.cyclo ./Core/Src/sensors.d ./Core/Src/sensors.o ./Core/Src/sensors.su ./Core/Src/serial.cyclo ./Core/Src/serial.d ./Core/Src/serial.o ./Core/Src/serial.su ./Core/Src/serial_print.cyclo ./Core/Src/serial_print.d ./Core/Src/serial_print.o ./Core/Src/serial_print.su ./Core/Src/stm32f7xx_hal_msp.cyclo ./Core/Src/stm32f7xx_hal_msp.d ./Core/Src/stm32f7xx_hal_msp.o ./Core/Src/stm32f7xx_hal_msp.su ./Core/Src/stm32f7xx_hal_timebase_tim.cyclo ./Core/Src/stm32f7xx_hal_timebase_tim.d ./Core/Src/stm32f7xx_hal_timebase_tim.o ./Core/Src/stm32f7xx_hal_timebase_tim.su ./Core/Src/stm32f7xx_it.cyclo ./Core/Src/stm32f7xx_it.d ./Core/Src/stm32f7xx_it.o ./Core/Src/stm32f7xx_it.su ./Core/Src/syscalls.cyclo ./Core/Src/syscalls.d ./Core/Src/syscalls.o ./Core/Src/syscalls.su ./Core/Src/sysmem.cyclo ./Core/Src/sysmem.d ./Core/Src/sysmem.o ./Core/Src/sysmem.su ./Core/Src/system_stm32f7xx.cyclo ./Core/Src/system_stm32f7xx.d ./Core/Src/system_stm32f7xx.o ./Core/Src/system_stm32f7xx.su ./Core/Src/telem.cyclo ./Core/Src/telem.d ./Core/Src/telem.o ./Core/Src/telem.su ./Core/Src/traction_control.cyclo ./Core/Src/traction_control.d ./Core/Src/traction_control.o ./Core/Src/traction_control.su ./Core/Src/ugui.cyclo ./Core/Src/ugui.d ./Core/Src/ugui.o ./Core/Src/ugui.su ./Core/Src/ugui_SSD1963.cyclo ./Core/Src/ugui_SSD1963.d ./Core/Src/ugui_SSD1963.o ./Core/Src/ugui_SSD1963.su ./Core/Src/wheel_speed.cyclo ./Core/Src/wheel_speed.d ./Core/Src/wheel_speed.o ./Core/Src/wheel_speed.su ./Core/Src/xsens.cyclo ./Core/Src/xsens.d ./Core/Src/xsens.o ./Core/Src/xsens.su

.PHONY: clean-Core-2f-Src

