################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables
C_SRCS += \
../src/r_fw_up_rx/r_fw_up_rx/r_fw_up_buf.c \
../src/r_fw_up_rx/r_fw_up_rx/r_fw_up_rx.c 

COMPILER_OBJS += \
src/r_fw_up_rx/r_fw_up_rx/r_fw_up_buf.obj \
src/r_fw_up_rx/r_fw_up_rx/r_fw_up_rx.obj 

C_DEPS += \
src/r_fw_up_rx/r_fw_up_rx/r_fw_up_buf.d \
src/r_fw_up_rx/r_fw_up_rx/r_fw_up_rx.d 

# Each subdirectory must supply rules for building sources it contributes
src/r_fw_up_rx/r_fw_up_rx/%.obj: ../src/r_fw_up_rx/r_fw_up_rx/%.c 
	@echo 'Scanning and building file: $<'
	@echo 'Invoking: Scanner and Compiler'
	@echo src\r_fw_up_rx\r_fw_up_rx\cDepSubCommand.tmp=
	@sed -e "s/^/ /" "src\r_fw_up_rx\r_fw_up_rx\cDepSubCommand.tmp"
	ccrx -subcommand="src\r_fw_up_rx\r_fw_up_rx\cDepSubCommand.tmp" -output=dep="$(@:%.obj=%.d)"  -MT="$(@:%.d=%.obj)"  -MT="$(@:%.obj=%.d)" "$<"
	@echo src\r_fw_up_rx\r_fw_up_rx\cSubCommand.tmp=
	@sed -e "s/^/ /" "src\r_fw_up_rx\r_fw_up_rx\cSubCommand.tmp"
	ccrx -subcommand="src\r_fw_up_rx\r_fw_up_rx\cSubCommand.tmp" "$<"
	@echo 'Finished Scanning and building: $<'
	@echo ''

