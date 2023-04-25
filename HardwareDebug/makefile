################################################################################
# Automatically-generated file. Do not edit!
################################################################################

-include makefile.init

RM := rm -rf *.sni *.dbg *.lst *.lis *.lpp *.map *.x *.lbp *.bls *.libelf *.rel *.lbk *.abs *.mi *.jmp *.fsy *.ud *.udm 1st/*.abs 1st/*.jmp 1st/*.fsy

# All of the sources participating in the build are defined here
-include sources.mk
-include src/subdir.mk
-include src/smc_gen/r_pincfg/subdir.mk
-include src/smc_gen/Config_MTU0/subdir.mk
-include src/smc_gen/Config_PORT/subdir.mk
-include src/smc_gen/r_bsp/mcu/rx130/subdir.mk
-include src/smc_gen/Config_S12AD0/subdir.mk
-include src/smc_gen/Config_CMT1/subdir.mk
-include src/smc_gen/Config_ICU/subdir.mk
-include src/smc_gen/Config_SCI12/subdir.mk
-include src/smc_gen/Config_SCI1/subdir.mk
-include src/smc_gen/Config_SCI5/subdir.mk
-include src/smc_gen/r_flash_rx/src/flash_type_4/subdir.mk
-include src/r_fw_up_rx/r_fw_up_rx/subdir.mk
-include src/smc_gen/Config_MTU3/subdir.mk
-include src/smc_gen/r_bsp/mcu/all/subdir.mk
-include src/smc_gen/general/subdir.mk
-include src/smc_gen/r_bsp/board/generic_rx130/subdir.mk
-include src/smc_gen/Config_CMT0/subdir.mk
-include src/smc_gen/r_flash_rx/src/subdir.mk
-include src/smc_gen/r_flash_rx/src/flash_type_1/subdir.mk
-include src/smc_gen/r_flash_rx/src/flash_type_3/subdir.mk

ifneq ($(MAKECMDGOALS),clean)
ifneq ($(strip $(C_DEPS)),)
-include $(C_DEPS)
endif 
ifneq ($(strip $(C_UPPER_DEPS)),)
-include $(C_UPPER_DEPS)
endif 
ifneq ($(strip $(CPP_DEPS)),)
-include $(CPP_DEPS)
endif 
ifneq ($(strip $(CXX_DEPS)),)
-include $(CXX_DEPS)
endif 
ifneq ($(strip $(CC_DEPS)),)
-include $(CC_DEPS)
endif 
ifneq ($(strip $(C++_DEPS)),)
-include $(C++_DEPS)
endif 
ifneq ($(strip $(S_UPPER_DEPS)),)
-include $(S_UPPER_DEPS)
endif 
ifneq ($(strip $(ASM_DEPS)),)
-include $(ASM_DEPS)
endif 
ifneq ($(strip $(SX_DEPS)),)
-include $(SX_DEPS)
endif 
ifneq ($(strip $(JMP_DEPS)),)
-include $(JMP_DEPS)
endif 
ifneq ($(strip $(SRC_DEPS)),)
-include $(SRC_DEPS)
endif 
ifneq ($(strip $(PP_DEPS)),)
-include $(PP_DEPS)
endif 
ifneq ($(strip $(P_DEPS)),)
-include $(P_DEPS)
endif 
ifneq ($(strip $(FSY_DEPS)),)
-include $(FSY_DEPS)
endif 
endif

ifneq ($(strip $(SRC_UPPER_SRCS)),)
ALL_ASMS += $(SRC_UPPER_SRCS) 
endif
ifneq ($(strip $(SRC_SRCS)),)
ALL_ASMS += $(SRC_SRCS) 
endif
ifneq ($(strip $(S_UPPER_SRCS)),)
ALL_ASMS += $(S_UPPER_SRCS) 
endif
ifneq ($(strip $(S_SRCS)),)
ALL_ASMS += $(S_SRCS) 
endif


-include ../makefile.defs

# Add inputs and outputs from these tool invocations to the build variables
LINKER_OUTPUT_OUTPUTS += \
MAC_Emko.abs

LIBRARY_GENERATOR_OUTPUTTYPE_OUTPUTS += \
MAC_Emko.lib

CONVERTOR_OUTPUT_OUTPUTS += \
MAC_Emko.mot

OBJS := \
$(COMPILER_OBJS) \
$(INDIVIDUAL_COMPILER_OBJS) \
$(ASSEMBLER_OBJS) \
$(CUSTOM_OBJS)

# All Target
all: $(CONVERTOR_OUTPUT_OUTPUTS) MAC_Emko.x MAC_Emko.udm

	@echo 'Build complete.'

%.mot: $(LINKER_OUTPUT_OUTPUTS) Converter.sub
	@echo 'Invoking: Converter'
	@echo 'Building target:'
	@echo ConverterMAC_Emko.tmp=
	@sed -e "s/^/    /" "ConverterMAC_Emko.tmp"
	rlink -subcommand="ConverterMAC_Emko.tmp"
	@echo 'Finished building target:'
	@echo ''


MAC_Emko.abs: $(OBJS)  $(ALL_ASMS) $(LIBRARY_GENERATOR_OUTPUTTYPE_OUTPUTS) 
	@echo 'Invoking: Linker'
	@echo LinkerMAC_Emko.tmp=
	@sed -e "s/^/    /" "LinkerMAC_Emko.tmp"
	rlink -subcommand="LinkerMAC_Emko.tmp"
	@echo 'Finished building:'
	@echo ''


MAC_Emko.lib: 
	@echo 'Invoking: Library Generator'
	@echo LibraryGeneratorMAC_Emko.tmp=
	@sed -e "s/^/    /" "LibraryGeneratorMAC_Emko.tmp"
	lbgrx -subcommand="LibraryGeneratorMAC_Emko.tmp"
	@echo 'Finished building:'
	@echo ''

MAC_Emko.x: MAC_Emko.abs
		-C:\Users\Admin\.eclipse\com.renesas.platform_1435879475\Utilities\\ccrx\renesas_cc_converter MAC_Emko.abs MAC_Emko.x

MAC_Emko.udm: $(LINKER_OUTPUT_OUTPUTS) $(CONVERTOR_OUTPUT_OUTPUTS)
	udcollector -subcommand=udSubcommand.tmp -output=MAC_Emko.udm

# Other Targets
clean:
	-$(RM) $(LINKER_OUTPUT_OUTPUTS) $(LIBRARY_GENERATOR_OUTPUTTYPE_OUTPUTS) $(CONVERTOR_OUTPUT_OUTPUTS)
ifneq ($(strip $(OBJS)),)
	-$(file > clean_OBJS.tmp, $(OBJS))
	-@xargs -s 4000 -t -a clean_OBJS.tmp rm -rf
	-@rm -rf clean_OBJS.tmp
endif
ifneq ($(strip $(C_DEPS)),)
	-$(file > clean_C_DEPS.tmp, $(C_DEPS))
	-@xargs -s 4000 -t -a clean_C_DEPS.tmp rm -rf
	-@rm -rf clean_C_DEPS.tmp
endif
ifneq ($(strip $(CPP_DEPS)),)
	-$(file > clean_CPP_DEPS.tmp, $(CPP_DEPS))
	-@xargs -s 4000 -t -a clean_CPP_DEPS.tmp rm -rf
	-@rm -rf clean_CPP_DEPS.tmp
endif
ifneq ($(strip $(ASM_DEPS)),)
	-$(file > clean_ASM_DEPS.tmp, $(ASM_DEPS))
	-@xargs -s 4000 -t -a clean_ASM_DEPS.tmp rm -rf
	-@rm -rf clean_ASM_DEPS.tmp
endif
ifneq ($(strip $(DSP_DEPS)),)
	-$(file > clean_DSP_DEPS.tmp, $(DSP_DEPS))
	-@xargs -s 4000 -t -a clean_DSP_DEPS.tmp rm -rf
	-@rm -rf clean_DSP_DEPS.tmp
endif
	-rm -rf src/*.asm src/*.p src/*.pp src/*.src src/*.jmp src/*.lst src/*.prn src/*.i src/*.ud 
	-rm -rf src/smc_gen/r_pincfg/*.asm src/smc_gen/r_pincfg/*.p src/smc_gen/r_pincfg/*.pp src/smc_gen/r_pincfg/*.src src/smc_gen/r_pincfg/*.jmp src/smc_gen/r_pincfg/*.lst src/smc_gen/r_pincfg/*.prn src/smc_gen/r_pincfg/*.i src/smc_gen/r_pincfg/*.ud 
	-rm -rf src/smc_gen/Config_MTU0/*.asm src/smc_gen/Config_MTU0/*.p src/smc_gen/Config_MTU0/*.pp src/smc_gen/Config_MTU0/*.src src/smc_gen/Config_MTU0/*.jmp src/smc_gen/Config_MTU0/*.lst src/smc_gen/Config_MTU0/*.prn src/smc_gen/Config_MTU0/*.i src/smc_gen/Config_MTU0/*.ud 
	-rm -rf src/smc_gen/Config_PORT/*.asm src/smc_gen/Config_PORT/*.p src/smc_gen/Config_PORT/*.pp src/smc_gen/Config_PORT/*.src src/smc_gen/Config_PORT/*.jmp src/smc_gen/Config_PORT/*.lst src/smc_gen/Config_PORT/*.prn src/smc_gen/Config_PORT/*.i src/smc_gen/Config_PORT/*.ud 
	-rm -rf src/smc_gen/r_bsp/mcu/rx130/*.asm src/smc_gen/r_bsp/mcu/rx130/*.p src/smc_gen/r_bsp/mcu/rx130/*.pp src/smc_gen/r_bsp/mcu/rx130/*.src src/smc_gen/r_bsp/mcu/rx130/*.jmp src/smc_gen/r_bsp/mcu/rx130/*.lst src/smc_gen/r_bsp/mcu/rx130/*.prn src/smc_gen/r_bsp/mcu/rx130/*.i src/smc_gen/r_bsp/mcu/rx130/*.ud 
	-rm -rf src/smc_gen/Config_S12AD0/*.asm src/smc_gen/Config_S12AD0/*.p src/smc_gen/Config_S12AD0/*.pp src/smc_gen/Config_S12AD0/*.src src/smc_gen/Config_S12AD0/*.jmp src/smc_gen/Config_S12AD0/*.lst src/smc_gen/Config_S12AD0/*.prn src/smc_gen/Config_S12AD0/*.i src/smc_gen/Config_S12AD0/*.ud 
	-rm -rf src/smc_gen/Config_CMT1/*.asm src/smc_gen/Config_CMT1/*.p src/smc_gen/Config_CMT1/*.pp src/smc_gen/Config_CMT1/*.src src/smc_gen/Config_CMT1/*.jmp src/smc_gen/Config_CMT1/*.lst src/smc_gen/Config_CMT1/*.prn src/smc_gen/Config_CMT1/*.i src/smc_gen/Config_CMT1/*.ud 
	-rm -rf src/smc_gen/Config_ICU/*.asm src/smc_gen/Config_ICU/*.p src/smc_gen/Config_ICU/*.pp src/smc_gen/Config_ICU/*.src src/smc_gen/Config_ICU/*.jmp src/smc_gen/Config_ICU/*.lst src/smc_gen/Config_ICU/*.prn src/smc_gen/Config_ICU/*.i src/smc_gen/Config_ICU/*.ud 
	-rm -rf src/smc_gen/Config_SCI12/*.asm src/smc_gen/Config_SCI12/*.p src/smc_gen/Config_SCI12/*.pp src/smc_gen/Config_SCI12/*.src src/smc_gen/Config_SCI12/*.jmp src/smc_gen/Config_SCI12/*.lst src/smc_gen/Config_SCI12/*.prn src/smc_gen/Config_SCI12/*.i src/smc_gen/Config_SCI12/*.ud 
	-rm -rf src/smc_gen/Config_SCI1/*.asm src/smc_gen/Config_SCI1/*.p src/smc_gen/Config_SCI1/*.pp src/smc_gen/Config_SCI1/*.src src/smc_gen/Config_SCI1/*.jmp src/smc_gen/Config_SCI1/*.lst src/smc_gen/Config_SCI1/*.prn src/smc_gen/Config_SCI1/*.i src/smc_gen/Config_SCI1/*.ud 
	-rm -rf src/smc_gen/Config_SCI5/*.asm src/smc_gen/Config_SCI5/*.p src/smc_gen/Config_SCI5/*.pp src/smc_gen/Config_SCI5/*.src src/smc_gen/Config_SCI5/*.jmp src/smc_gen/Config_SCI5/*.lst src/smc_gen/Config_SCI5/*.prn src/smc_gen/Config_SCI5/*.i src/smc_gen/Config_SCI5/*.ud 
	-rm -rf src/smc_gen/r_flash_rx/src/flash_type_4/*.asm src/smc_gen/r_flash_rx/src/flash_type_4/*.p src/smc_gen/r_flash_rx/src/flash_type_4/*.pp src/smc_gen/r_flash_rx/src/flash_type_4/*.src src/smc_gen/r_flash_rx/src/flash_type_4/*.jmp src/smc_gen/r_flash_rx/src/flash_type_4/*.lst src/smc_gen/r_flash_rx/src/flash_type_4/*.prn src/smc_gen/r_flash_rx/src/flash_type_4/*.i src/smc_gen/r_flash_rx/src/flash_type_4/*.ud 
	-rm -rf src/r_fw_up_rx/r_fw_up_rx/*.asm src/r_fw_up_rx/r_fw_up_rx/*.p src/r_fw_up_rx/r_fw_up_rx/*.pp src/r_fw_up_rx/r_fw_up_rx/*.src src/r_fw_up_rx/r_fw_up_rx/*.jmp src/r_fw_up_rx/r_fw_up_rx/*.lst src/r_fw_up_rx/r_fw_up_rx/*.prn src/r_fw_up_rx/r_fw_up_rx/*.i src/r_fw_up_rx/r_fw_up_rx/*.ud 
	-rm -rf src/smc_gen/Config_MTU3/*.asm src/smc_gen/Config_MTU3/*.p src/smc_gen/Config_MTU3/*.pp src/smc_gen/Config_MTU3/*.src src/smc_gen/Config_MTU3/*.jmp src/smc_gen/Config_MTU3/*.lst src/smc_gen/Config_MTU3/*.prn src/smc_gen/Config_MTU3/*.i src/smc_gen/Config_MTU3/*.ud 
	-rm -rf src/smc_gen/r_bsp/mcu/all/*.asm src/smc_gen/r_bsp/mcu/all/*.p src/smc_gen/r_bsp/mcu/all/*.pp src/smc_gen/r_bsp/mcu/all/*.src src/smc_gen/r_bsp/mcu/all/*.jmp src/smc_gen/r_bsp/mcu/all/*.lst src/smc_gen/r_bsp/mcu/all/*.prn src/smc_gen/r_bsp/mcu/all/*.i src/smc_gen/r_bsp/mcu/all/*.ud 
	-rm -rf src/smc_gen/general/*.asm src/smc_gen/general/*.p src/smc_gen/general/*.pp src/smc_gen/general/*.src src/smc_gen/general/*.jmp src/smc_gen/general/*.lst src/smc_gen/general/*.prn src/smc_gen/general/*.i src/smc_gen/general/*.ud 
	-rm -rf src/smc_gen/r_bsp/board/generic_rx130/*.asm src/smc_gen/r_bsp/board/generic_rx130/*.p src/smc_gen/r_bsp/board/generic_rx130/*.pp src/smc_gen/r_bsp/board/generic_rx130/*.src src/smc_gen/r_bsp/board/generic_rx130/*.jmp src/smc_gen/r_bsp/board/generic_rx130/*.lst src/smc_gen/r_bsp/board/generic_rx130/*.prn src/smc_gen/r_bsp/board/generic_rx130/*.i src/smc_gen/r_bsp/board/generic_rx130/*.ud 
	-rm -rf src/smc_gen/Config_CMT0/*.asm src/smc_gen/Config_CMT0/*.p src/smc_gen/Config_CMT0/*.pp src/smc_gen/Config_CMT0/*.src src/smc_gen/Config_CMT0/*.jmp src/smc_gen/Config_CMT0/*.lst src/smc_gen/Config_CMT0/*.prn src/smc_gen/Config_CMT0/*.i src/smc_gen/Config_CMT0/*.ud 
	-rm -rf src/smc_gen/r_flash_rx/src/*.asm src/smc_gen/r_flash_rx/src/*.p src/smc_gen/r_flash_rx/src/*.pp src/smc_gen/r_flash_rx/src/*.src src/smc_gen/r_flash_rx/src/*.jmp src/smc_gen/r_flash_rx/src/*.lst src/smc_gen/r_flash_rx/src/*.prn src/smc_gen/r_flash_rx/src/*.i src/smc_gen/r_flash_rx/src/*.ud 
	-rm -rf src/smc_gen/r_flash_rx/src/flash_type_1/*.asm src/smc_gen/r_flash_rx/src/flash_type_1/*.p src/smc_gen/r_flash_rx/src/flash_type_1/*.pp src/smc_gen/r_flash_rx/src/flash_type_1/*.src src/smc_gen/r_flash_rx/src/flash_type_1/*.jmp src/smc_gen/r_flash_rx/src/flash_type_1/*.lst src/smc_gen/r_flash_rx/src/flash_type_1/*.prn src/smc_gen/r_flash_rx/src/flash_type_1/*.i src/smc_gen/r_flash_rx/src/flash_type_1/*.ud 
	-rm -rf src/smc_gen/r_flash_rx/src/flash_type_3/*.asm src/smc_gen/r_flash_rx/src/flash_type_3/*.p src/smc_gen/r_flash_rx/src/flash_type_3/*.pp src/smc_gen/r_flash_rx/src/flash_type_3/*.src src/smc_gen/r_flash_rx/src/flash_type_3/*.jmp src/smc_gen/r_flash_rx/src/flash_type_3/*.lst src/smc_gen/r_flash_rx/src/flash_type_3/*.prn src/smc_gen/r_flash_rx/src/flash_type_3/*.i src/smc_gen/r_flash_rx/src/flash_type_3/*.ud 
	@echo ''
	@echo 'Clean complete.'

.PHONY: all clean  
-include ../makefile.targets