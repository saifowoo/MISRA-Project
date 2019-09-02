################################################################################
# Automatically-generated file. Do not edit!
################################################################################

SHELL = cmd.exe

# Each subdirectory must supply rules for building sources it contributes
%.obj: ../%.c $(GEN_OPTS) | $(GEN_FILES)
	@echo 'Building file: "$<"'
	@echo 'Invoking: ARM Compiler'
	"E:/program files/cc901/ccs/tools/compiler/ti-cgt-arm_18.12.1.LTS/bin/armcl" -mv4 --code_state=32 --include_path="D:/ENGINEERING/EMBEDDED/CC/MISRA Challenge" --include_path="E:/program files/cc901/ccs/tools/compiler/ti-cgt-arm_18.12.1.LTS/include" -g --diag_warning=225 --diag_wrap=off --display_error_number --check_misra="all,-1" --misra_advisory=error --misra_required=error --preproc_with_compile --preproc_dependency="$(basename $(<F)).d_raw" $(GEN_OPTS__FLAG) "$<"
	@echo 'Finished building: "$<"'
	@echo ' '


