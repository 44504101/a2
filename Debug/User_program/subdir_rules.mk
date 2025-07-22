################################################################################
# Automatically-generated file. Do not edit!
################################################################################

SHELL = cmd.exe

# Each subdirectory must supply rules for building sources it contributes
User_program/%.obj: ../User_program/%.c $(GEN_OPTS) | $(GEN_FILES) $(GEN_MISC_FILES)
	@echo 'Building file: "$<"'
	@echo 'Invoking: C2000 Compiler'
	"D:/ti/ccs1200/ccs/tools/compiler/ti-cgt-c2000_16.9.11.LTS/bin/cl2000" -v28 -ml -mt --float_support=fpu32 --include_path="F:/WS/DSP28335_PMSM_XBEV/Basic_include" --include_path="D:/ti/ccs1200/ccs/tools/compiler/ti-cgt-c2000_16.9.11.LTS/include" --include_path="/packages/ti/xdais" --include_path="F:/WS/DSP28335_PMSM_XBEV/MotorControl_include" --include_path="F:/WS/DSP28335_PMSM_XBEV/Drive_include" --include_path="F:/WS/DSP28335_PMSM_XBEV/DSP2833x_include" --include_path="F:/WS/DSP28335_PMSM_XBEV/User_include" --advice:performance=all --define=_DEBUG --define=LARGE_MODEL -g --diag_warning=225 --issue_remarks --verbose_diagnostics --quiet --output_all_syms --preproc_with_compile --preproc_dependency="User_program/$(basename $(<F)).d_raw" --obj_directory="User_program" $(GEN_OPTS__FLAG) "$<"
	@echo 'Finished building: "$<"'
	@echo ' '


