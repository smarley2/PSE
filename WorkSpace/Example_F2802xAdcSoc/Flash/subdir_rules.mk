################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Each subdirectory must supply rules for building sources it contributes
Example_2802xAdcSoc.obj: ../Example_2802xAdcSoc.c $(GEN_OPTS) $(GEN_HDRS)
	@echo 'Building file: $<'
	@echo 'Invoking: C2000 Compiler'
	"C:/ti/ccsv6/tools/compiler/ti-cgt-c2000_6.4.2/bin/cl2000" -v28 -ml -mt -O2 -g --include_path="C:/ti/ccsv6/tools/compiler/ti-cgt-c2000_6.4.2/include" --include_path="/packages/ti/xdais" --include_path="C:/projetos/PSE/trunk/f2802x_common/include" --include_path="C:/projetos/PSE/trunk/f2802x_headers/include" --include_path="C:/projetos/PSE/trunk" --include_path="C:/libs/math/IQmath/v15c/include" --define="_DEBUG" --define="_FLASH" --define="LARGE_MODEL" --quiet --verbose_diagnostics --diag_warning=225 --diag_suppress=232 --diag_suppress=10063 --output_all_syms --cdebug_asm_data --preproc_with_compile --preproc_dependency="Example_2802xAdcSoc.pp" $(GEN_OPTS__FLAG) "$<"
	@echo 'Finished building: $<'
	@echo ' '


