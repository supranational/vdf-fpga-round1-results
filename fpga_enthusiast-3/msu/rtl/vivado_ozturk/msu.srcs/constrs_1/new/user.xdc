
create_clock -period 8.000 -name ap_clk -waveform {0.000 4.000} [get_ports ap_clk]

create_pblock sl_exclusion
resize_pblock [get_pblocks sl_exclusion] -add {CLOCKREGION_X4Y0:CLOCKREGION_X5Y9}
set_property EXCLUDE_PLACEMENT 1 [get_pblocks sl_exclusion]
create_pblock SLR2
add_cells_to_pblock [get_pblocks SLR2] [get_cells -quiet [list inst_wrapper/inst_kernel/msu/modsqr/modsqr]]
resize_pblock [get_pblocks SLR2] -add {CLOCKREGION_X0Y10:CLOCKREGION_X5Y14}

#set_false_path -from [get_clocks -of_objects [get_pins inst_wrapper/inst_kernel/msu/squarer_mmcm/inst/mmcme4_adv_inst/CLKOUT0]] -to [get_clocks ap_clk]
#set_false_path -from [get_clocks ap_clk] -to [get_clocks -of_objects [get_pins inst_wrapper/inst_kernel/msu/squarer_mmcm/inst/mmcme4_adv_inst/CLKOUT0]]
