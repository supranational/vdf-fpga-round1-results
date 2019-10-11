
# This works
#create_clock -period 29.0 -name ap_clk -waveform {0.000 14.5} [get_ports ap_clk]

# Requires 125MHz clock
create_clock -period 8.0 -name ap_clk -waveform {0.000 4.0} [get_ports ap_clk]

create_pblock sl_exclusion
resize_pblock [get_pblocks sl_exclusion] -add {CLOCKREGION_X4Y0:CLOCKREGION_X5Y9}
set_property EXCLUDE_PLACEMENT 1 [get_pblocks sl_exclusion]
create_pblock SLR2
add_cells_to_pblock [get_pblocks SLR2] [get_cells -quiet [list inst_wrapper/inst_kernel/msu/modsqr/modsqr/modsqr/i_modsqr_iter]]
resize_pblock [get_pblocks SLR2] -add {CLOCKREGION_X0Y10:CLOCKREGION_X5Y14}

#set_property CLOCK_DEDICATED_ROUTE ANY_CMT_COLUMN [get_nets WRAPPER_INST/SH/kernel_clks_i/clkwiz_kernel_clk0/inst/CLK_CORE_DRP_I/clk_inst/clk_out1]