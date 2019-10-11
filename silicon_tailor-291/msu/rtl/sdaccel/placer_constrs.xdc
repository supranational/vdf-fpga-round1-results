add_cells_to_pblock [get_pblocks pblock_dynamic_SLR2] [get_cells [list {WRAPPER_INST/CL/vdf_1/inst/inst_wrapper/inst_kernel/msu/modsqr/modsqr/modsqr/i_modsqr_iter}]]
add_cells_to_pblock [get_pblocks pblock_dynamic_SLR1] [get_cells [list {WRAPPER_INST/CL/vdf_1/inst/inst_wrapper/inst_kernel/msu/modsqr/modsqr/modsqr/i_modsqr_post}]]
add_cells_to_pblock [get_pblocks pblock_dynamic_SLR2] [get_cells [list {WRAPPER_INST/CL/vdf_1/inst/inst_wrapper/inst_kernel/msu/modsqr/modsqr/i_vdfpll}]]


set_property CLOCK_DEDICATED_ROUTE ANY_CMT_COLUMN [get_nets WRAPPER_INST/SH/kernel_clks_i/clkwiz_kernel_clk0/inst/CLK_CORE_DRP_I/clk_inst/clk_out1]
