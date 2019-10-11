`timescale  1 ns / 1 ns

//
// Wrapper for a Xilinx Dual-Clock Fifo
//
module dcfifo 
#( 
  parameter integer  WIDTH = 32,
  parameter integer  LOGDEPTH = 5
)
(
	input logic					areset,

	input logic					wrclk,
	input logic					wrpush,
	input logic [WIDTH-1:0]		wrdata,
	output logic 				afull,		// almost-full

	input logic					rdclk,
	input logic					rdpop,
	output logic [WIDTH-1:0]	rddata,
	output logic 				rdvalid
);


logic empty;

// xpm_fifo_async: Asynchronous FIFO
// Xilinx Parameterized Macro, Version 2016.4
xpm_fifo_async # (
  .FIFO_MEMORY_TYPE          ("auto"),          //string; "auto", "block", "distributed", or "ultra";
  .ECC_MODE                  ("no_ecc"),        //string; "no_ecc" or "en_ecc";
  .RELATED_CLOCKS            (0),               //positive integer; 0 or 1
  .FIFO_WRITE_DEPTH          (1 << LOGDEPTH),   //positive integer
  .WRITE_DATA_WIDTH          (WIDTH),        	//positive integer
  .WR_DATA_COUNT_WIDTH       (LOGDEPTH+1),      //positive integer, Not used
  .PROG_FULL_THRESH          (8),               //positive integer
  .FULL_RESET_VALUE          (1),               //positive integer; 0 or 1
  .READ_MODE                 ("fwft"),          //string; "std" or "fwft";
  .FIFO_READ_LATENCY         (1),               //positive integer;
  .READ_DATA_WIDTH           (WIDTH),           //positive integer
  .RD_DATA_COUNT_WIDTH       (LOGDEPTH+1),      //positive integer, not used
  .PROG_EMPTY_THRESH         (8),        		//positive integer, not used 
  .DOUT_RESET_VALUE          ("0"),             //string, don't care
  .CDC_SYNC_STAGES           (3),               //positive integer
  .WAKEUP_TIME               (0)                //positive integer; 0 or 2;

) inst_rd_xpm_fifo_async (
  .rst           (areset),
  .wr_clk        (wrclk),
  .wr_en         (wrpush),
  .din           (wrdata),
  .full          (),
  .overflow      (),
  .wr_rst_busy   (),
  .rd_clk        (rdclk),
  .rd_en         (rdpop),
  .dout          (rddata),
  .empty         (empty),
  .underflow     (),
  .rd_rst_busy   (),
  .prog_full     (afull),
  .wr_data_count (),
  .prog_empty    (),
  .rd_data_count (),
  .sleep         (1'b0),
  .injectsbiterr (1'b0),
  .injectdbiterr (1'b0),
  .sbiterr       (),
  .dbiterr       () 
);

assign rdvalid = ~empty;

endmodule