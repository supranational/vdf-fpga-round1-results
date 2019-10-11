/*******************************************************************************
  Copyright 2019 Silicon Tailor Ltd

  Licensed under the Apache License, Version 2.0 (the "License");
  you may not use this file except in compliance with the License.
  You may obtain a copy of the License at

    http://www.apache.org/licenses/LICENSE-2.0

  Unless required by applicable law or agreed to in writing, software
  distributed under the License is distributed on an "AS IS" BASIS,
  WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
  See the License for the specific language governing permissions and
  limitations under the License.
*******************************************************************************/

`timescale  1 ns / 1 ns

//
// This block introduces a PLL and manages the Clock-crossing logic needed
//
// This interface does not allow the parent block to be stalled until reset has completed.
//   Because of this, if the remote clock is slower than 'clk' then it cannot be guaranteed that 
//   valid sq_out values will not continue to be received from a previous run even after reset has been asserted.
// A solution to this is to use a sequence-number system where all sq_out values are tagged with the current sequence-number.
//   The sequence number is incremented whenever a start pulse is asserted, and then all outputs with a different sequence-number can then be ignored.
//
module modular_square_metzgen_wrapper
#(
	parameter int MOD_LEN,
	parameter     MODULUS,
	parameter int USE_DENSE_ADDERS
)
(
	input logic                     clk,
	input logic                     reset,
	input logic                     start,
	input logic [MOD_LEN-1:0]       sq_in,
	output logic [MOD_LEN-1:0] 	    sq_out,
	output logic                    valid
);

localparam SEQNUMLEN = 4;

// Initialized sequence numbers
logic [SEQNUMLEN-1:0] sq_seqnum = 1;
logic [SEQNUMLEN-1:0] sq_nextseqnum = 2;




logic prevreset;
always @(posedge clk) begin
    prevreset <= reset;
end


// Note: vdf clock should always be slower than clk
logic vdf_clk;



// generate a reset pulse on startup to initialize the PLL
logic pllrst;
logic [16:0] pllrstcnt = 0;
always @(posedge clk) begin
	if (!pllrstcnt[16])
		pllrstcnt = pllrstcnt + 1;
end
assign pllrst = !pllrstcnt[16];


logic pllclk;
logic plllocked;

BUFG i_BUFG (
	.O	(pllclk),
	.I	(clk)
);

vdfpll i_vdfpll (
  .clk_out1		(vdf_clk),
  .reset		(pllrst),
  .locked		(plllocked),
  .clk_in1		(pllclk)
);

// Accelerator/Fifos should only be used when pll is locked
logic clklockedsync;
xpm_cdc_async_rst #(
    .RST_ACTIVE_HIGH(1)
) i_xpm_cdc_async_rst (
  .dest_arst(clklockedsync),
  .dest_clk(clk), 
  .src_arst(plllocked)  
);

logic [7:0] clklockedsyncdelay = 0;
always @(posedge clk) begin
	clklockedsyncdelay = (clklockedsyncdelay << 1) | clklockedsync;
end

logic acceleratorIsReady;
assign acceleratorIsReady = clklockedsyncdelay[7];

logic accFifoFull;

logic reset_pending = 0;
logic start_pending = 0;

logic [MOD_LEN-1:0] acc_sq_in;


// State machine for updating sequence-number for each batch
//   and latching any reset/start signals (until fifo is ready)
always @(posedge clk) begin
	if (~accFifoFull & acceleratorIsReady) begin
		if (reset_pending) 
			reset_pending = 0;
		else if (start_pending)
			start_pending = 0;
	end
	if (start) begin
		start_pending <= 1;
		acc_sq_in <= sq_in;
		sq_seqnum <= sq_nextseqnum;
		sq_nextseqnum <= sq_nextseqnum + 1;
	end
	if (reset) begin
		reset_pending = 1;
		start_pending = 0;
	end
end

logic accFifoDoPush;
assign accFifoDoPush = (~accFifoFull & acceleratorIsReady) & (reset_pending | start_pending);


logic					vdf_resetlevel;
logic					vdf_startorreset;
logic 					vdf_reset;
logic 					vdf_start;
logic [MOD_LEN-1:0]		vdf_sq_in;
logic [MOD_LEN-1:0]		vdf_sq_out;
logic [SEQNUMLEN-1:0]	vdf_sq_seqnum_in;
logic [SEQNUMLEN-1:0]	vdf_sq_seqnum_out = 3;
logic					vdf_valid;
logic                   vdf_ready_n;

dcfifo  #( 
  .WIDTH		(SEQNUMLEN + MOD_LEN + 1),
  .LOGDEPTH 	(5)
) i_dcfifo_to_vdf (
	.areset		(~acceleratorIsReady),		// synchronous to wrclk (=clk)

	.wrclk		(clk),
	.wrpush		(accFifoDoPush),      
	.wrdata		({ sq_seqnum, acc_sq_in, reset_pending }),
	.afull		(accFifoFull),

	.rdclk		(vdf_clk),
	.rdpop		(vdf_startorreset & ~vdf_ready_n),		// always pop as soon as data is available 
	.rddata		({ vdf_sq_seqnum_in, vdf_sq_in, vdf_resetlevel }),
	.rdvalid	(vdf_startorreset)
);

assign vdf_reset = vdf_startorreset & ~vdf_ready_n & vdf_resetlevel;
assign vdf_start = vdf_startorreset & ~vdf_ready_n & ~vdf_resetlevel;

// generate a reset pulse on startup to initialize the fifo from vdf
logic vdffiforst;
logic [5:0] vdffiforstcnt = 0;
always @(posedge vdf_clk) begin
	if (!vdffiforstcnt[5])
		vdffiforstcnt = vdffiforstcnt + 1;
end
assign vdffiforst = !vdffiforstcnt[5];


logic sq_out_valid;
logic [SEQNUMLEN-1:0] 	sq_seqnum_out;

dcfifo  #( 
  .WIDTH		(SEQNUMLEN + MOD_LEN),
  .LOGDEPTH 	(5)
) i_dcfifo_from_vdf (
	.areset		(vdffiforst),	// synchronous to wrclk (=vdf_clk)

	.wrclk		(vdf_clk),
	.wrpush		(vdf_valid),
	.wrdata		({ vdf_sq_seqnum_out, vdf_sq_out } ),
	.afull		(vdf_ready_n),

	.rdclk		(clk),
	.rdpop		(sq_out_valid & acceleratorIsReady),		// always pop as soon as data is available
	.rddata		({ sq_seqnum_out, sq_out }),
	.rdvalid	(sq_out_valid)
);

// Ignore data if it is from a different batch
assign valid = sq_out_valid & acceleratorIsReady & (sq_seqnum_out == sq_seqnum);

// Update sequence number in VDF clock domain
always @(posedge vdf_clk) begin
	if (vdf_reset) begin
		vdf_sq_seqnum_out <= vdf_sq_seqnum_in;
	end
end



modular_square_metzgen #(
	.MOD_LEN(MOD_LEN),
	.MODULUS(MODULUS),
	.USE_DENSE_ADDERS(USE_DENSE_ADDERS)
) modsqr (
	.clk        (vdf_clk),
	.reset      (vdf_reset),
	.start      (vdf_start),
	.sq_in      (vdf_sq_in),
	.sq_out     (vdf_sq_out),
	.valid      (vdf_valid)
);


endmodule
