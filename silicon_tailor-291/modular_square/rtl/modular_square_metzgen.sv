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

module modular_square_metzgen
#(
	parameter int 	MOD_LEN,
	parameter 		MODULUS,
	parameter int   USE_DENSE_ADDERS
)
(
	input logic                     clk,
	input logic                     reset,
	input logic                     start,
	input logic [MOD_LEN-1:0]       sq_in,
	output logic [MOD_LEN-1:0] 	    sq_out,
	output logic                    valid
);

	localparam int IO_STAGES = 3;
	
	// IO Stages are needed to cross SLR regions without incurring a latency penalty for the core logic
	logic               start_stages[IO_STAGES];
	logic               reset_stages[IO_STAGES];
	logic [MOD_LEN-1:0] sq_in_stages[IO_STAGES];
	logic [MOD_LEN+34:0] sqa_out_stages[IO_STAGES];
	logic [MOD_LEN+34:0] sqb_out_stages[IO_STAGES];
	logic               valid_stages[IO_STAGES];
	logic [(2*IO_STAGES)-1:0] ignore_valid;
		
	always_ff @(posedge clk) begin
		start_stages[0] <= start;
		reset_stages[0] <= reset;
		sq_in_stages[0] <= sq_in;
		if (reset)
			ignore_valid <= 0 - 1;
		else
			ignore_valid <= ignore_valid >> 1;
	end
		
	// Create the pipeline
	generate
		genvar j;
		for(j = 1; j < IO_STAGES; j++) begin
			always_ff @(posedge clk) begin
				start_stages[j]  <= start_stages[j-1];
				reset_stages[j]  <= reset_stages[j-1];
				sq_in_stages[j]  <= sq_in_stages[j-1];
				sqa_out_stages[j] <= sqa_out_stages[j-1];
				sqb_out_stages[j] <= sqb_out_stages[j-1];
				valid_stages[j]  <= valid_stages[j-1];
			end
		end
	endgenerate

	// This block can reside in SLR2 as all inputs and outputs are pipelined to cross SLR boundary
	(* keep_hierarchy = "yes" *)
	modular_square_metzgen_iter #(
		.MOD_LEN(MOD_LEN),
		.MODULUS(MODULUS),
		.USE_DENSE_ADDERS(USE_DENSE_ADDERS)
	) i_modsqr_iter (
		.clk        (clk),
		.reset      (reset_stages[IO_STAGES-1]),
		.start      (start_stages[IO_STAGES-1]),
		.sq_in      (sq_in_stages[IO_STAGES-1]),
		.sqa_out    (sqa_out_stages[0]),
		.sqb_out    (sqb_out_stages[0]),
		.valid      (valid_stages[0])
	);
	
	logic postvalid;
	
	// This block corrects the output into standard 2s complement form
	//   This block does not need to reside in SLR2, and is not performance critical
	(* keep_hierarchy = "yes" *)
	modular_square_metzgen_post #(
		.MOD_LEN(MOD_LEN),
		.MODULUS(MODULUS)
	) i_modsqr_post (
		.clk        (clk),
		.reset      (reset),
		.valid_in   (valid_stages[IO_STAGES-1] & ~ignore_valid[0]),
		.sqa_in     (sqa_out_stages[IO_STAGES-1]),
		.sqb_in     (sqb_out_stages[IO_STAGES-1]),
		.sq_out     (sq_out),
		.valid_out  (postvalid)
	);
	
    assign valid = postvalid & ~ignore_valid[0];


endmodule
