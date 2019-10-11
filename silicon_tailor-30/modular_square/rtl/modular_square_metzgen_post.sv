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

//`define USESIMPLEIMPL


//
// Post-processing computes sq_out = sq_in % MODULUS
//   This is not timing critical, so is done in a straight-forward way.
//
// Modulo-lookup tables are used to produce a small set of terms that are added together
//   using standard modulo-adders to guarantee the result is MOD_LEN bits wide
//
module modular_square_metzgen_post
#(
	parameter int 	MOD_LEN,
	parameter 		MODULUS
)
(
	input logic                     clk,
	input logic                     reset,
	input logic                     valid_in,
	input logic [MOD_LEN+34:0]      sqa_in,
	input logic [MOD_LEN+34:0]      sqb_in,
	output logic [MOD_LEN-1:0] 	    sq_out,
	output logic                    valid_out
);

import vdfpackage::bigmod;


// Two cycle latency
logic valid_reg = 0;

always_ff @(posedge clk) begin
	if (reset) begin
		valid_reg <= 0;
		valid_out <= 0;
	end else begin
		valid_reg <= valid_in;
		valid_out <= valid_reg;
	end
end


`ifdef USESIMPLEIMPL

logic [MOD_LEN+34:0] sq_sum;

always_ff @(posedge clk) begin
	sq_sum <= sqa_in + sqb_in;
	sq_out <= bigmod(sq_sum, MODULUS);
end

`else

logic [MOD_LEN-1:0]	modterm [8];

assign modterm[0] = { 1'b0, sqa_in[MOD_LEN-2:0] };		// Note: this is always less than MODULUS
assign modterm[1] = { 1'b0, sqb_in[MOD_LEN-2:0] };		// Note: this is always less than MODULUS

// compute the sum of the upper bits, as this will be used for Modulo-lookup tables
logic [35:0] excess_sq_bits;
assign excess_sq_bits = (sqa_in >> (MOD_LEN-1)) + (sqb_in >> (MOD_LEN-1));

generate
	genvar i;
    for (i = 0; i < 6; ++i) begin : postmod
        modulolut6 #(
            .MODULUS		(MODULUS),
            .MODBITWIDTH	(MOD_LEN),
            .MOD0			(bigmod(4096'h1 << (MOD_LEN+6*i-1), MODULUS)),
            .MOD1			(bigmod(4096'h1 << (MOD_LEN+6*i+0), MODULUS)),
            .MOD2			(bigmod(4096'h1 << (MOD_LEN+6*i+1), MODULUS)),
            .MOD3			(bigmod(4096'h1 << (MOD_LEN+6*i+2), MODULUS)),
            .MOD4			(bigmod(4096'h1 << (MOD_LEN+6*i+3), MODULUS)),
            .MOD5			(bigmod(4096'h1 << (MOD_LEN+6*i+4), MODULUS))
        ) i_modulolut6 (
            .x0_in			(excess_sq_bits[6*i+0]),
            .x1_in			(excess_sq_bits[6*i+1]),
            .x2_in			(excess_sq_bits[6*i+2]),
            .x3_in			(excess_sq_bits[6*i+3]),
            .x4_in			(excess_sq_bits[6*i+4]),	
            .x5_in			(excess_sq_bits[6*i+5]),	
            .value_out		(modterm[i+2])
        );
    end
endgenerate

//////////////////////////////////////

logic [MOD_LEN-1:0]	firststagesum [4];

generate
    for (i = 0; i < 4; ++i) begin : i_firststagesum
        moduloadd #(
            .MODULUS		(MODULUS),
            .MOD_LEN       	(MOD_LEN)
        ) i_firststagesum (
            .a_in			(modterm[2*i+0]),
            .b_in			(modterm[2*i+1]),
            .value_out		(firststagesum[i])
        );
    end
endgenerate


logic [MOD_LEN-1:0]	firststagesum_reg [4];

always_ff @(posedge clk) begin
	firststagesum_reg <= firststagesum;
end

//////////////////////////////////////

logic [MOD_LEN-1:0]	secondstagesum [2];

moduloadd #(
    .MODULUS		(MODULUS),
    .MOD_LEN       	(MOD_LEN)
) i_secondstagesum0 (
    .a_in			(firststagesum_reg[0]),
    .b_in			(firststagesum_reg[1]),
    .value_out		(secondstagesum[0])
);

moduloadd #(
    .MODULUS		(MODULUS),
    .MOD_LEN       	(MOD_LEN)
) i_secondstagesum1 (
    .a_in			(firststagesum_reg[2]),
    .b_in			(firststagesum_reg[3]),
    .value_out		(secondstagesum[1])
);


logic [MOD_LEN-1:0]	finalstagesum;

moduloadd #(
	.MODULUS		(MODULUS),
	.MOD_LEN       	(MOD_LEN)
) i_finalstagesum (
	.a_in			(secondstagesum[0]),
	.b_in			(secondstagesum[1]),
	.value_out		(finalstagesum)
);

always_ff @(posedge clk) begin
	sq_out <= finalstagesum;
end


`endif


endmodule
