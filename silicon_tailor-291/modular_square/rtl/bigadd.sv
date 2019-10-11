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
// Implements a Carry-Select adder
//   Note: output is truncated to N bits
//
module bigadd 
#(
	parameter int N = 32
) 
(
	input logic [N-1:0]   	a_in,
	input logic [N-1:0]   	b_in,
	output logic [N-1:0]  	sum_out
);

localparam SYMBITWIDTH = 32;
localparam NUMSYMBOLS = (N + SYMBITWIDTH - 1) / SYMBITWIDTH;

// Symbol includes Carryout
logic [SYMBITWIDTH:0] sum_ab0 [NUMSYMBOLS];
logic [SYMBITWIDTH:0] sum_ab1 [NUMSYMBOLS];
logic [SYMBITWIDTH:0] sum_ab [NUMSYMBOLS];

logic [NUMSYMBOLS*SYMBITWIDTH-1:0] result;
   

generate
	genvar i;
	for (i = 0; i < NUMSYMBOLS; ++i) begin : sym
		logic [SYMBITWIDTH-1:0] syma;
		logic [SYMBITWIDTH-1:0] symb;
		logic prevcarry;
		assign syma = a_in >> (i * SYMBITWIDTH);
		assign symb = b_in >> (i * SYMBITWIDTH);
		assign sum_ab0[i] = ({ 1'b0, syma, 1'b0 } + { 1'b0, symb, 1'b0 }) >> 1;		// syma + symb + 0
		assign sum_ab1[i] = ({ 1'b0, syma, 1'b1 } + { 1'b0, symb, 1'b1 }) >> 1;		// syma + symb + 1
		assign prevcarry = (i == 0) ? 1'b0 : sum_ab[i-1][SYMBITWIDTH];
		assign sum_ab[i] = (prevcarry ? sum_ab1[i] : sum_ab0[i]);
		assign result[i * SYMBITWIDTH +: SYMBITWIDTH] = sum_ab[i];
	end
endgenerate;


assign sum_out = result;

   
endmodule
