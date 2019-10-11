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

module sum4to1
#(
	parameter int N,
	parameter int USE_DENSE_ADDERS
) 
(
	input logic [N-1:0]   	a_in,
	input logic [N-1:0]   	b_in,
	input logic [N-1:0]   	c_in,
	input logic [N-1:0]   	d_in,
	input logic				carry0_in,
	input logic				carry1_in,
	output logic [N+1:0]  	sum_out
);

generate
	if (USE_DENSE_ADDERS) begin : genDense

		logic [N+1:0] partialsum;
		logic [N-1:0] splitMask = { (N+1)/2 { 2'b01 } };

		// Xilinx fpgas can implement 2.5:1 adders by using the LUT5s that preceed the adder logic
		add3 #( .N(N) ) i_add3a (
			.a_in		(a_in),
			.b_in		(b_in),
			.c_in		(c_in & splitMask),		// only uses odd bits in c_in
			.carry_in	(carry0_in),
			.sum_out	(partialsum)
		);

		add3 #( .N(N+2) ) i_add3b (
			.a_in		({2'b0, c_in & ~splitMask}),
			.b_in		({2'b0, d_in }),
			.c_in		(partialsum),	
			.carry_in	(carry1_in),
			.sum_out	(sum_out)
		);
		
	end else begin : genSimple

		assign sum_out = 
			({2'b0, a_in} + {2'b0, b_in} + carry0_in) +
			({2'b0, c_in} + {2'b0, d_in} + carry1_in);	
	
	end
endgenerate;


endmodule
