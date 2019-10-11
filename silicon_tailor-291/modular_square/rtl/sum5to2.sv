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

module sum5to2 
#(
	parameter int N,
	parameter int USE_DENSE_ADDERS
) 
(
	input logic [N-1:0]   	a_in,
	input logic [N-1:0]   	b_in,
	input logic [N-1:0]   	c_in,
	input logic [N-1:0]   	d_in,
	input logic [N-1:0]   	e_in,
	output logic [N+1:0]  	sumA_out,
	output logic [N+1:0]  	sumB_out
);

generate
	if (USE_DENSE_ADDERS) begin : genDense

		logic [N-1:0] splitMask = { (N+1)/2 { 2'b01 } };

		// Xilinx fpgas can implement 2.5:1 adders by using the LUT5s that preceed the adder logic
		add3 #( .N(N) ) i_add3a (
			.a_in		(a_in),
			.b_in		(b_in),
			.c_in		(c_in & splitMask),		// only uses odd bits in c_in
			.carry_in	(1'b0),
			.sum_out	(sumA_out)
		);

		add3 #( .N(N) ) i_add3b (
			.a_in		(d_in),
			.b_in		(e_in),
			.c_in		(c_in & ~splitMask),		// only uses even bits in c_in
			.carry_in	(1'b0),
			.sum_out	(sumB_out)
		);
		
	end else begin : genSimple

		logic [N+1:0] expectedsum;
		logic [N+1:0] actualsum;
		assign sumA_out = {2'b0, a_in} + {2'b0, b_in} + {2'b0, c_in};	
		assign sumB_out = {2'b0, d_in} + {2'b0, e_in};	
	
	end
endgenerate;


endmodule
