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
// Adds three inputs and a carry
//   Note: truncates output to N bits
//
module bigadd3 
#(
	parameter int N = 32
) 
(
	input logic [N-1:0]   	a_in,
	input logic [N-1:0]   	b_in,
	input logic [N-1:0]   	c_in,
	input logic 			carry_in,
	output logic [N-1:0]  	sum_out
);


logic [N-1:0] sums; 
logic [N-1:0] carrys;

assign sums = { a_in ^ b_in ^ c_in };
assign carrys = { (a_in & b_in) | (a_in & c_in) | (b_in & c_in), carry_in };

(* keep_hierarchy = "no" *)
bigadd #( .N(N) ) i_bigadd (
	.a_in		(sums),
	.b_in		(carrys),
	.sum_out	(sum_out)
);


endmodule
