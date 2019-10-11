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
// Implements modulo-addition on two unsigned numbers 
//   Simple method used.
//
// value_out = (a_in + b_in) % MODULUS
//
module moduloadd
#(
	parameter int 	MOD_LEN,
	parameter 		MODULUS
)
(
	input logic [MOD_LEN-1:0]       a_in,		// should be less than MODULUS
	input logic [MOD_LEN-1:0]       b_in,		// should be less than MODULUS
	output logic [MOD_LEN-1:0] 	    value_out
);


// Internal values have an extra bit of precision as well as a sign bit
logic [MOD_LEN+1:0] sum;
logic [MOD_LEN+1:0] sumMinusM;

logic [MOD_LEN+1:0] negM;
assign negM = 0 - MODULUS;

bigadd #( .N(MOD_LEN+2) ) i_addab (
	.a_in			({2'b0, a_in}),
	.b_in			({2'b0, b_in}),
	.sum_out		(sum)
);

bigadd3 #( .N(MOD_LEN+2) ) i_addabminusm (
	.a_in			({2'b0, a_in}),
	.b_in			({2'b0, b_in}),
	.c_in			(negM),
	.carry_in		(1'b0),
	.sum_out		(sumMinusM)
);

// Only pick sum if sumMinusM is negative
assign value_out = sumMinusM[MOD_LEN+1] ? sum : sumMinusM;


endmodule
