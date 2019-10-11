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
// Perform modulo addition using LUT6s
//
// value_out = (
//		( x0_in * MOD0 ) +
//  	( x1_in * MOD1 ) + 
//  	( x2_in * MOD2 ) + 
//  	( x3_in * MOD3 ) + 
// 		( x4_in * MOD4 ) + 
//		( x5_in * MOD5 )) % MODULUS
//
module modulolut6
#(
	parameter MODULUS,
	parameter int MODBITWIDTH,
	parameter MOD0,
	parameter MOD1,
	parameter MOD2,
	parameter MOD3,
	parameter MOD4,
	parameter MOD5
) 
(
	input logic 						x0_in,
	input logic 						x1_in,
	input logic 						x2_in,
	input logic 						x3_in,
	input logic 						x4_in,
	input logic 						x5_in,
	output logic [MODBITWIDTH-1:0]		value_out
);
   
      
import vdfpackage::bigmod;

function bit [MODBITWIDTH-1:0] modfn(
	bit [5:0] x
	);
	logic [MODBITWIDTH:0] term;
	begin
		term = 0;
		if (x[0])
			term = term + MOD0;
		if (term > MODULUS)
			term = term - MODULUS;
		if (x[1])
			term = term + MOD1;
		if (term > MODULUS)
			term = term - MODULUS;
		if (x[2])
			term = term + MOD2;
		if (term > MODULUS)
			term = term - MODULUS;
		if (x[3])
			term = term + MOD3;
		if (term > MODULUS)
			term = term - MODULUS;
		if (x[4])
			term = term + MOD4;
		if (term > MODULUS)
			term = term - MODULUS;
		if (x[5])
			term = term + MOD5;
		if (term > MODULUS)
			term = term - MODULUS;
		modfn = term;
	end
endfunction


logic [MODBITWIDTH-1:0] modvalues [64];

generate
	genvar i;
	for (i = 0; i < 64; i = i + 1) begin : computemod
		assign modvalues[i] = modfn(i);	
	end
endgenerate;

assign value_out = modvalues[{x5_in, x4_in, x3_in, x2_in, x1_in, x0_in}];


endmodule
