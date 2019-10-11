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
// Convert a Polynomial into a new polynomial with reduced Symbol-bitwidth
//   This is achieved by adding 'excess' bits from each symbol to their next symbol
//
// Note: The uppermost symbol's excess bits are output using an extra symbol
//
module signedcarrycorrection 
#(
	parameter int NUMSYMBOLS,
	parameter int INPUTSYMBOLBITWIDTH,		// Includes a sign bit
	parameter int LOGRADIX
) 
(
	input logic [INPUTSYMBOLBITWIDTH-1:0]	data_in		[NUMSYMBOLS],
	output logic [LOGRADIX+1:0]  			data_out	[NUMSYMBOLS+1]
);


generate
	genvar i;
	for (i = 0; i <= NUMSYMBOLS; i = i + 1) begin : symboladd
	
		logic signed [INPUTSYMBOLBITWIDTH-1:0] currSymbol;
		logic signed [INPUTSYMBOLBITWIDTH-1:0] prevSymbol;
		
		assign currSymbol = (i < NUMSYMBOLS) ? data_in[i] : 0;
		assign prevSymbol = (i > 0) ? data_in[i-1] : 0;

		// Note: Sign-extend the top-bit in prevCarrys
		logic signed [LOGRADIX+1:0] prevCarrys;
		assign prevCarrys = prevSymbol >>> LOGRADIX;
						
		assign data_out[i] = { 2'b0, currSymbol[LOGRADIX-1:0] } + prevCarrys;
		
	end
endgenerate;



endmodule
