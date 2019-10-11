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
// Convert a large number (data_in) into a polynomial consisting of
//   NUMSYBOLS where each symbol is an unsigned value with bitwidth OUTPUTSYMBOLBITWIDTH
//
module converttomultisymbols 
#(
	parameter int INPUTBITWIDTH,
	parameter int NUMSYMBOLS,
	parameter int OUTPUTSYMBOLBITWIDTH,
	parameter int LOGRADIX
) 
(
	input logic [INPUTBITWIDTH-1:0]				data_in,
	output logic [OUTPUTSYMBOLBITWIDTH-1:0]		data_out	[NUMSYMBOLS]
);


generate
	genvar i;
	for (i = 0; i < NUMSYMBOLS; i = i + 1) begin : converttosymbol
	
		logic [LOGRADIX-1:0] currSymbol;
		assign currSymbol = data_in >> (i * LOGRADIX);
		
		// Zero extend to OUTPUTSYMBOLBITWIDTH
		assign data_out[i] = currSymbol;
		
	end
endgenerate;



endmodule
