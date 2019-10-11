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
// Convert from a polynomial back into a single large number (data_out)
//   Polynomial has NUMSYBOLS where each symbol is a value with bitwidth INPUTSYMBOLBITWIDTH
//   
// Dual-outputs can be used to improve the latency
//
module convertfrommultisymbols 
#(
	parameter int OUTPUTBITWIDTH,
	parameter int NUMSYMBOLS,
	parameter int INPUTSYMBOLBITWIDTH,	// Includes any sign bit
	parameter int LOGRADIX,
	parameter int SYMBOLS_ARE_SIGNED,		// If 1 then each Symbol is treated as a signed value
	parameter int USEDUALOUTPUT = 0
) 
(
	input logic [INPUTSYMBOLBITWIDTH-1:0]	data_in		[NUMSYMBOLS],
	
	// If USEDUALOUTPUT==1, then result is sum of data_out and dataaux_out,
	// otherwise the result is contained in just data_out
	output logic [OUTPUTBITWIDTH-1:0]		data_out,
	output logic [OUTPUTBITWIDTH-1:0]		dataaux_out	
);


logic [NUMSYMBOLS*LOGRADIX-1:0] truncateddata;
logic [NUMSYMBOLS*LOGRADIX-1:0] carrys;
logic [NUMSYMBOLS*LOGRADIX-1:0] signs;

generate
	genvar i;
	for (i = 0; i < NUMSYMBOLS; i = i + 1) begin : converttosymbol
	
		if (SYMBOLS_ARE_SIGNED) begin : gensigned
		
			logic [INPUTSYMBOLBITWIDTH-2:0] udata;
			logic [INPUTSYMBOLBITWIDTH-1:0] dataSign;
			assign udata = data_in[i];
			assign dataSign = data_in[i] ^ { 1'b0, udata };
			
			assign truncateddata[(i * LOGRADIX) +: LOGRADIX] = udata;
			assign carrys[(i * LOGRADIX) +: LOGRADIX] = (udata >> LOGRADIX);
			assign signs[(i * LOGRADIX) +: LOGRADIX] = (dataSign >> LOGRADIX);
					
		end else begin : genunsigned
		
			// Truncate to INPUTSYMBOLBITWIDTH
			assign truncateddata[(i * LOGRADIX) +: LOGRADIX] = data_in[i];
			assign carrys[(i * LOGRADIX) +: LOGRADIX] = (data_in[i] >> LOGRADIX);
			assign signs[(i * LOGRADIX) +: LOGRADIX] = 0;
			
		end
		
	end
endgenerate;


logic [OUTPUTBITWIDTH-1:0] resizeddata;
logic [OUTPUTBITWIDTH-1:0] resizedcarrys;
logic [OUTPUTBITWIDTH-1:0] resizedsigns;

always_comb begin
    resizeddata = truncateddata;
    resizedcarrys = carrys;
    resizedcarrys = resizedcarrys << LOGRADIX;
    resizedsigns = signs;
    resizedsigns = resizedsigns << LOGRADIX;
end

generate
    if (USEDUALOUTPUT) begin
        assign data_out = resizeddata;
        assign dataaux_out = resizedcarrys - resizedsigns;
    end else begin
        assign data_out = resizeddata + resizedcarrys - resizedsigns;
        assign dataaux_out = 0;
    end
endgenerate;

endmodule
