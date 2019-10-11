
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


module multisymbolsquarer 
#(
	parameter int LOGNUMSYMBOLS,
	parameter int INPUTSYMBOLBITWIDTH,	
	parameter int LOGRADIX,
	parameter int USE_DENSE_ADDERS
) 
(
	input logic clk,
	
	// input is unsigned
	input logic [INPUTSYMBOLBITWIDTH-1:0]								data_in		[1 << LOGNUMSYMBOLS],

	// output is signed
	output logic [2*INPUTSYMBOLBITWIDTH - LOGRADIX + 4*LOGNUMSYMBOLS:0]	data_out	[2 << LOGNUMSYMBOLS]
);


generate
	genvar i;
	if (LOGNUMSYMBOLS == 0) begin : genBaseCase
	
		// Base case:
		// ==========
		//   Square a single Symbol
		//   Note: This results in *two* Symbols

        logic [2*INPUTSYMBOLBITWIDTH-1:0] sqrresult;

		squarer #(
			.INPUTWIDTH 	(INPUTSYMBOLBITWIDTH)
		) i_squarer (
			.data_in	(data_in[0]),
			.data_out	(sqrresult)
		);

		// Note: sqr result should be positive
        assign data_out[0] = sqrresult[LOGRADIX-1:0];
        assign data_out[1] = sqrresult >> LOGRADIX;

	end else begin : genRecurse
	
		// Recursive general case:
		// =======================
		//    Input can be written as { [ A ][ B ] }
		//      where A is all the upper-half symbols, and B is all lower-half symbols
		//
		//    Square({ A, B }):
		//                  [ A ][ B ]
		//                * [ A ][ B ]
		//    ========================
		//                  [  B*B   ]
		//           [  2*A*B   ][ 0 ]
		//  +     [  A*A   ][ 0 ][ 0 ]
		//    ========================
		//        [  A*A   ][  B*B   ]
		//  +        [  2*A*B   ][ 0 ]			Note that: (A+B)^2 = A^2 + 2*A*B + B^2
		//    ========================          
		//        [  A*A   ][  B*B   ]          Therefore: 2*A*B = (A+B)^2 - A^2 - B^2
		//  +         [ (A+B)^2 ][ 0 ]
		//  -         [   A*A   ][ 0 ]
		//  -         [   B*B   ][ 0 ]
		//    ========================
		//     [Upper][  Middle ][Lower] <-- Sections
		//    ========================

		localparam HALFNUMSYMBOLS = 1 << (LOGNUMSYMBOLS-1);
		localparam OUTPUTSYMBOLBITWIDTH = 2*INPUTSYMBOLBITWIDTH - LOGRADIX + 4*LOGNUMSYMBOLS + 1;
	
		logic [INPUTSYMBOLBITWIDTH-1:0] 	upperdata 	[HALFNUMSYMBOLS];
		logic [INPUTSYMBOLBITWIDTH:0] 		middledata 	[HALFNUMSYMBOLS];
		logic [INPUTSYMBOLBITWIDTH-1:0] 	lowerdata 	[HALFNUMSYMBOLS];
	
		for (i = 0; i < HALFNUMSYMBOLS; i = i + 1) begin : split	
			assign upperdata[i] = data_in[i + HALFNUMSYMBOLS];
			assign lowerdata[i] = data_in[i];
			assign middledata[i] = { 1'b0, upperdata[i] } + { 1'b0, lowerdata[i] };			
		end
		
		logic [OUTPUTSYMBOLBITWIDTH-5:0] raw_sqrupperdata [1 << LOGNUMSYMBOLS];
		logic [OUTPUTSYMBOLBITWIDTH-3:0] raw_sqrmiddledata [1 << LOGNUMSYMBOLS];
		logic [OUTPUTSYMBOLBITWIDTH-5:0] raw_sqrlowerdata [1 << LOGNUMSYMBOLS];

		multisymbolsquarer #(
			.LOGNUMSYMBOLS			(LOGNUMSYMBOLS - 1),
			.INPUTSYMBOLBITWIDTH 	(INPUTSYMBOLBITWIDTH),	
			.LOGRADIX 				(LOGRADIX),
			.USE_DENSE_ADDERS       (USE_DENSE_ADDERS)
		) i_uppermssqr (
			.data_in	(upperdata),
			.data_out	(raw_sqrupperdata)
		);
		
		multisymbolsquarer #(
			.LOGNUMSYMBOLS			(LOGNUMSYMBOLS - 1),
			.INPUTSYMBOLBITWIDTH 	(INPUTSYMBOLBITWIDTH + 1),	
			.LOGRADIX 				(LOGRADIX),
			.USE_DENSE_ADDERS       (USE_DENSE_ADDERS)
		) i_middlemssqr (
			.data_in	(middledata),
			.data_out	(raw_sqrmiddledata)
		);

		multisymbolsquarer #(
			.LOGNUMSYMBOLS			(LOGNUMSYMBOLS - 1),
			.INPUTSYMBOLBITWIDTH 	(INPUTSYMBOLBITWIDTH),	
			.LOGRADIX 				(LOGRADIX),
			.USE_DENSE_ADDERS       (USE_DENSE_ADDERS)
		) i_lowermssqr (
			.data_in	(lowerdata),
			.data_out	(raw_sqrlowerdata)
		);

		logic signed [OUTPUTSYMBOLBITWIDTH-1:0] sqrupperdata [1 << LOGNUMSYMBOLS];
		logic signed [OUTPUTSYMBOLBITWIDTH-1:0] sqrmiddledata [1 << LOGNUMSYMBOLS];
		logic signed [OUTPUTSYMBOLBITWIDTH-1:0] sqrlowerdata [1 << LOGNUMSYMBOLS];

		// sign extend to OUTPUTSYMBOLBITWIDTH
		for (i = 0; i < (1 << LOGNUMSYMBOLS); i = i + 1) begin : signext
			assign sqrupperdata[i] = $signed(raw_sqrupperdata[i]);
			assign sqrmiddledata[i] = $signed(raw_sqrmiddledata[i]);
			assign sqrlowerdata[i] = $signed(raw_sqrlowerdata[i]);
        end

		for (i = 0; i < (2 << LOGNUMSYMBOLS); i = i + 1) begin : persymbol
		
			if (i < HALFNUMSYMBOLS) begin : genupper
			
				// Lower section
				assign data_out[i] = sqrlowerdata[i];
										
			end else if (i < 3*HALFNUMSYMBOLS) begin : genmiddle
				
				// Middle section
				logic [OUTPUTSYMBOLBITWIDTH-1:0] a2b2data;
				assign a2b2data = (i < 2*HALFNUMSYMBOLS) ? sqrlowerdata[i] : sqrupperdata[i - 2*HALFNUMSYMBOLS];			

				sum4to1 #(
					.N(OUTPUTSYMBOLBITWIDTH),
					.USE_DENSE_ADDERS(USE_DENSE_ADDERS)
				) i_sum4to1	(
					.a_in		(~sqrupperdata[i - HALFNUMSYMBOLS]),
					.b_in		(~sqrlowerdata[i - HALFNUMSYMBOLS]),
					.c_in		(a2b2data),	
					.d_in		(sqrmiddledata[i - HALFNUMSYMBOLS]),
					.carry0_in	(1'b1),
					.carry1_in	(1'b1),
					.sum_out	(data_out[i])
				);
						
			end else begin : genlower
			
				// Upper section
				assign data_out[i] = sqrupperdata[i - 2*HALFNUMSYMBOLS];
				
			end		
		end
	end
endgenerate;


endmodule
