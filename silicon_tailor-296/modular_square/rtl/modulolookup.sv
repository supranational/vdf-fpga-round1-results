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
// Converts a polynomial number data_in (with signed digits)
//   into 200 polynomials (with unsigned digits)
//   such that the (sum of the 200 polynomials) % MODULUS == data_in % MODULUS
//
module modulolookup 
#(
	parameter int NUMSYMBOLS,
	parameter int LOGRADIX,
	parameter MODULUS
) 
(
	input logic [LOGRADIX+1:0]		data_in		[(2*NUMSYMBOLS)+1],		// signed
	output logic [LOGRADIX-1:0]		data_out	[200][NUMSYMBOLS]
);


import vdfpackage::bigmod;

localparam MODBITWIDTH = LOGRADIX * NUMSYMBOLS;

logic [MODBITWIDTH-1:0] adderterms [200];

// signsymbol has a single '1' in the bit location of the sign-bit
//   (used to invert sign bits in symbols)
logic [LOGRADIX+1:0] signsymbol = { 1'b1, { (LOGRADIX+1) { 1'b0 } } };

// generate mask for all sign bit positions
localparam ZERODIGIT = { LOGRADIX { 1'b0 } };
localparam ALLSIGNBITS = { { ((2 * NUMSYMBOLS)+1) { ZERODIGIT | 2'h2 } }, ZERODIGIT };


// Place holder
generate
	genvar i;

	// The first adderterm is the constant correction needed for all the negative symbols
	//    This could be done instead by changing the MOD values for signed bits, but it makes the code easier
	//    to use this constant and then invert all the signed bits.
    assign adderterms[0] = bigmod({ MODULUS, (ALLSIGNBITS ^ ALLSIGNBITS) } - ALLSIGNBITS, MODULUS);

	// The second adderterm is for all the symbols below the modulo-bitwidth (excluding the upper 2 bits per symbol)
	for (i = 0; i < NUMSYMBOLS; i = i + 1) begin : secondterm
		assign adderterms[1][i * LOGRADIX +: LOGRADIX] = data_in[i][LOGRADIX-1:0];
	end

	// The third adderterm is for all the upper bits of each symbol below the modulo-bitwidth
	assign adderterms[2][0 +: LOGRADIX] = 0;
	for (i = 1; i < NUMSYMBOLS; i = i + 1) begin : thirdterm
		assign adderterms[2][i * LOGRADIX +: LOGRADIX] = (data_in[i-1] ^ signsymbol) >> LOGRADIX;
	end

	// The remaining adderterms are for all the bits above the modulo-bitwidth
	for (i = 3; i < 200; i = i + 1) begin : modulolutterm
		
		localparam iter = (i-3)*6 + (2+LOGRADIX)*NUMSYMBOLS - 2;
		
		localparam SYM0 = (iter+0)/(2+LOGRADIX);
		localparam BIT0 = (iter+0)%(2+LOGRADIX);
		if (SYM0 <= (2*NUMSYMBOLS)) begin : gen
		
			localparam SYM1 = (iter+1)/(2+LOGRADIX);
			localparam BIT1 = (iter+1)%(2+LOGRADIX);
			localparam SYM2 = (iter+2)/(2+LOGRADIX);
			localparam BIT2 = (iter+2)%(2+LOGRADIX);
			localparam SYM3 = (iter+3)/(2+LOGRADIX);
			localparam BIT3 = (iter+3)%(2+LOGRADIX);
			localparam SYM4 = (iter+4)/(2+LOGRADIX);
			localparam BIT4 = (iter+4)%(2+LOGRADIX);
			localparam SYM5 = (iter+5)/(2+LOGRADIX);
			localparam BIT5 = (iter+5)%(2+LOGRADIX);
		
			logic x0, x1, x2, x3, x4, x5;
			
			assign x0 = (SYM0 <= (2*NUMSYMBOLS) ? (data_in[SYM0] ^ signsymbol) : 0) >> BIT0;
			assign x1 = (SYM1 <= (2*NUMSYMBOLS) ? (data_in[SYM1] ^ signsymbol) : 0) >> BIT1;
			assign x2 = (SYM2 <= (2*NUMSYMBOLS) ? (data_in[SYM2] ^ signsymbol) : 0) >> BIT2;
			assign x3 = (SYM3 <= (2*NUMSYMBOLS) ? (data_in[SYM3] ^ signsymbol) : 0) >> BIT3;
			assign x4 = (SYM4 <= (2*NUMSYMBOLS) ? (data_in[SYM4] ^ signsymbol) : 0) >> BIT4;
			assign x5 = (SYM5 <= (2*NUMSYMBOLS) ? (data_in[SYM5] ^ signsymbol) : 0) >> BIT5;
			
			localparam MOD0 = bigmod(4096'h1 << ((SYM0 * LOGRADIX) + BIT0), MODULUS);
			localparam MOD1 = bigmod(4096'h1 << ((SYM1 * LOGRADIX) + BIT1), MODULUS);
			localparam MOD2 = bigmod(4096'h1 << ((SYM2 * LOGRADIX) + BIT2), MODULUS);
			localparam MOD3 = bigmod(4096'h1 << ((SYM3 * LOGRADIX) + BIT3), MODULUS);
			localparam MOD4 = bigmod(4096'h1 << ((SYM4 * LOGRADIX) + BIT4), MODULUS);
			localparam MOD5 = bigmod(4096'h1 << ((SYM5 * LOGRADIX) + BIT5), MODULUS);
			
			modulolut6 #(
				.MODULUS		(MODULUS),
				.MODBITWIDTH	(MODBITWIDTH),
				.MOD0			(MOD0),
				.MOD1			(MOD1),
				.MOD2			(MOD2),
				.MOD3			(MOD3),
				.MOD4			(MOD4),
				.MOD5			(MOD5)
			) i_modulolut6 (
				.x0_in			(x0),
				.x1_in			(x1),
				.x2_in			(x2),
				.x3_in			(x3),
				.x4_in			(x4),	
				.x5_in			(x5),	
				.value_out		(adderterms[i])
			);
			
		end else begin : gen0
		
			assign adderterms[i] = 0;
		
		end
		
	end
endgenerate;


//////////////////////////////////////////////////////////////
// Split each adder term into symbols
//

generate
	for (i = 0; i < 200; i = i + 1) begin : symbolsplitter
		converttomultisymbols #(
			.INPUTBITWIDTH 			(MODBITWIDTH),
			.NUMSYMBOLS			    (NUMSYMBOLS),
			.OUTPUTSYMBOLBITWIDTH	(LOGRADIX),
			.LOGRADIX 				(LOGRADIX)
		) i_converttomultisymbols (
			.data_in		(adderterms[i]),
			.data_out		(data_out[i])
		);
	end
endgenerate


endmodule
