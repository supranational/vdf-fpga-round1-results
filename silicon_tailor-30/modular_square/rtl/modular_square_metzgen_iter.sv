/*******************************************************************************
  Copyright	2019 Supranational LLC

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

//`define DEBUG


module modular_square_metzgen_iter
#(
	parameter int	MOD_LEN,
	parameter 		MODULUS,
	parameter int   USE_DENSE_ADDERS
)
(
	input	logic					clk,
	input	logic					reset,
	input	logic					start,
	
	// Input value to be squared
	input	logic [MOD_LEN-1:0]		sq_in,
	
	// The output value is (sqa_out + sqb_out) % MODULUS
	output logic [MOD_LEN+34:0]		sqa_out,
	output logic [MOD_LEN+34:0]		sqb_out,
	
	output logic					valid
);

	import vdfpackage::bigmod;


	///////////////////////////////////////////////////////////////////////////
	// Control Logic
	//
	
	logic running = 0;

	//	Control
	always_ff @(posedge clk) begin
		if (reset) begin
			running <= 0;
			valid <= 0;
		end else if (start) begin
			running <= 1;
			valid <= 0;
		end else begin
			valid <= running;
		end
	end
		  
	///////////////////////////////////////////////////////////////////////////
	// Iterative Loop logic
	//
	
	localparam int LOGNUMSYMBOLS = 5;
	localparam int LOGRADIX = 32+1;
	
	logic [LOGRADIX:0] codedvalue		[1 << LOGNUMSYMBOLS];
	logic [LOGRADIX:0] nextcodedvalue	[1 << LOGNUMSYMBOLS];
	
	localparam int SQUAREROUTPUTSYMBOLWIDTH = 2*(1 + LOGRADIX) - LOGRADIX + 4*LOGNUMSYMBOLS + 1;
	logic [SQUAREROUTPUTSYMBOLWIDTH-1:0] 	squareroutput	[2 << LOGNUMSYMBOLS];
	logic [LOGRADIX+1:0] 					moduloinput		[(2 << LOGNUMSYMBOLS)+1];
	logic [LOGRADIX-1:0] 					modulooutput	[200][1 << LOGNUMSYMBOLS];
	logic [LOGRADIX+7:0] 					addertreesum	[1 << LOGNUMSYMBOLS];
	logic [LOGRADIX+7:0]					codedloadvalue	[1 << LOGNUMSYMBOLS];
	
	
	multisymbolsquarer #(
		.LOGNUMSYMBOLS			(LOGNUMSYMBOLS),
		.INPUTSYMBOLBITWIDTH	(1 + LOGRADIX),
		.LOGRADIX				(LOGRADIX),
		.USE_DENSE_ADDERS       (USE_DENSE_ADDERS)
	) i_multisymbolsquarer (
		.clk			(clk),
		.data_in		(codedvalue),
		.data_out		(squareroutput)
	);
	
	signedcarrycorrection #(
		.NUMSYMBOLS			    (2 << LOGNUMSYMBOLS),
		.INPUTSYMBOLBITWIDTH	(SQUAREROUTPUTSYMBOLWIDTH),
		.LOGRADIX				(LOGRADIX)
	) i_signedcarrycorrection (
		.data_in		(squareroutput),
		.data_out		(moduloinput)
	);
	
	modulolookup #(
		.NUMSYMBOLS		     	(1 << LOGNUMSYMBOLS),
		.LOGRADIX				(LOGRADIX),
		.MODULUS				(MODULUS)
	) i_modulolookup (
		.data_in		(moduloinput),
		.data_out		(modulooutput)
	);
	
	multisymbolsum200to1 #(
		.LOGNUMSYMBOLS			(LOGNUMSYMBOLS),
		.INPUTSYMBOLBITWIDTH	(LOGRADIX),
		.USE_DENSE_ADDERS       (USE_DENSE_ADDERS)
	) i_multisymbolsum200to1 (
		.vect_in		(modulooutput),
		.sum_out		(addertreesum)
	);

	converttomultisymbols #(
	   .INPUTBITWIDTH			(1024),
	   .NUMSYMBOLS			    (1 << LOGNUMSYMBOLS),
	   .OUTPUTSYMBOLBITWIDTH	(8 + LOGRADIX),
	   .LOGRADIX				(LOGRADIX)
	) i_converttomultisymbols (
		.data_in		(sq_in),
		.data_out		(codedloadvalue)
	);

	unsignedcarrycorrection #(
		.NUMSYMBOLS			    (1 << LOGNUMSYMBOLS),
		.INPUTSYMBOLBITWIDTH	(8 + LOGRADIX),
		.LOGRADIX				(LOGRADIX)
	) i_unsignedcarrycorrection (
		.data_in		(start ? codedloadvalue : addertreesum),
		.data_out		(nextcodedvalue)
	);

    integer k;
	always_ff @(posedge clk) begin
	   for (k = 0; k < (1 << LOGNUMSYMBOLS); k = k + 1) begin
			//codedvalue[k] <= 0;
	       //if (k < 4)
   	       codedvalue[k] <= nextcodedvalue[k];
    	end
	end


	///////////////////////////////////////////////////////////////////////////
	// Output logic
	//   A dual output is used to avoid a long carry chain in this SLR 
	//   that could impact on place-and-route performance
	//
    convertfrommultisymbols #(
       .OUTPUTBITWIDTH			(MOD_LEN+35),
       .NUMSYMBOLS			    (1 << LOGNUMSYMBOLS),
       .INPUTSYMBOLBITWIDTH		(1 + LOGRADIX),
       .LOGRADIX				(LOGRADIX),
       .SYMBOLS_ARE_SIGNED		(0),
       .USEDUALOUTPUT           (1)
    ) i_convertfrommultisymbols (
        .data_in		(codedvalue),
        .data_out		(sqa_out),
        .dataaux_out	(sqb_out)
    );


	///////////////////////////////////////////////////////////////////////////
	// Debug logic
	//
`ifdef DEBUG
	logic [1024 + 64 - 1:0] dbg_sqrin;
	logic [2048 + 128 - 1:0] dbg_sqrresult;
	logic [2048 + 128 - 1:0] dbg_sqrrawresult;
	logic [1024 + 64 - 1:0] dbg_modtreeout;
	logic [1024 + 64 - 1:0] dbg_modout;
	
	convertfrommultisymbols #(
	   .OUTPUTBITWIDTH			(1024 + 64),
	   .NUMSYMBOLS			    (1 << LOGNUMSYMBOLS),
	   .INPUTSYMBOLBITWIDTH		(LOGRADIX + 1),
	   .LOGRADIX				(LOGRADIX),
	   .SYMBOLS_ARE_SIGNED		(0)
	) i_dbg_sqrinput (
		.data_in		(codedvalue),
		.data_out		(dbg_sqrin)
	);

	convertfrommultisymbols #(
	   .OUTPUTBITWIDTH			(2048 + 128),
	   .NUMSYMBOLS			    (2 << LOGNUMSYMBOLS),
	   .INPUTSYMBOLBITWIDTH		(SQUAREROUTPUTSYMBOLWIDTH),
	   .LOGRADIX				(LOGRADIX),
	   .SYMBOLS_ARE_SIGNED		(1)
	) i_dbg_sqrrawresult (
		.data_in		(squareroutput),
		.data_out		(dbg_sqrrawresult)
	);
	
	convertfrommultisymbols #(
	   .OUTPUTBITWIDTH			(2048 + 128),
	   .NUMSYMBOLS			    ((2 << LOGNUMSYMBOLS)+1),
	   .INPUTSYMBOLBITWIDTH		(LOGRADIX + 2),
	   .LOGRADIX				(LOGRADIX),
	   .SYMBOLS_ARE_SIGNED		(1)
	) i_dbg_sqrresult (
		.data_in		(moduloinput),
		.data_out		(dbg_sqrresult)
	);
	
	convertfrommultisymbols #(
		.OUTPUTBITWIDTH			(1024 + 64),
		.NUMSYMBOLS 			(1 << LOGNUMSYMBOLS),
		.INPUTSYMBOLBITWIDTH 	(8 + LOGRADIX),
		.LOGRADIX 				(LOGRADIX),
		.SYMBOLS_ARE_SIGNED 	(0)
	) i_dbg_modtreeoutput (
		.data_in		(addertreesum),
		.data_out		(dbg_modtreeout),
		.dataaux_out	()
	);

	convertfrommultisymbols #(
	   .OUTPUTBITWIDTH			(1024 + 64),
	   .NUMSYMBOLS			    (1 << LOGNUMSYMBOLS),
	   .INPUTSYMBOLBITWIDTH		(LOGRADIX + 1),
	   .LOGRADIX				(LOGRADIX),
	   .SYMBOLS_ARE_SIGNED		(0)
	) i_dbg_modoutput (
		.data_in		(nextcodedvalue),
		.data_out		(dbg_modout)
	);

	logic [1024 + 64 - 1:0] dbg_sqrin_normalized;
	logic [1024 + 64 - 1:0] dbg_sqrin_squared_normalized;
	logic [1024 + 64 - 1:0] dbg_sqrrawresult_normalized;
	logic [1024 + 64 - 1:0] dbg_sqrresult_normalized;
	logic [1024 + 64 - 1:0] dbg_modtreeout_normalized;
	logic [1024 + 64 - 1:0] dbg_modout_normalized;
	assign dbg_sqrin_normalized = bigmod(dbg_sqrin, MODULUS);
	assign dbg_sqrin_squared_normalized = bigmod(dbg_sqrin * dbg_sqrin, MODULUS);
	assign dbg_sqrrawresult_normalized = bigmod(dbg_sqrrawresult, MODULUS);
	assign dbg_sqrresult_normalized = bigmod(dbg_sqrresult, MODULUS);
	assign dbg_modtreeout_normalized = bigmod(dbg_modtreeout, MODULUS);
	assign dbg_modout_normalized = bigmod(dbg_modout, MODULUS);
	
	logic dbgchk_sqrin_raw;
	logic dbgchk_sqrresult_raw;
	logic dbgchk_modtreeresult;
	logic dbgchk_modresult;
	assign dbgchk_sqrin_raw = (dbg_sqrin_squared_normalized == dbg_sqrrawresult_normalized);
	assign dbgchk_sqrresult_raw = (dbg_sqrresult_normalized == dbg_sqrrawresult_normalized);
	assign dbgchk_modtreeresult = (dbg_modtreeout_normalized == dbg_sqrresult_normalized);
	assign dbgchk_modresult = (dbg_modtreeout_normalized == dbg_modout_normalized);

`endif


endmodule
