/*******************************************************************************
  Copyright 2019 Supranational LLC

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

//`define USE_RAND


module modulo_tb 
#(
	parameter int LOGNUMSYMBOLS = 5,
	parameter int LOGRADIX = 33,
	parameter MODULUS
)
(
	output logic result
);
  
      
import vdfpackage::bigmod;


localparam NUMENCODEDINPUTBITS = ((2 << LOGNUMSYMBOLS)+1) * (2 + LOGRADIX);
localparam NUMUNENCODEDOUTPUTBITS = ((1 << LOGNUMSYMBOLS) * LOGRADIX) + 3;


/////////////////////////////////////////////////////////
// Clock generation 
//
logic clk = 0;
logic rst;

always #5 clk = ~clk; 

initial begin
	rst = 1;
	#50 rst = 0;
end


/////////////////////////////////////////////////////////
// Input data (Random number)
//
logic [NUMENCODEDINPUTBITS-1:0] inputdata = 0;

`ifdef USERAND

randgen #( .N(NUMENCODEDINPUTBITS) ) i_randgen (
	.clk		(clk),
	.rst		(rst),
	.data_out	(inputdata)
);

`else

always_ff @(posedge clk) begin
    if (inputdata == 0) 
        inputdata <= 1;
    else
        inputdata <= inputdata << 1;
end

`endif


/////////////////////////////////////////////////////////
// Unit under test
//
logic [NUMUNENCODEDOUTPUTBITS-1:0] actualoutputdata;
logic [NUMUNENCODEDOUTPUTBITS-1:0] expectedoutputdata;


logic [((2 << LOGNUMSYMBOLS)+1) * LOGRADIX + 3:0] actualinputdata;
logic [((2 << LOGNUMSYMBOLS)+1) * LOGRADIX + 3:0] negActualinputdata;

logic [LOGRADIX+1:0]	encodedinputdata	[(2 << LOGNUMSYMBOLS) + 1];
logic [LOGRADIX-1:0]	modulooutput		[200][1 << LOGNUMSYMBOLS];
logic [LOGRADIX+7:0] 	encodedoutputdata 	[1 << LOGNUMSYMBOLS];
logic [NUMUNENCODEDOUTPUTBITS+7:0] unencodedoutputdata;


generate
	genvar i;
	for (i = 0; i <= (2 << LOGNUMSYMBOLS); ++i) begin
		assign encodedinputdata[i] = inputdata[i * (LOGRADIX + 2) +: (LOGRADIX + 2)];
	end
endgenerate

convertfrommultisymbols #(
	.OUTPUTBITWIDTH			(((2 << LOGNUMSYMBOLS)+1) * LOGRADIX + 4),
	.NUMSYMBOLS 			((2 << LOGNUMSYMBOLS)+1),
	.INPUTSYMBOLBITWIDTH 	(2 + LOGRADIX),
	.LOGRADIX 				(LOGRADIX),
	.SYMBOLS_ARE_SIGNED 	(1)
) i_convertactual (
	.data_in		(encodedinputdata),
	.data_out		(actualinputdata),
	.dataaux_out	()
);

modulolookup #(
	.LOGNUMSYMBOLS			(LOGNUMSYMBOLS),
	.LOGRADIX				(LOGRADIX),
	.MODULUS				(MODULUS)
) i_modulolookup (
	.data_in		(encodedinputdata),
	.data_out		(modulooutput)
);

multisymbolsum200to1 #(
	.LOGNUMSYMBOLS			(LOGNUMSYMBOLS),
	.INPUTSYMBOLBITWIDTH	(LOGRADIX)
) i_multisymbolsum200to1 (
	.vect_in		(modulooutput),
	.sum_out		(encodedoutputdata)
);


convertfrommultisymbols #(
	.OUTPUTBITWIDTH			(NUMUNENCODEDOUTPUTBITS + 8),
	.NUMSYMBOLS 			(1 << LOGNUMSYMBOLS),
	.INPUTSYMBOLBITWIDTH 	(8 + LOGRADIX),
	.LOGRADIX 				(LOGRADIX),
	.SYMBOLS_ARE_SIGNED 	(0)
) i_convertfrommultisymbols (
	.data_in		(encodedoutputdata),
	.data_out		(unencodedoutputdata),
	.dataaux_out	()
);

assign actualoutputdata = bigmod(unencodedoutputdata, MODULUS);


/////////////////////////////////////////////////////////
// Expected result
//
logic inputIsNegative;
assign inputIsNegative = $signed(actualinputdata) < 0;
assign negActualinputdata = 0 - actualinputdata;

assign expectedoutputdata = inputIsNegative ? 
	bigmod(MODULUS - bigmod(negActualinputdata, MODULUS), MODULUS) :
	bigmod(actualinputdata, MODULUS);
	

/////////////////////////////////////////////////////////
// Test actual matches expected
//

always @(posedge clk) begin
	if (rst)
		result <= 1;
	else
		result <= (expectedoutputdata == actualoutputdata);
end


/////////////////////////////////////////////////////////
// Display errors
//
always @(posedge clk) begin
	if (expectedoutputdata != actualoutputdata) begin
   		$display("Error (modulo_tb):");
   		$display("input:              0x%h", inputdata);
   		$display("expectedoutputdata: 0x%h", expectedoutputdata);
   		$display("actualoutputdata:   0x%h", actualoutputdata);
	end	
end


endmodule
