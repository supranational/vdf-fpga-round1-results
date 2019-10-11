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

module uconvert_tb 
#(
	parameter int LOGNUMSYMBOLS = 5,
	parameter int LOGRADIX = 33
)
(
	output logic result
);


localparam NUMUNENCODEDBITS = (1 << LOGNUMSYMBOLS) * LOGRADIX;


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
logic [NUMUNENCODEDBITS-1:0] inputdata;

randgen #( .N(NUMUNENCODEDBITS) ) i_randgen (
	.clk		(clk),
	.rst		(rst),
	.data_out	(inputdata)
);


/////////////////////////////////////////////////////////
// Unit under test
//
logic [NUMUNENCODEDBITS-1:0] actualoutputdata;
logic [NUMUNENCODEDBITS-1:0] expectedoutputdata;

logic [LOGRADIX:0]		encodedinputdata [1 << LOGNUMSYMBOLS];
logic [LOGRADIX:0]		encodedoutputdata [1 << LOGNUMSYMBOLS];

converttomultisymbols #(
	.INPUTBITWIDTH			(NUMUNENCODEDBITS),
	.NUMSYMBOLS 			(1 << LOGNUMSYMBOLS),
	.OUTPUTSYMBOLBITWIDTH 	(1 + LOGRADIX),
	.LOGRADIX 				(LOGRADIX)
) i_converttomultisymbols (
	.data_in	(inputdata),
	.data_out	(encodedinputdata)
);

generate
	genvar i;
	for (i = 0; i < (1 << LOGNUMSYMBOLS); ++i) begin
		assign encodedoutputdata[i] = encodedinputdata[i] << 1;
	end
endgenerate

convertfrommultisymbols #(
	.OUTPUTBITWIDTH			(NUMUNENCODEDBITS),
	.NUMSYMBOLS 			(1 << LOGNUMSYMBOLS),
	.INPUTSYMBOLBITWIDTH 	(1 + LOGRADIX),
	.LOGRADIX 				(LOGRADIX),
	.SYMBOLS_ARE_SIGNED 	(0)
) i_convertfrommultisymbols (
	.data_in		(encodedoutputdata),
	.data_out		(actualoutputdata),
	.dataaux_out	()
);



/////////////////////////////////////////////////////////
// Expected result
//
assign expectedoutputdata = inputdata << 1;



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
   		$display("Error (uconvert_tb):");
   		$display("input:              0x%h", inputdata);
   		$display("expectedoutputdata: 0x%h", expectedoutputdata);
   		$display("actualoutputdata:   0x%h", actualoutputdata);
	end	
end


endmodule
