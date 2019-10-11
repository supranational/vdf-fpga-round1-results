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

module addertree_tb 
#(
	parameter int NUMBITS = 4
)
(
	output logic result
);



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
logic [200*NUMBITS-1:0] inputdata;

randgen #( .N(200*NUMBITS) ) i_randgen (
	.clk		(clk),
	.rst		(rst),
	.data_out	(inputdata)
);


/////////////////////////////////////////////////////////
// Unit under test
//
logic [NUMBITS-1:0]   	adderterms	[200];
logic [NUMBITS+7:0]  	actualoutputdata;
logic [NUMBITS+7:0]  	expectedoutputdata;

generate
	genvar i;
	for (i = 0; i < 200; ++i) begin
		assign adderterms[i] = inputdata >> (i * NUMBITS);
	end
endgenerate

sum200to1 #(
	.N 	(NUMBITS)
) i_uut (
	.vect_in	(adderterms),
	.sum_out	(actualoutputdata)
);



/////////////////////////////////////////////////////////
// Expected result
//
always_comb begin
	expectedoutputdata = 0;
	for (int j = 0; j < 200; ++j)
		expectedoutputdata = expectedoutputdata + adderterms[j];
end


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
   		$display("Error (addertree_tb):");
   		$display("input:              0x%h", inputdata);
   		$display("expectedoutputdata: 0x%h", expectedoutputdata);
   		$display("actualoutputdata:   0x%h", actualoutputdata);
	end	
end


endmodule
