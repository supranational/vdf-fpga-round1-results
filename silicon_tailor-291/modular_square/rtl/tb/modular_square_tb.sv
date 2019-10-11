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

module modular_square_tb ();

import vdfpackage::bigmod;

localparam MOD_LEN = 1024;

logic clk = 0;
logic reset;
logic start;

logic [MOD_LEN-1:0]		sq_in;
logic [MOD_LEN-1:0]		sq_out;
logic valid;

// Clock generation 
always #5 clk = ~clk; 

initial begin
	reset = 1;
	start = 0;
	#50 reset = 0;
	#50 start = 1;
	#10 start = 0;
end

assign sq_in = { 16 { 64'h123456789abcdef0 } };


modular_square_metzgen i_modular_square (
	.clk	(clk),
	.reset	(reset),
	.start	(start),
	.sq_in	(sq_in),
	.sq_out	(sq_out),
	.valid	(valid)
	);


/////////////////////////////////////////////////////////
// Test bigmod
//
localparam MODULUS_DEF = 1024'd124066695684124741398798927404814432744698427125735684128131855064976895337309138910015071214657674309443149407457493434579063840841220334555160125016331040933690674569571217337630239191517205721310197608387239846364360850220896772964978569683229449266819903414117058030106528073928633017118689826625594484331;

int i;
initial begin
    $display("M = 0x%h", MODULUS_DEF);
    for (i = 1020; i < 2048; i = i + 1)
        $display("%4d  0x%h", i, bigmod(2048'h1 << i, MODULUS_DEF));
end


endmodule
