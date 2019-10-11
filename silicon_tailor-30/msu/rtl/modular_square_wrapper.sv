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

`ifndef MOD_LEN_DEF
`define MOD_LEN_DEF 1024
`endif
`ifndef MODULUS_DEF
 `define MODULUS_DEF 1024'd124066695684124741398798927404814432744698427125735684128131855064976895337309138910015071214657674309443149407457493434579063840841220334555160125016331040933690674569571217337630239191517205721310197608387239846364360850220896772964978569683229449266819903414117058030106528073928633017118689826625594484331
`endif

module modular_square_wrapper
#(
	parameter int MOD_LEN = `MOD_LEN_DEF,
	parameter MODULUS = `MODULUS_DEF,
	parameter USE_DENSE_ADDERS = 1
)
(
	input logic                     clk,
	input logic                     reset,
	input logic                     start,
	input logic [MOD_LEN-1:0]       sq_in,
	output logic [MOD_LEN-1:0] 	    sq_out,
	output logic                    valid
);

 	modular_square_metzgen_wrapper      // Uses Clock-domain crossing
	//modular_square_metzgen                 // Single Clock
	#(
		.MOD_LEN  (MOD_LEN),
		.MODULUS  (MODULUS),
		.USE_DENSE_ADDERS (USE_DENSE_ADDERS)
	)
	modsqr(
		.clk        (clk),
		.reset      (reset),
		.start      (start),
		.sq_in      (sq_in),
		.sq_out     (sq_out),
		.valid      (valid)
	);

endmodule
