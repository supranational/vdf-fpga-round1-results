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
// Implements an unsigned square operation
//   data_out = data_in * data_in
//
module squarer 
#(
	parameter int INPUTWIDTH		// must be 42 or less
) 
(
	input logic [INPUTWIDTH-1:0]		data_in,
	
	output logic [2*INPUTWIDTH-1:0]  	data_out
);


generate
	if (INPUTWIDTH <= 34) begin : gen34

		logic [33:0] data;
		logic [67:0] result;
		assign data = data_in;

		square34 i_square34 (
			.data_in    (data),	
			.data_out   (result)
		);

		assign data_out = result;

	end else if (INPUTWIDTH <= 42) begin : gen42

	   logic [41:0] data;
	   logic [83:0] result;
	   assign data = data_in;

	   square42 i_square42 (
			.data_in    (data),	
			.data_out   (result)
		);

		assign data_out = result;

	end 
endgenerate

endmodule
