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

module randgen
#(
	parameter int N
)
(
	input logic 			clk,
	input logic 			rst,
	output logic [N-1:0] 	data_out
);


logic [4095:0] state;
logic [4095:0] initialstate;


function [127:0] xorshift128p;
	input [127:0] x;
	logic [63:0] s, t;

	s = x;
	t = x >> 64;
	
	t = t ^ (t << 23);
	t = t ^ (t >> 17);
	t = t ^ s ^ (s >> 26);
	
	xorshift128p = { s, t };

endfunction


generate
	genvar i;
	for (i = 0; i < 4096; i = i + 128) begin	
		
		always_comb begin
			initialstate[i +: 128] <= (i/128) + 1;
		end
		
		always @(posedge clk) begin
			if (rst)
				state <= 0;
			else if (state == 0)
				state <= initialstate;
			else begin
				state[i +: 128] <= xorshift128p(state[i +: 128]);
			end
		end
		
	end
endgenerate


// Ensure 0 is first value out
assign data_out = (state ^ initialstate);


endmodule
