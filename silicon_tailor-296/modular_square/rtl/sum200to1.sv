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


//`define USESIMPLEIMPL


//
// Sum 200 values together
//
module sum200to1 
#(
	parameter int N,
	parameter int USE_DENSE_ADDERS
) 
(
	input logic [N-1:0]   	vect_in	[200],
	output logic [N+7:0]  	sum_out
);


`ifdef USESIMPLEIMPL

integer k;
always_comb begin
    sum_out = 0;
       for (k = 0; k < 200; k = k + 1) begin
          sum_out = sum_out + { 8'b0, vect_in[k] };
    end
end

`else

logic [N+1:0] firststage_sums	[80];
logic [N+3:0] secondstage_sums	[32];
logic [N+5:0] thirdstage_sums	[13];

genvar i;
generate
	for (i = 0; i < 200; i = i + 5) begin : firststage_compress
		sum5to2 #( .N(N), .USE_DENSE_ADDERS(USE_DENSE_ADDERS) ) i_sum5to2 (
			.a_in		(vect_in[i+0]),
			.b_in		(vect_in[i+1]),
			.c_in		(vect_in[i+2]),
			.d_in		(vect_in[i+3]),
			.e_in		(vect_in[i+4]),
			.sumA_out	(firststage_sums[(2*(i/5))+0]),
			.sumB_out	(firststage_sums[(2*(i/5))+1])
		);		
	end
endgenerate;

generate
	for (i = 0; i < 80; i = i + 5) begin : secondstage_compress
		sum5to2 #( .N(N+2), .USE_DENSE_ADDERS(USE_DENSE_ADDERS) ) i_sum5to2 (
			.a_in		(firststage_sums[i+0]),
			.b_in		(firststage_sums[i+1]),
			.c_in		(firststage_sums[i+2]),
			.d_in		(firststage_sums[i+3]),
			.e_in		(firststage_sums[i+4]),
			.sumA_out	(secondstage_sums[(2*(i/5))+0]),
			.sumB_out	(secondstage_sums[(2*(i/5))+1])
		);		
	end
endgenerate;

generate
	for (i = 0; i < 30; i = i + 5) begin : thirdstage_compress
		sum5to2 #( .N(N+4), .USE_DENSE_ADDERS(USE_DENSE_ADDERS) ) i_sum5to2 (
			.a_in		(secondstage_sums[i+0]),
			.b_in		(secondstage_sums[i+1]),
			.c_in		(secondstage_sums[i+2]),
			.d_in		(secondstage_sums[i+3]),
			.e_in		(secondstage_sums[i+4]),
			.sumA_out	(thirdstage_sums[2*(i/5)+0]),
			.sumB_out	(thirdstage_sums[2*(i/5)+1])
		);		
	end
endgenerate;

assign thirdstage_sums[12] = 
	{ 2'b0, secondstage_sums[30] } + 
	{ 2'b0, secondstage_sums[31] };


logic [N+9:0] finalstage_sum;

sum13to1 #(	.N(N+6), .USE_DENSE_ADDERS(USE_DENSE_ADDERS) ) i_sum13to1 (
	.vect_in	(thirdstage_sums),
	.sum_out	(finalstage_sum)
);


assign sum_out = finalstage_sum;

`endif



endmodule
