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


module sum13to1 
#(
	parameter int N,
	parameter int USE_DENSE_ADDERS
) 
(
	input logic [N-1:0]   	vect_in		[13],
	output logic [N+3:0]  	sum_out
);


generate
	if (USE_DENSE_ADDERS) begin : genDense

		logic [N-1:0] splitMask = { (N+1)/2 { 2'b01 } };


		logic [N+1:0] firststage_sum0;
		logic [N+1:0] firststage_sum1;
		logic [N+1:0] firststage_sum2;
		logic [N+1:0] firststage_sum3;
		logic [N+1:0] firststage_sum4;		
		logic [N+1:0] firststage_half;
		
		assign firststage_half = { 2'b0, vect_in[12] & ~splitMask };

		sum5to2 #( .N(N), .USE_DENSE_ADDERS(USE_DENSE_ADDERS) ) i_firststage0 (
			.a_in		(vect_in[0]),
			.b_in		(vect_in[1]),
			.c_in		(vect_in[2]),
			.d_in		(vect_in[3]),
			.e_in		(vect_in[4]),
			.sumA_out	(firststage_sum0),
			.sumB_out	(firststage_sum1)
		);

		sum5to2 #( .N(N), .USE_DENSE_ADDERS(USE_DENSE_ADDERS) ) i_firststage1 (
			.a_in		(vect_in[5]),
			.b_in		(vect_in[6]),
			.c_in		(vect_in[7]),
			.d_in		(vect_in[8]),
			.e_in		(vect_in[9]),
			.sumA_out	(firststage_sum2),
			.sumB_out	(firststage_sum3)
		);

		add3 #( .N(N) ) i_firststage2 (
			.a_in		(vect_in[10]),
			.b_in		(vect_in[11]),
			.c_in		(vect_in[12] & splitMask),
			.carry_in	(1'b0),
			.sum_out	(firststage_sum4)
		);

		//////////////////////////////////////////////

		logic [N+3:0] secondstage_sum0;
		logic [N+3:0] secondstage_sum1;

		sum5to2 #( .N(N+2), .USE_DENSE_ADDERS(USE_DENSE_ADDERS) ) i_secondstage (
			.a_in		(firststage_sum0),
			.b_in		(firststage_sum1),
			.c_in		(firststage_sum2),
			.d_in		(firststage_sum3),
			.e_in		(firststage_sum4),
			.sumA_out	(secondstage_sum0),
			.sumB_out	(secondstage_sum1)
		);

		//////////////////////////////////////////////

		logic [N+5:0] finalstage_sum;

		add3 #( .N(N+4) ) i_finalstage (
			.a_in		(secondstage_sum0),
			.b_in		(secondstage_sum1),
			.c_in		({ 2'b0, firststage_half }),
			.carry_in	(1'b0),
			.sum_out	(finalstage_sum)
		);

		assign sum_out = finalstage_sum;

	end else begin : genSimple
	
		assign sum_out = 
			{ 4'b0, vect_in[0] } +
			{ 4'b0, vect_in[1] } +
			{ 4'b0, vect_in[2] } +
			{ 4'b0, vect_in[3] } +
			{ 4'b0, vect_in[4] } +
			{ 4'b0, vect_in[5] } +
			{ 4'b0, vect_in[6] } +
			{ 4'b0, vect_in[7] } +
			{ 4'b0, vect_in[8] } +
			{ 4'b0, vect_in[9] } +
			{ 4'b0, vect_in[10] } +
			{ 4'b0, vect_in[11] } +
			{ 4'b0, vect_in[12] };
	
	end
endgenerate;

endmodule
