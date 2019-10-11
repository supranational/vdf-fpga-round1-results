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
// This is achieved by splitting data_in as follows:
//
//    data_in[41:34][33:17][16:0 ]
//              \/    \/     \/
//           [  A  ][  B  ][  C  ]
//
//
//                   [ A ][ B ][ C ]
//                 x [ A ][ B ][ C ]
//   ===============================
//                        [  C*C   ]
//              [  2*{A,B}*C  ][ 0 ]
// +  [   {A,B}*{A,B}    ][ 0 ][ 0 ]
//   ===============================
//                        [  C*C   ]
//              [  2*{A,B}*C  ][ 0 ]
//         [  {A,B}*B    ][ 0 ][ 0 ]
// +  [  {A,B}*A    ][ 0 ][ 0 ][ 0 ]
//   ===============================
//   [          data_out           ]
//
module square42
(
	input logic [41:0] 		data_in,	
	output logic [83:0]		data_out
);


logic [29:0] dataA  [0:3];
logic [17:0] dataB  [0:3];
logic [47:0] dataC  [0:3];

logic [47:0] result	[0:3];
logic [47:0] pcout	[0:3];


assign dataA[0] = data_in[16:0];				// C
assign dataB[0] = data_in[16:0];				// C
assign dataC[0] = 0;

assign dataA[1] = { data_in[41:17], 1'b0 };		// 2*{A,B}
assign dataB[1] = data_in[16:0];				// C
assign dataC[1] = 0;

assign dataA[2] = data_in[41:17];				// {A,B}
assign dataB[2] = data_in[33:17];				// B
assign dataC[2] = 0;

assign dataA[3] = data_in[41:17];				// {A,B}
assign dataB[3] = data_in[41:34];				// A
assign dataC[3] = 0;


xilinxdspmuladd i_dsp0 (
	.dataA				(dataA[0]),
	.dataB				(dataB[0]),
	.dataC				(dataC[0]),

	.pcin				(48'b0),

	.doAddC				(1'b0),
	.doAddPcin			(1'b0),
	.doAddPcinShifted	(1'b0),

	.result				(result[0]),
	.pcout				(pcout[0])
);

xilinxdspmuladd i_dsp1 (
	.dataA				(dataA[1]),
	.dataB				(dataB[1]),
	.dataC				(dataC[1]),

	.pcin				(pcout[0]),

	.doAddC				(1'b0),
	.doAddPcin			(1'b0),
	.doAddPcinShifted	(1'b1),

	.result				(result[1]),
	.pcout				(pcout[1])
);

xilinxdspmuladd i_dsp2 (
	.dataA				(dataA[2]),
	.dataB				(dataB[2]),
	.dataC				(dataC[2]),

	.pcin				(pcout[1]),

	.doAddC				(1'b0),
	.doAddPcin			(1'b0),
	.doAddPcinShifted	(1'b1),

	.result				(result[2]),
	.pcout				(pcout[2])
);

xilinxdspmuladd i_dsp3 (
	.dataA				(dataA[3]),
	.dataB				(dataB[3]),
	.dataC				(dataC[3]),

	.pcin				(pcout[2]),

	.doAddC				(1'b0),
	.doAddPcin			(1'b0),
	.doAddPcinShifted	(1'b1),

	.result				(result[3]),
	.pcout				(pcout[3])
);


assign data_out = { result[3], result[2][16:0], result[1][16:0], result[0][16:0] };



endmodule