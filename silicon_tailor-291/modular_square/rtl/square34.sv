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
//    data_in[31:17][16:0 ]
//              \/    \/
//           [  A  ][  B  ]
//
//
//               [ A ][ B ]
//             x [ A ][ B ]
//   ======================
//               [  B*B   ]
// +      [  2*A*B   ][ 0 ]
// +   [  A*A   ][ 0 ][ 0 ]
//   ======================
//    [     data_out      ]
//
module square34
(
	input logic [33:0] 		data_in,	
	output logic [67:0]		data_out
);


logic [29:0] dataA  [0:2];
logic [17:0] dataB  [0:2];
logic [47:0] dataC  [0:2];

logic [47:0] result	[0:2];
logic [47:0] pcout	[0:2];


assign dataA[0] = data_in[16:0];				// B
assign dataB[0] = data_in[16:0];				// B
assign dataC[0] = 0;

assign dataA[1] = { data_in[33:17], 1'b0 };		// 2*A
assign dataB[1] = data_in[16:0];				// B
assign dataC[1] = 0;

assign dataA[2] = data_in[33:17];				// A
assign dataB[2] = data_in[33:17];				// A
assign dataC[2] = 0;


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


assign data_out = { result[2], result[1][16:0], result[0][16:0] };



endmodule