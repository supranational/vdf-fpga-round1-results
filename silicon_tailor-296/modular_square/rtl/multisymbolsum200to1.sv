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

module multisymbolsum200to1 
#(
	parameter int LOGNUMSYMBOLS,
	parameter int INPUTSYMBOLBITWIDTH,
	parameter int USE_DENSE_ADDERS
) 
(
	input logic [INPUTSYMBOLBITWIDTH-1:0]   vect_in		[200][1 << LOGNUMSYMBOLS],
	output logic [INPUTSYMBOLBITWIDTH+7:0]  sum_out		[1 << LOGNUMSYMBOLS]
);
	
generate
	genvar i, j;
	for (i = 0; i < (1 << LOGNUMSYMBOLS); i = i + 1) begin : symboladdertree
		
		logic [INPUTSYMBOLBITWIDTH-1:0] symvect [200];
		
		for (j = 0; j < 200; j = j + 1) begin : symboladderterms
	       assign symvect[j] = vect_in[j][i];
		end
		
		sum200to1 #(
			.N		             (INPUTSYMBOLBITWIDTH),
			.USE_DENSE_ADDERS    (USE_DENSE_ADDERS)
		) i_sum200to1 (
			.vect_in	(symvect),
			.sum_out	(sum_out[i])
		);
	end
endgenerate;


endmodule
