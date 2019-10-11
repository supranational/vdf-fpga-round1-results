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

package vdfpackage;

	// Helper function to compute (x % m)
	function automatic bit [1023:0] bigmodR(
		bit [4095:0] x, 		// value to modulo
		bit [1024:0] m,			// modulus
		bit [1024:0] _twopowerimodm	//  (2^i) % modulus
		);
		logic [1024:0] term;
		logic [1024:0] twopowerimodm;
		begin
			term = 0;
			twopowerimodm = _twopowerimodm;
			for (integer i = 0; (i < 4); ++i) begin			
                for (integer j = 0; (j < 1024) && ((x >> ((i*1024)+j)) != 0); ++j) begin
                    if (x[(i*1024)+j]) begin
                        term = term + twopowerimodm;
                        if (term >= m)
                            term = term - m;
                    end
                    twopowerimodm = { twopowerimodm, 1'b0 };
                    if (twopowerimodm >= m)
                        twopowerimodm = twopowerimodm - m;
                end
			end
			bigmodR = term;
		end
	endfunction
   
	function bit [1023:0] bigmod(
		bit [4095:0] x, 
		bit [1023:0] m
		);
		bit [1024:0] term;
		begin
			if (m[1023]) begin
				// Speed optimization for 1024-bit Modulo values
				term = x[1022:0] + bigmodR(x >> 1023, {1'b0, m}, 1024'b1 << 1023);
				if (term >= {1'b0, m})
					term = term - m;
				bigmod = term;
			end else begin
				// General case
				bigmod = bigmodR(x, {1'b0, m}, 1);
			end
		end
	endfunction
	
endpackage